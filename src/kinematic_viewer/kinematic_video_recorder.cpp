#include "kinematic_viewer/kinematic_video_recorder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sstream>

namespace kinematic_viewer {

    namespace {

        constexpr size_t MAX_QUEUE_SIZE = 4;

    }  // namespace

    VideoRecorder::VideoRecorder() = default;

    VideoRecorder::~VideoRecorder() {
        if (is_recording_.load()) {
            StopRecording();
        }
    }

    std::string VideoRecorder::DefaultOutputDirectory() {
        const char* home = std::getenv("HOME");
        if (home == nullptr || home[0] == '\0') {
            home = ".";
        }
        std::string dir = std::string(home) + "/robot_kinematic_viewer_recordings";
        std::filesystem::create_directories(dir);
        return dir;
    }

    std::string VideoRecorder::TimestampString() {
        auto now = std::time(nullptr);
        auto tm  = *std::localtime(&now);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
        return std::string(buf);
    }

    bool VideoRecorder::StartRecording(const std::string& output_path, int width, int height, int fps, Format format) {
        if (is_recording_.load()) {
            return false;
        }

        format_     = format;
        target_fps_ = std::max(1, fps);
        frame_count_.store(0);
        dropped_frames_.store(0);

        bool ok = false;
        if (format == Format::MP4) {
            ok = InitMp4(output_path, width, height, target_fps_);
        } else {
            ok = InitGif(output_path, width, height, target_fps_);
        }

        if (!ok) {
            return false;
        }

        is_recording_.store(true);
        encoder_stop_requested_ = false;
        encoder_thread_         = std::thread(&VideoRecorder::EncoderThreadLoop, this);
        return true;
    }

    void VideoRecorder::StopRecording() {
        if (!is_recording_.load()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            encoder_stop_requested_ = true;
        }
        queue_cv_.notify_all();

        if (encoder_thread_.joinable()) {
            encoder_thread_.join();
        }

        if (format_ == Format::MP4) {
            FinalizeMp4();
        } else {
            FinalizeGif();
        }

        is_recording_.store(false);
    }

    bool VideoRecorder::IsRecording() const {
        return is_recording_.load();
    }

    void VideoRecorder::SubmitFrame(const uint8_t* rgba_data, int width, int height) {
        if (!is_recording_.load()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (frame_queue_.size() >= MAX_QUEUE_SIZE) {
                dropped_frames_.fetch_add(1);
                return;
            }
            FrameBuffer fb;
            fb.width  = width;
            fb.height = height;
            fb.data.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
            std::memcpy(fb.data.data(), rgba_data, fb.data.size());
            frame_queue_.push(std::move(fb));
        }
        queue_cv_.notify_one();
    }

    std::string VideoRecorder::GetStatus() const {
        if (!is_recording_.load()) {
            return "未录制";
        }
        std::ostringstream oss;
        oss << (format_ == Format::MP4 ? "录制 MP4" : "录制 GIF") << ": " << frame_count_.load() << " 帧";
        int dropped = dropped_frames_.load();
        if (dropped > 0) {
            oss << " (丢弃 " << dropped << " 帧)";
        }
        return oss.str();
    }

    void VideoRecorder::EncoderThreadLoop() {
        while (true) {
            FrameBuffer fb;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || encoder_stop_requested_; });
                if (frame_queue_.empty() && encoder_stop_requested_) {
                    break;
                }
                if (!frame_queue_.empty()) {
                    fb = std::move(frame_queue_.front());
                    frame_queue_.pop();
                }
            }
            if (!fb.data.empty()) {
                if (format_ == Format::MP4) {
                    EncodeMp4Frame(fb.data.data(), fb.width, fb.height);
                } else {
                    EncodeGifFrame(fb.data.data(), fb.width, fb.height);
                }
                frame_count_.fetch_add(1);
            }
        }
    }

    // ---------------------------------------------------------------------------
    // MP4 / FFmpeg implementation
    // ---------------------------------------------------------------------------

    bool VideoRecorder::InitMp4(const std::string& path, int width, int height, int fps) {
        avformat_alloc_output_context2(&fmt_ctx_, nullptr, nullptr, path.c_str());
        if (!fmt_ctx_) {
            std::fprintf(stderr, "[VideoRecorder] Could not allocate output context\n");
            return false;
        }

        const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec) {
            std::fprintf(stderr, "[VideoRecorder] H264 encoder not found\n");
            return false;
        }

        video_stream_ = avformat_new_stream(fmt_ctx_, nullptr);
        if (!video_stream_) {
            std::fprintf(stderr, "[VideoRecorder] Could not create stream\n");
            return false;
        }

        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_) {
            std::fprintf(stderr, "[VideoRecorder] Could not allocate codec context\n");
            return false;
        }

        codec_ctx_->width        = width;
        codec_ctx_->height       = height;
        codec_ctx_->time_base    = AVRational{1, fps};
        codec_ctx_->framerate    = AVRational{fps, 1};
        codec_ctx_->pix_fmt      = AV_PIX_FMT_YUV420P;
        codec_ctx_->gop_size     = fps;
        codec_ctx_->max_b_frames = 2;

        if (fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER) {
            codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        // Use medium preset for speed/quality balance
        av_opt_set(codec_ctx_->priv_data, "preset", "medium", 0);
        av_opt_set(codec_ctx_->priv_data, "tune", "zerolatency", 0);

        if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
            std::fprintf(stderr, "[VideoRecorder] Could not open codec\n");
            return false;
        }

        if (avcodec_parameters_from_context(video_stream_->codecpar, codec_ctx_) < 0) {
            std::fprintf(stderr, "[VideoRecorder] Could not copy codec params\n");
            return false;
        }
        video_stream_->time_base = codec_ctx_->time_base;

        if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&fmt_ctx_->pb, path.c_str(), AVIO_FLAG_WRITE) < 0) {
                std::fprintf(stderr, "[VideoRecorder] Could not open output file: %s\n", path.c_str());
                return false;
            }
        }

        if (avformat_write_header(fmt_ctx_, nullptr) < 0) {
            std::fprintf(stderr, "[VideoRecorder] Could not write header\n");
            return false;
        }

        yuv_frame_ = av_frame_alloc();
        if (!yuv_frame_) {
            return false;
        }
        yuv_frame_->format = AV_PIX_FMT_YUV420P;
        yuv_frame_->width  = width;
        yuv_frame_->height = height;
        if (av_frame_get_buffer(yuv_frame_, 32) < 0) {
            return false;
        }

        packet_ = av_packet_alloc();
        if (!packet_) {
            return false;
        }

        sws_ctx_ =
            sws_getContext(width, height, AV_PIX_FMT_RGBA, width, height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!sws_ctx_) {
            return false;
        }

        next_pts_ = 0;
        return true;
    }

    void VideoRecorder::FinalizeMp4() {
        if (!codec_ctx_) {
            return;
        }
        // Flush encoder
        int ret = avcodec_send_frame(codec_ctx_, nullptr);
        while (ret >= 0) {
            ret = avcodec_receive_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            }
            if (ret < 0) {
                break;
            }
            av_packet_rescale_ts(packet_, codec_ctx_->time_base, video_stream_->time_base);
            packet_->stream_index = video_stream_->index;
            av_interleaved_write_frame(fmt_ctx_, packet_);
            av_packet_unref(packet_);
        }

        av_write_trailer(fmt_ctx_);

        if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&fmt_ctx_->pb);
        }

        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
        av_frame_free(&yuv_frame_);
        av_packet_free(&packet_);
        avcodec_free_context(&codec_ctx_);
        avformat_free_context(fmt_ctx_);
        fmt_ctx_      = nullptr;
        video_stream_ = nullptr;
    }

    void VideoRecorder::EncodeMp4Frame(const uint8_t* rgba_data, int width, int height) {
        if (!sws_ctx_ || !yuv_frame_ || !codec_ctx_) {
            return;
        }

        const uint8_t* src_slice[1] = {rgba_data};
        int src_stride[1]           = {width * 4};
        sws_scale(sws_ctx_, src_slice, src_stride, 0, height, yuv_frame_->data, yuv_frame_->linesize);

        yuv_frame_->pts = next_pts_++;

        int ret = avcodec_send_frame(codec_ctx_, yuv_frame_);
        if (ret < 0) {
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_packet(codec_ctx_, packet_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            }
            if (ret < 0) {
                break;
            }
            av_packet_rescale_ts(packet_, codec_ctx_->time_base, video_stream_->time_base);
            packet_->stream_index = video_stream_->index;
            av_interleaved_write_frame(fmt_ctx_, packet_);
            av_packet_unref(packet_);
        }
    }

    // ---------------------------------------------------------------------------
    // GIF / libgif implementation
    // ---------------------------------------------------------------------------

    bool VideoRecorder::InitGif(const std::string& path, int width, int height, int fps) {
        gif_file_ = EGifOpenFileName(path.c_str(), false, nullptr);
        if (!gif_file_) {
            std::fprintf(stderr, "[VideoRecorder] Could not open GIF file: %s\n", path.c_str());
            return false;
        }

        // GIF89a
        EGifSetGifVersion(gif_file_, true);

        gif_color_resolution_       = 8;
        gif_frame_delay_hundredths_ = std::max(1, static_cast<int>(100.0 / fps + 0.5));

        // Write logical screen descriptor (placeholder palette, will be updated on first frame)
        if (EGifPutScreenDesc(gif_file_, width, height, gif_color_resolution_, 0, nullptr) == GIF_ERROR) {
            std::fprintf(stderr, "[VideoRecorder] Could not write GIF screen desc\n");
            return false;
        }

        // Loop forever extension
        {
            unsigned char loop_ext[19] = {'N', 'E', 'T', 'S', 'C', 'A', 'P', 'E', '2', '.', '0', 0x03, 0x01, 0x00, 0x00, 0x00};
            if (EGifPutExtensionLeader(gif_file_, APPLICATION_EXT_FUNC_CODE) == GIF_ERROR) {
                return false;
            }
            if (EGifPutExtensionBlock(gif_file_, 11, loop_ext) == GIF_ERROR) {
                return false;
            }
            if (EGifPutExtensionBlock(gif_file_, 3, loop_ext + 11) == GIF_ERROR) {
                return false;
            }
            if (EGifPutExtensionTrailer(gif_file_) == GIF_ERROR) {
                return false;
            }
        }

        return true;
    }

    void VideoRecorder::FinalizeGif() {
        if (gif_file_) {
            EGifCloseFile(gif_file_, nullptr);
            gif_file_ = nullptr;
        }
    }

    void VideoRecorder::EncodeGifFrame(const uint8_t* rgba_data, int width, int height) {
        if (!gif_file_) {
            return;
        }

        std::vector<uint8_t> indices;
        std::vector<GifColorType> palette;
        QuantizeToPalette(rgba_data, width, height, &indices, &palette);

        // Graphics control extension for frame delay
        {
            GraphicsControlBlock gcb;
            gcb.DisposalMode     = DISPOSE_DO_NOT;
            gcb.UserInputFlag    = false;
            gcb.TransparentColor = NO_TRANSPARENT_COLOR;
            gcb.DelayTime        = gif_frame_delay_hundredths_;

            unsigned char gce[4];
            gce[0] = 0;
            gce[0] |= (gcb.DisposalMode << 2);
            gce[0] |= (gcb.UserInputFlag ? 0x02 : 0x00);
            gce[0] |= (gcb.TransparentColor != NO_TRANSPARENT_COLOR ? 0x01 : 0x00);
            gce[1] = static_cast<unsigned char>(gcb.DelayTime & 0xFF);
            gce[2] = static_cast<unsigned char>((gcb.DelayTime >> 8) & 0xFF);
            gce[3] = (gcb.TransparentColor != NO_TRANSPARENT_COLOR) ? static_cast<unsigned char>(gcb.TransparentColor) : 0;

            if (EGifPutExtension(gif_file_, GRAPHICS_EXT_FUNC_CODE, 4, gce) == GIF_ERROR) {
                return;
            }
        }

        ColorMapObject* color_map = GifMakeMapObject(static_cast<int>(palette.size()), palette.data());
        if (!color_map) {
            return;
        }

        if (EGifPutImageDesc(gif_file_, 0, 0, width, height, false, color_map) == GIF_ERROR) {
            GifFreeMapObject(color_map);
            return;
        }
        GifFreeMapObject(color_map);

        for (int y = 0; y < height; ++y) {
            if (EGifPutLine(gif_file_, indices.data() + y * width, width) == GIF_ERROR) {
                break;
            }
        }
    }

    void VideoRecorder::QuantizeToPalette(const uint8_t* rgba, int width, int height, std::vector<uint8_t>* indices,
                                          std::vector<GifColorType>* palette) {
        indices->resize(static_cast<size_t>(width) * height);
        palette->clear();

        // Simple median-cut style quantization: collect unique colors, then downsample.
        struct Color {
            uint8_t r, g, b;
            bool operator==(const Color& o) const { return r == o.r && g == o.g && b == o.b; }
        };
        struct ColorHash {
            size_t operator()(const Color& c) const {
                return (static_cast<size_t>(c.r) << 16) | (static_cast<size_t>(c.g) << 8) | static_cast<size_t>(c.b);
            }
        };

        std::unordered_map<Color, size_t, ColorHash> color_counts;
        for (int i = 0; i < width * height; ++i) {
            Color c{rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2]};
            color_counts[c]++;
        }

        if (color_counts.size() <= 256) {
            palette->reserve(color_counts.size());
            for (const auto& kv : color_counts) {
                GifColorType gc;
                gc.Red   = kv.first.r;
                gc.Green = kv.first.g;
                gc.Blue  = kv.first.b;
                palette->push_back(gc);
            }
            std::unordered_map<Color, uint8_t, ColorHash> color_to_idx;
            for (size_t i = 0; i < palette->size(); ++i) {
                color_to_idx[Color{palette->at(i).Red, palette->at(i).Green, palette->at(i).Blue}] = static_cast<uint8_t>(i);
            }
            for (int i = 0; i < width * height; ++i) {
                Color c{rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2]};
                (*indices)[i] = color_to_idx[c];
            }
            return;
        }

        // More than 256 colors: use a simple octree-like bucketing by top bits.
        struct Bucket {
            uint32_t r_sum = 0, g_sum = 0, b_sum = 0, count = 0;
        };
        std::vector<Bucket> buckets(256);
        for (const auto& kv : color_counts) {
            int idx = ((kv.first.r >> 5) << 5) | ((kv.first.g >> 5) << 2) | (kv.first.b >> 6);
            buckets[idx].r_sum += static_cast<uint32_t>(kv.first.r) * static_cast<uint32_t>(kv.second);
            buckets[idx].g_sum += static_cast<uint32_t>(kv.first.g) * static_cast<uint32_t>(kv.second);
            buckets[idx].b_sum += static_cast<uint32_t>(kv.first.b) * static_cast<uint32_t>(kv.second);
            buckets[idx].count += static_cast<uint32_t>(kv.second);
        }

        // Sort by count descending and pick top 256
        std::vector<size_t> idxs(256);
        for (size_t i = 0; i < 256; ++i) {
            idxs[i] = i;
        }
        std::sort(idxs.begin(), idxs.end(), [&](size_t a, size_t b) { return buckets[a].count > buckets[b].count; });

        palette->reserve(256);
        std::unordered_map<int, uint8_t> bucket_to_idx;
        for (size_t i = 0; i < 256; ++i) {
            const Bucket& b = buckets[idxs[i]];
            GifColorType gc;
            if (b.count > 0) {
                gc.Red   = static_cast<uint8_t>(b.r_sum / b.count);
                gc.Green = static_cast<uint8_t>(b.g_sum / b.count);
                gc.Blue  = static_cast<uint8_t>(b.b_sum / b.count);
            } else {
                gc.Red = gc.Green = gc.Blue = 0;
            }
            palette->push_back(gc);
            bucket_to_idx[static_cast<int>(idxs[i])] = static_cast<uint8_t>(i);
        }

        for (int i = 0; i < width * height; ++i) {
            int idx       = ((rgba[i * 4] >> 5) << 5) | ((rgba[i * 4 + 1] >> 5) << 2) | (rgba[i * 4 + 2] >> 6);
            auto it       = bucket_to_idx.find(idx);
            (*indices)[i] = (it != bucket_to_idx.end()) ? it->second : 0;
        }
    }

}  // namespace kinematic_viewer
