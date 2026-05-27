#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <gif_lib.h>

namespace kinematic_viewer {

    class VideoRecorder {
       public:
        enum class Format {
            MP4,
            GIF,
        };

        VideoRecorder();
        ~VideoRecorder();

        // Non-copyable, non-movable
        VideoRecorder(const VideoRecorder&)            = delete;
        VideoRecorder& operator=(const VideoRecorder&) = delete;

        // Start recording. Returns true on success.
        bool StartRecording(const std::string& output_path, int width, int height, int fps, Format format);

        // Stop recording and finalize the file.
        void StopRecording();

        bool IsRecording() const;

        // Submit a raw RGBA frame from the main thread.
        // The data is copied into an internal queue; this call is non-blocking.
        void SubmitFrame(const uint8_t* rgba_data, int width, int height);

        // Human-readable status (e.g. "Recording MP4: 120 frames").
        std::string GetStatus() const;

        // Default output directory (created on demand).
        static std::string DefaultOutputDirectory();

        // Timestamp string for filenames.
        static std::string TimestampString();

       private:
        struct FrameBuffer {
            int width  = 0;
            int height = 0;
            std::vector<uint8_t> data;
        };

        void EncoderThreadLoop();

        // MP4 encoding helpers
        bool InitMp4(const std::string& path, int width, int height, int fps);
        void FinalizeMp4();
        void EncodeMp4Frame(const uint8_t* rgba_data, int width, int height);

        // GIF encoding helpers
        bool InitGif(const std::string& path, int width, int height, int fps);
        void FinalizeGif();
        void EncodeGifFrame(const uint8_t* rgba_data, int width, int height);

        // Quantize RGBA -> 256-color palette for GIF
        static void QuantizeToPalette(const uint8_t* rgba, int width, int height, std::vector<uint8_t>* indices,
                                      std::vector<GifColorType>* palette);

        mutable std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        std::queue<FrameBuffer> frame_queue_;
        bool encoder_stop_requested_ = false;

        std::thread encoder_thread_;

        std::atomic<bool> is_recording_{false};
        std::atomic<int> frame_count_{0};
        std::atomic<int> dropped_frames_{0};

        Format format_  = Format::MP4;
        int target_fps_ = 30;

        // FFmpeg state (MP4)
        AVFormatContext* fmt_ctx_  = nullptr;
        AVCodecContext* codec_ctx_ = nullptr;
        AVStream* video_stream_    = nullptr;
        AVFrame* yuv_frame_        = nullptr;
        AVPacket* packet_          = nullptr;
        SwsContext* sws_ctx_       = nullptr;
        int64_t next_pts_          = 0;

        // libgif state
        GifFileType* gif_file_    = nullptr;
        int gif_color_resolution_ = 8;
        std::vector<GifColorType> gif_global_palette_;
        int gif_frame_delay_hundredths_ = 3;  // default ~30fps
    };

}  // namespace kinematic_viewer
