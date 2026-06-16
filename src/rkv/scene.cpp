#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "rkv/scene.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <urdf_parser/urdf_parser.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>

namespace rkv {
    namespace {

        struct Vertex {
            glm::vec3 position;
            glm::vec3 normal;
            glm::vec2 tex_coords;
        };

        struct Texture {
            unsigned int id = 0;
            std::string type;
        };

        struct Mesh {
            std::vector<Vertex> vertices;
            std::vector<unsigned int> indices;
            std::vector<Texture> textures;
            glm::vec3 diffuse_color = glm::vec3(0.8f, 0.8f, 0.8f);
            unsigned int vao        = 0;
            unsigned int vbo        = 0;
            unsigned int ebo        = 0;

            void setup() {
                if (vao) {
                    glDeleteVertexArrays(1, &vao);
                    glDeleteBuffers(1, &vbo);
                    glDeleteBuffers(1, &ebo);
                    vao = vbo = ebo = 0;
                }

                glGenVertexArrays(1, &vao);
                glGenBuffers(1, &vbo);
                glGenBuffers(1, &ebo);

                glBindVertexArray(vao);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tex_coords));

                glBindVertexArray(0);
            }

            bool hasValidTexture() const { return !textures.empty() && textures[0].id != 0; }

            void draw(GLuint shader, const glm::vec3* override_color = nullptr, bool force_color_only = false) {
                if (!vao) {
                    setup();
                }
                glBindVertexArray(vao);
                const glm::vec3 color = override_color ? *override_color : diffuse_color;
                glUniform3f(glGetUniformLocation(shader, "diffuseColor"), color.r, color.g, color.b);
                const bool use_texture = hasValidTexture() && !force_color_only;
                if (use_texture) {
                    glUniform1i(glGetUniformLocation(shader, "hasTexture"), true);
                    glUniform1i(glGetUniformLocation(shader, "texture_diffuse1"), 0);
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, textures[0].id);
                } else {
                    glUniform1i(glGetUniformLocation(shader, "hasTexture"), false);
                }
                glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
                glBindVertexArray(0);
            }
        };

        class Model {
           public:
            std::vector<Mesh> meshes;
            std::string directory;

            void loadAssimp(const std::string& path, bool geometry_only = false) {
                Assimp::Importer importer;
                const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals);

                if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
                    std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
                    return;
                }

                size_t last_slash = path.find_last_of('/');
                if (last_slash != std::string::npos) {
                    directory = path.substr(0, last_slash);
                }

                processNode(scene->mRootNode, scene, geometry_only);
            }

           private:
            void processNode(aiNode* node, const aiScene* scene, bool geometry_only) {
                for (unsigned int i = 0; i < node->mNumMeshes; i++) {
                    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
                    meshes.push_back(processMesh(mesh, scene, geometry_only));
                }

                for (unsigned int i = 0; i < node->mNumChildren; i++) {
                    processNode(node->mChildren[i], scene, geometry_only);
                }
            }

            Mesh processMesh(aiMesh* mesh, const aiScene* scene, bool geometry_only) {
                Mesh m;

                for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
                    Vertex vertex;
                    vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);

                    if (mesh->HasNormals()) {
                        vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
                    } else {
                        vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
                    }

                    if (mesh->mTextureCoords[0]) {
                        vertex.tex_coords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
                    } else {
                        vertex.tex_coords = glm::vec2(0.0f, 0.0f);
                    }

                    m.vertices.push_back(vertex);
                }

                for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
                    aiFace face = mesh->mFaces[i];
                    for (unsigned int j = 0; j < face.mNumIndices; j++) {
                        m.indices.push_back(face.mIndices[j]);
                    }
                }

                if (!geometry_only && mesh->mMaterialIndex >= 0) {
                    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
                    aiColor3D color(1.0f, 1.0f, 1.0f);
                    material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
                    m.diffuse_color = glm::vec3(color.r, color.g, color.b);

                    if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
                        aiString str;
                        material->GetTexture(aiTextureType_DIFFUSE, 0, &str);
                        std::string tex_path = str.C_Str();
                        if (!tex_path.empty() && tex_path[0] != '/') {
                            tex_path = directory.empty() ? tex_path : (directory + "/" + tex_path);
                        }
                        const unsigned int tex_id = loadTexture(tex_path);
                        if (tex_id != 0) {
                            Texture tex;
                            tex.id   = tex_id;
                            tex.type = "texture_diffuse";
                            m.textures.push_back(tex);
                        }
                    }
                }

                if (!geometry_only) {
                    m.setup();
                }
                return m;
            }

            unsigned int loadTexture(const std::string& path) {
                if (path.empty()) {
                    return 0;
                }

                int width           = 0;
                int height          = 0;
                int nr_components   = 0;
                unsigned char* data = stbi_load(path.c_str(), &width, &height, &nr_components, 0);
                if (!data || width <= 0 || height <= 0) {
                    std::cerr << "Texture failed to load at path: " << path << std::endl;
                    stbi_image_free(data);
                    return 0;
                }

                unsigned int texture_id = 0;
                glGenTextures(1, &texture_id);
                GLenum format = nr_components == 1 ? GL_RED : nr_components == 3 ? GL_RGB : GL_RGBA;
                glBindTexture(GL_TEXTURE_2D, texture_id);
                glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
                glGenerateMipmap(GL_TEXTURE_2D);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                stbi_image_free(data);
                return texture_id;
            }
        };

        struct LinkVisual {
            std::string mesh_file;
            glm::vec3 scale = glm::vec3(1.0f);

            std::string parent_link_name;
            glm::mat4 local_transform = glm::mat4(1.0f);
            glm::vec3 urdf_color      = glm::vec3(0.8f, 0.8f, 0.8f);
            bool has_urdf_color       = false;

            Model model;
            bool loaded                     = false;
            glm::vec3 local_bounding_center = glm::vec3(0.0f);
            float local_bounding_radius     = 0.0f;
        };

        struct LocalCollisionProxy {
            std::string link_name;
            std::string collision_name;
            glm::mat4 local_transform = glm::mat4(1.0f);
            glm::vec3 local_center    = glm::vec3(0.0f);
            float radius_m            = 0.0f;
            bool has_model_aabb       = false;
            glm::vec3 model_aabb_min  = glm::vec3(0.0f);
            glm::vec3 model_aabb_max  = glm::vec3(0.0f);
            std::vector<glm::vec3> model_triangles;
        };

        enum class CollisionGeomKind { Sphere, Box, Cylinder, Mesh };

        struct CollisionVisual {
            std::string link_name;
            glm::mat4 local_transform = glm::mat4(1.0f);
            CollisionGeomKind kind    = CollisionGeomKind::Sphere;
            glm::vec3 params          = glm::vec3(0.1f);
            Model model;
            bool loaded = false;
        };

        void appendMeshTriangle(Mesh* mesh, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
            if (mesh == nullptr) {
                return;
            }
            const unsigned int base = static_cast<unsigned int>(mesh->vertices.size());
            auto push_vertex        = [&](const glm::vec3& p, const glm::vec3& n) {
                Vertex v;
                v.position   = p;
                v.normal     = n;
                v.tex_coords = glm::vec2(0.0f);
                mesh->vertices.push_back(v);
            };
            const glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));
            push_vertex(a, n);
            push_vertex(b, n);
            push_vertex(c, n);
            mesh->indices.push_back(base);
            mesh->indices.push_back(base + 1);
            mesh->indices.push_back(base + 2);
        }

        void buildUnitSphereMesh(Model* model, int stacks = 14, int sectors = 22) {
            if (model == nullptr) {
                return;
            }
            Mesh mesh;
            for (int iy = 0; iy <= stacks; ++iy) {
                const float v     = static_cast<float>(iy) / static_cast<float>(stacks);
                const float phi   = glm::pi<float>() * v;
                const float sin_p = std::sin(phi);
                const float cos_p = std::cos(phi);
                for (int ix = 0; ix <= sectors; ++ix) {
                    const float u     = static_cast<float>(ix) / static_cast<float>(sectors);
                    const float theta = glm::two_pi<float>() * u;
                    const glm::vec3 p(0.5f * sin_p * std::cos(theta), 0.5f * sin_p * std::sin(theta), 0.5f * cos_p);
                    Vertex vtx;
                    vtx.position   = p;
                    vtx.normal     = glm::normalize(p);
                    vtx.tex_coords = glm::vec2(u, v);
                    mesh.vertices.push_back(vtx);
                }
            }
            for (int iy = 0; iy < stacks; ++iy) {
                for (int ix = 0; ix < sectors; ++ix) {
                    const int cur  = iy * (sectors + 1) + ix;
                    const int next = cur + sectors + 1;
                    mesh.indices.push_back(static_cast<unsigned int>(cur));
                    mesh.indices.push_back(static_cast<unsigned int>(next));
                    mesh.indices.push_back(static_cast<unsigned int>(cur + 1));
                    mesh.indices.push_back(static_cast<unsigned int>(cur + 1));
                    mesh.indices.push_back(static_cast<unsigned int>(next));
                    mesh.indices.push_back(static_cast<unsigned int>(next + 1));
                }
            }
            mesh.setup();
            model->meshes.push_back(std::move(mesh));
        }

        void buildUnitBoxMesh(Model* model) {
            if (model == nullptr) {
                return;
            }
            Mesh mesh;
            const glm::vec3 corners[8] = {
                {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
                {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},  {-0.5f, 0.5f, 0.5f},
            };
            const int faces[12][3] = {
                {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                {2, 6, 7}, {2, 7, 3}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
            };
            for (const auto& face : faces) {
                appendMeshTriangle(&mesh, corners[face[0]], corners[face[1]], corners[face[2]]);
            }
            mesh.setup();
            model->meshes.push_back(std::move(mesh));
        }

        void buildUnitCylinderMesh(Model* model, int sectors = 24) {
            if (model == nullptr) {
                return;
            }
            Mesh mesh;
            const float half_z = 0.5f;
            for (int i = 0; i < sectors; ++i) {
                const float a0 = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(sectors);
                const float a1 = glm::two_pi<float>() * static_cast<float>(i + 1) / static_cast<float>(sectors);
                const glm::vec3 p0(0.5f * std::cos(a0), 0.5f * std::sin(a0), -half_z);
                const glm::vec3 p1(0.5f * std::cos(a1), 0.5f * std::sin(a1), -half_z);
                const glm::vec3 p2(0.5f * std::cos(a1), 0.5f * std::sin(a1), half_z);
                const glm::vec3 p3(0.5f * std::cos(a0), 0.5f * std::sin(a0), half_z);
                appendMeshTriangle(&mesh, p0, p1, p2);
                appendMeshTriangle(&mesh, p0, p2, p3);
            }
            const glm::vec3 top(0.0f, 0.0f, half_z);
            const glm::vec3 bottom(0.0f, 0.0f, -half_z);
            for (int i = 0; i < sectors; ++i) {
                const float a0 = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(sectors);
                const float a1 = glm::two_pi<float>() * static_cast<float>(i + 1) / static_cast<float>(sectors);
                appendMeshTriangle(&mesh, top, glm::vec3(0.5f * std::cos(a0), 0.5f * std::sin(a0), half_z),
                                   glm::vec3(0.5f * std::cos(a1), 0.5f * std::sin(a1), half_z));
                appendMeshTriangle(&mesh, bottom, glm::vec3(0.5f * std::cos(a1), 0.5f * std::sin(a1), -half_z),
                                   glm::vec3(0.5f * std::cos(a0), 0.5f * std::sin(a0), -half_z));
            }
            mesh.setup();
            model->meshes.push_back(std::move(mesh));
        }

        bool linkVisualNeedsHighlight(const std::string& link_name, const RobotScene::SceneDrawStyle& style) {
            return (!style.selected_link.empty() && link_name == style.selected_link) ||
                   (!style.hovered_link.empty() && link_name == style.hovered_link);
        }

        glm::vec3 linkVisualHighlightColor(const std::string& link_name, const RobotScene::SceneDrawStyle& style) {
            if (!style.selected_link.empty() && link_name == style.selected_link) {
                return glm::vec3(0.30f, 0.88f, 1.0f);
            }
            return glm::vec3(0.35f, 0.82f, 1.0f);
        }

        struct JointState {
            std::string name;
            float position  = 0.0f;
            float min_angle = -3.14f;
            float max_angle = 3.14f;
            bool revolute   = false;
        };

        struct JointAxisState {
            std::string name;
            glm::vec3 axis_local   = glm::vec3(0.0f, 0.0f, 1.0f);
            glm::vec3 world_origin = glm::vec3(0.0f);
            glm::vec3 world_axis   = glm::vec3(0.0f, 0.0f, 1.0f);
            bool revolute          = false;
        };

        glm::mat4 poseToTransform(const urdf::Pose& pose) {
            glm::vec3 origin_xyz(pose.position.x, pose.position.y, pose.position.z);
            double roll  = 0.0;
            double pitch = 0.0;
            double yaw   = 0.0;
            pose.rotation.getRPY(roll, pitch, yaw);
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), origin_xyz);
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(roll), glm::vec3(1, 0, 0));
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(pitch), glm::vec3(0, 1, 0));
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(yaw), glm::vec3(0, 0, 1));
            return transform;
        }

    }  // namespace

    struct LinkInertialLocal {
        std::string link_name;
        glm::mat4 local_transform = glm::mat4(1.0f);
    };

    struct PickMeshTriangle {
        std::string link_name;
        glm::vec3 v0 = glm::vec3(0.0f);
        glm::vec3 v1 = glm::vec3(0.0f);
        glm::vec3 v2 = glm::vec3(0.0f);
    };

    bool intersectRayTriangle(const glm::vec3& ray_o, const glm::vec3& ray_d, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                              float* out_t) {
        const float eps      = 1e-6f;
        const glm::vec3 e1   = v1 - v0;
        const glm::vec3 e2   = v2 - v0;
        const glm::vec3 pvec = glm::cross(ray_d, e2);
        const float det      = glm::dot(e1, pvec);
        if (std::fabs(det) < eps) {
            return false;
        }
        const float inv_det  = 1.0f / det;
        const glm::vec3 tvec = ray_o - v0;
        const float u        = glm::dot(tvec, pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) {
            return false;
        }
        const glm::vec3 qvec = glm::cross(tvec, e1);
        const float v        = glm::dot(ray_d, qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) {
            return false;
        }
        const float t = glm::dot(e2, qvec) * inv_det;
        if (t <= eps) {
            return false;
        }
        if (out_t != nullptr) {
            *out_t = t;
        }
        return true;
    }

    struct RobotScene::Impl {
        std::map<std::string, LinkVisual> visuals;
        std::vector<LocalCollisionProxy> collision_proxies_local;
        std::vector<CollisionVisual> collision_visuals;
        std::map<std::string, glm::mat4> transforms;
        std::map<std::string, std::string> link_parent;
        std::unordered_map<std::string, LinkInertialLocal> link_inertials;
        std::vector<PickMeshTriangle> pick_mesh_triangles;
        bool pick_mesh_cache_dirty         = true;
        bool collision_proxies_cache_valid = false;
        mutable std::vector<LinkCollisionProxy> cached_collision_proxies;
        bool joint_pose_dirty = false;
        std::unordered_map<std::string, std::string> link_to_parent_joint;
        std::unordered_map<std::string, JointDetailInfo> joint_details;
        std::vector<JointState> joint_states;
        std::vector<JointAxisState> joint_axis_states;
        std::unordered_map<std::string, size_t> joint_axis_index;

        std::string package_path;
        std::string urdf_file_path;
        urdf::ModelInterfaceSharedPtr urdf_model;

        bool fixed_base_mode       = true;
        float virtual_base_x_m     = 0.0f;
        float virtual_base_y_m     = 0.0f;
        float virtual_base_yaw_rad = 0.0f;

        glm::mat4 rootWorldTransform() const {
            if (fixed_base_mode) {
                return glm::mat4(1.0f);
            }
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(virtual_base_x_m, virtual_base_y_m, 0.0f));
            transform           = transform * glm::rotate(glm::mat4(1.0f), virtual_base_yaw_rad, glm::vec3(0.0f, 0.0f, 1.0f));
            return transform;
        }

        bool computeBoundsFromModel(const Model& model, glm::vec3* out_min, glm::vec3* out_max, glm::vec3* out_center = nullptr,
                                    float* out_radius = nullptr) const {
            if (out_min == nullptr || out_max == nullptr) {
                return false;
            }
            glm::vec3 min_v(1e9f);
            glm::vec3 max_v(-1e9f);
            bool has_vertex = false;
            for (const auto& mesh : model.meshes) {
                for (const auto& vertex : mesh.vertices) {
                    min_v      = glm::min(min_v, vertex.position);
                    max_v      = glm::max(max_v, vertex.position);
                    has_vertex = true;
                }
            }
            if (!has_vertex) {
                *out_min = glm::vec3(0.0f);
                *out_max = glm::vec3(0.0f);
                if (out_center != nullptr) {
                    *out_center = glm::vec3(0.0f);
                }
                if (out_radius != nullptr) {
                    *out_radius = 0.0f;
                }
                return false;
            }

            *out_min = min_v;
            *out_max = max_v;
            if (out_center != nullptr || out_radius != nullptr) {
                const glm::vec3 center = 0.5f * (min_v + max_v);
                if (out_center != nullptr) {
                    *out_center = center;
                }
                if (out_radius != nullptr) {
                    float radius = 0.0f;
                    for (const auto& mesh : model.meshes) {
                        for (const auto& vertex : mesh.vertices) {
                            radius = std::max(radius, glm::length(vertex.position - center));
                        }
                    }
                    *out_radius = radius;
                }
            }
            return true;
        }

        bool computeBoundingSphereFromModel(const Model& model, glm::vec3* out_center, float* out_radius) const {
            if (out_center == nullptr || out_radius == nullptr) {
                return false;
            }
            glm::vec3 min_v;
            glm::vec3 max_v;
            if (!computeBoundsFromModel(model, &min_v, &max_v, out_center, out_radius)) {
                *out_center = glm::vec3(0.0f);
                *out_radius = 0.0f;
                return false;
            }
            return true;
        }

        void ExpandWorldAabbFromModelAabb(const glm::mat4& world_from_model, const glm::vec3& model_min, const glm::vec3& model_max,
                                          glm::vec3* world_min, glm::vec3* world_max) const {
            if (world_min == nullptr || world_max == nullptr) {
                return;
            }
            *world_min = glm::vec3(1e9f);
            *world_max = glm::vec3(-1e9f);
            for (int ix = 0; ix <= 1; ++ix) {
                for (int iy = 0; iy <= 1; ++iy) {
                    for (int iz = 0; iz <= 1; ++iz) {
                        const glm::vec3 model_corner(ix ? model_max.x : model_min.x, iy ? model_max.y : model_min.y,
                                                     iz ? model_max.z : model_min.z);
                        const glm::vec3 world_corner = glm::vec3(world_from_model * glm::vec4(model_corner, 1.0f));
                        *world_min                   = glm::min(*world_min, world_corner);
                        *world_max                   = glm::max(*world_max, world_corner);
                    }
                }
            }
        }

        void AppendModelTriangles(const Model& model, std::vector<glm::vec3>* out_triangles) const {
            if (out_triangles == nullptr) {
                return;
            }
            for (const auto& mesh : model.meshes) {
                for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                    out_triangles->push_back(mesh.vertices[mesh.indices[i]].position);
                    out_triangles->push_back(mesh.vertices[mesh.indices[i + 1]].position);
                    out_triangles->push_back(mesh.vertices[mesh.indices[i + 2]].position);
                }
            }
        }

        void FillLinkCollisionProxyAabb(const glm::mat4& world_from_model, const LocalCollisionProxy& local_proxy,
                                        LinkCollisionProxy* proxy) const {
            if (proxy == nullptr || !local_proxy.has_model_aabb) {
                return;
            }
            ExpandWorldAabbFromModelAabb(world_from_model, local_proxy.model_aabb_min, local_proxy.model_aabb_max, &proxy->world_aabb_min,
                                         &proxy->world_aabb_max);
            proxy->has_world_aabb = true;
        }

        void AppendWorldCollisionTrianglesForLink(const std::string& link_name, std::vector<glm::vec3>* out_triangles) const {
            if (out_triangles == nullptr) {
                return;
            }
            for (const auto& local_proxy : collision_proxies_local) {
                if (local_proxy.link_name != link_name || local_proxy.model_triangles.empty()) {
                    continue;
                }
                const auto link_it = transforms.find(local_proxy.link_name);
                if (link_it == transforms.end()) {
                    continue;
                }
                const glm::mat4 world_from_model = link_it->second * local_proxy.local_transform;
                for (const glm::vec3& vertex : local_proxy.model_triangles) {
                    out_triangles->push_back(glm::vec3(world_from_model * glm::vec4(vertex, 1.0f)));
                }
            }
        }

        std::vector<LinkCollisionProxy> buildLinkCollisionProxies() const {
            std::vector<LinkCollisionProxy> proxies;
            if (!collision_proxies_local.empty()) {
                proxies.reserve(collision_proxies_local.size());
                for (const auto& local_proxy : collision_proxies_local) {
                    if (local_proxy.radius_m <= 1e-6f) {
                        continue;
                    }
                    const auto link_it = transforms.find(local_proxy.link_name);
                    if (link_it == transforms.end()) {
                        continue;
                    }

                    const glm::mat4 world_from_model = link_it->second * local_proxy.local_transform;
                    LinkCollisionProxy proxy;
                    proxy.link_name    = local_proxy.link_name;
                    proxy.visual_name  = local_proxy.collision_name;
                    proxy.world_center = glm::vec3(world_from_model * glm::vec4(local_proxy.local_center, 1.0f));
                    proxy.radius_m     = local_proxy.radius_m;
                    FillLinkCollisionProxyAabb(world_from_model, local_proxy, &proxy);
                    proxies.push_back(std::move(proxy));
                }
                return proxies;
            }

            proxies.reserve(visuals.size());
            for (const auto& [visual_name, visual] : visuals) {
                if (!visual.loaded || visual.local_bounding_radius <= 1e-6f) {
                    continue;
                }

                const auto link_it = transforms.find(visual.parent_link_name);
                if (link_it == transforms.end()) {
                    continue;
                }

                glm::mat4 proxy_transform    = link_it->second * visual.local_transform;
                proxy_transform              = proxy_transform * glm::scale(glm::mat4(1.0f), visual.scale);
                const glm::vec3 world_center = glm::vec3(proxy_transform * glm::vec4(visual.local_bounding_center, 1.0f));
                const float scale_factor =
                    std::max(std::max(std::fabs(visual.scale.x), std::fabs(visual.scale.y)), std::fabs(visual.scale.z));

                LinkCollisionProxy proxy;
                proxy.link_name              = visual.parent_link_name;
                proxy.visual_name            = visual_name;
                proxy.world_center           = world_center;
                proxy.radius_m               = std::max(0.0f, visual.local_bounding_radius * scale_factor);
                const glm::vec3 local_extent = glm::vec3(proxy.radius_m);
                LocalCollisionProxy visual_aabb;
                visual_aabb.has_model_aabb = proxy.radius_m > 1e-6f;
                visual_aabb.model_aabb_min = visual.local_bounding_center - local_extent;
                visual_aabb.model_aabb_max = visual.local_bounding_center + local_extent;
                FillLinkCollisionProxyAabb(proxy_transform, visual_aabb, &proxy);
                proxies.push_back(std::move(proxy));
            }
            return proxies;
        }

        void computeBoundingSphere(LinkVisual* visual) const {
            if (visual == nullptr) {
                return;
            }
            if (!computeBoundingSphereFromModel(visual->model, &visual->local_bounding_center, &visual->local_bounding_radius)) {
                visual->local_bounding_center = glm::vec3(0.0f);
                visual->local_bounding_radius = 0.0f;
            }
        }

        void rebuildPickMeshCache() {
            pick_mesh_triangles.clear();
            for (const auto& [visual_key, lv] : visuals) {
                (void)visual_key;
                if (!lv.loaded) {
                    continue;
                }
                const auto it = transforms.find(lv.parent_link_name);
                if (it == transforms.end()) {
                    continue;
                }
                const glm::mat4 model_mat = it->second * lv.local_transform * glm::scale(glm::mat4(1.0f), lv.scale);
                for (const auto& mesh : lv.model.meshes) {
                    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                        const auto world_vertex = [&](unsigned int idx) -> glm::vec3 {
                            const glm::vec3 local = mesh.vertices[mesh.indices[idx]].position;
                            const glm::vec4 world = model_mat * glm::vec4(local, 1.0f);
                            return glm::vec3(world);
                        };
                        PickMeshTriangle tri;
                        tri.link_name = lv.parent_link_name;
                        tri.v0        = world_vertex(static_cast<unsigned int>(i));
                        tri.v1        = world_vertex(static_cast<unsigned int>(i + 1));
                        tri.v2        = world_vertex(static_cast<unsigned int>(i + 2));
                        pick_mesh_triangles.push_back(tri);
                    }
                }
            }
        }

        bool buildCollisionProxyFromGeometry(const urdf::CollisionSharedPtr& collision, const std::string& link_name,
                                             const std::string& collision_name, LocalCollisionProxy* out_proxy) const {
            if (collision == nullptr || collision->geometry == nullptr || out_proxy == nullptr) {
                return false;
            }

            out_proxy->link_name       = link_name;
            out_proxy->collision_name  = collision_name;
            out_proxy->local_transform = poseToTransform(collision->origin);
            out_proxy->local_center    = glm::vec3(0.0f);
            out_proxy->radius_m        = 0.0f;
            out_proxy->has_model_aabb  = false;

            if (collision->geometry->type == urdf::Geometry::SPHERE) {
                auto sphere               = std::static_pointer_cast<urdf::Sphere>(collision->geometry);
                out_proxy->radius_m       = static_cast<float>(sphere->radius);
                const float radius        = out_proxy->radius_m;
                out_proxy->model_aabb_min = glm::vec3(-radius);
                out_proxy->model_aabb_max = glm::vec3(radius);
                out_proxy->has_model_aabb = radius > 1e-6f;
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::BOX) {
                auto box = std::static_pointer_cast<urdf::Box>(collision->geometry);
                const glm::vec3 dims(static_cast<float>(box->dim.x), static_cast<float>(box->dim.y), static_cast<float>(box->dim.z));
                out_proxy->radius_m       = 0.5f * glm::length(dims);
                const glm::vec3 half_dims = 0.5f * dims;
                out_proxy->model_aabb_min = -half_dims;
                out_proxy->model_aabb_max = half_dims;
                out_proxy->has_model_aabb = out_proxy->radius_m > 1e-6f;
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cylinder             = std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
                const float radius        = static_cast<float>(cylinder->radius);
                const float half_length   = static_cast<float>(0.5 * cylinder->length);
                out_proxy->radius_m       = std::sqrt(radius * radius + half_length * half_length);
                out_proxy->model_aabb_min = glm::vec3(-radius, -radius, -half_length);
                out_proxy->model_aabb_max = glm::vec3(radius, radius, half_length);
                out_proxy->has_model_aabb = out_proxy->radius_m > 1e-6f;
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::MESH) {
                auto mesh                   = std::static_pointer_cast<urdf::Mesh>(collision->geometry);
                const std::string mesh_file = resolvePath(mesh->filename);
                if (mesh_file.empty()) {
                    return false;
                }
                Model collision_model;
                collision_model.loadAssimp(mesh_file, true);
                glm::vec3 mesh_center(0.0f);
                float mesh_radius = 0.0f;
                if (!computeBoundsFromModel(collision_model, &out_proxy->model_aabb_min, &out_proxy->model_aabb_max, &mesh_center,
                                            &mesh_radius)) {
                    return false;
                }
                AppendModelTriangles(collision_model, &out_proxy->model_triangles);
                const glm::vec3 mesh_scale(static_cast<float>(mesh->scale.x), static_cast<float>(mesh->scale.y),
                                           static_cast<float>(mesh->scale.z));
                out_proxy->local_transform = out_proxy->local_transform * glm::scale(glm::mat4(1.0f), mesh_scale);
                out_proxy->local_center    = mesh_center;
                const float scale_factor   = std::max(std::max(std::fabs(mesh_scale.x), std::fabs(mesh_scale.y)), std::fabs(mesh_scale.z));
                out_proxy->radius_m        = std::max(0.0f, mesh_radius * scale_factor);
                out_proxy->has_model_aabb  = out_proxy->radius_m > 1e-6f;
                return out_proxy->radius_m > 1e-6f;
            }

            return false;
        }

        bool buildCollisionVisualFromGeometry(const urdf::CollisionSharedPtr& collision, const std::string& link_name,
                                              const std::string& collision_name, LocalCollisionProxy* out_proxy,
                                              CollisionVisual* out_visual) const {
            if (!buildCollisionProxyFromGeometry(collision, link_name, collision_name, out_proxy)) {
                return false;
            }
            if (out_visual == nullptr || collision->geometry == nullptr) {
                return true;
            }

            out_visual->link_name       = link_name;
            out_visual->local_transform = out_proxy->local_transform;
            out_visual->loaded          = false;
            out_visual->model.meshes.clear();

            if (collision->geometry->type == urdf::Geometry::SPHERE) {
                auto sphere        = std::static_pointer_cast<urdf::Sphere>(collision->geometry);
                out_visual->kind   = CollisionGeomKind::Sphere;
                const float radius = static_cast<float>(sphere->radius);
                out_visual->params = glm::vec3(radius, radius, radius);
                buildUnitSphereMesh(&out_visual->model);
                out_visual->loaded = !out_visual->model.meshes.empty();
                return true;
            }

            if (collision->geometry->type == urdf::Geometry::BOX) {
                auto box         = std::static_pointer_cast<urdf::Box>(collision->geometry);
                out_visual->kind = CollisionGeomKind::Box;
                out_visual->params =
                    glm::vec3(static_cast<float>(box->dim.x), static_cast<float>(box->dim.y), static_cast<float>(box->dim.z));
                buildUnitBoxMesh(&out_visual->model);
                out_visual->loaded = !out_visual->model.meshes.empty();
                return true;
            }

            if (collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cylinder      = std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
                out_visual->kind   = CollisionGeomKind::Cylinder;
                out_visual->params = glm::vec3(static_cast<float>(cylinder->radius), static_cast<float>(cylinder->radius),
                                               static_cast<float>(cylinder->length));
                buildUnitCylinderMesh(&out_visual->model);
                out_visual->loaded = !out_visual->model.meshes.empty();
                return true;
            }

            if (collision->geometry->type == urdf::Geometry::MESH) {
                auto mesh                   = std::static_pointer_cast<urdf::Mesh>(collision->geometry);
                const std::string mesh_file = resolvePath(mesh->filename);
                if (mesh_file.empty()) {
                    return true;
                }
                out_visual->kind = CollisionGeomKind::Mesh;
                out_visual->params =
                    glm::vec3(static_cast<float>(mesh->scale.x), static_cast<float>(mesh->scale.y), static_cast<float>(mesh->scale.z));
                out_visual->model.loadAssimp(mesh_file);
                out_visual->loaded = !out_visual->model.meshes.empty();
                return true;
            }

            return true;
        }

        std::string resolvePath(const std::string& path) const {
            if (path.empty()) {
                return path;
            }

            // Absolute path: use as-is.
            if (path.front() == '/') {
                return path;
            }

            // Relative path from URDF directory.
            if (!package_path.empty()) {
                std::string candidate = package_path + "/" + path;
                std::ifstream f(candidate);
                if (f.good()) {
                    return candidate;
                }
            }

            if (path.rfind("package://", 0) == 0) {
                size_t package_end = path.find('/', 10);
                if (package_end != std::string::npos) {
                    std::string package_name  = path.substr(10, package_end - 10);
                    std::string relative_path = path.substr(package_end);

                    if (!package_path.empty()) {
                        std::string candidate = package_path + relative_path;
                        std::ifstream f(candidate);
                        if (f.good()) {
                            return candidate;
                        }
                    }

                    std::string cmd = "rospack find " + package_name;
                    FILE* pipe      = popen(cmd.c_str(), "r");
                    if (!pipe) {
                        return path;
                    }

                    char buffer[512];
                    std::string result;
                    if (fgets(buffer, sizeof(buffer), pipe)) {
                        result = buffer;
                        if (!result.empty() && result.back() == '\n') {
                            result.pop_back();
                        }
                    }
                    pclose(pipe);

                    if (!result.empty()) {
                        std::string candidate = result + relative_path;
                        std::ifstream f2(candidate);
                        if (f2.good()) {
                            return candidate;
                        }
                    }
                }
            }
            return path;
        }

        static std::string JointTypeToString(int joint_type) {
            switch (joint_type) {
                case urdf::Joint::REVOLUTE:
                    return "revolute";
                case urdf::Joint::CONTINUOUS:
                    return "continuous";
                case urdf::Joint::PRISMATIC:
                    return "prismatic";
                case urdf::Joint::FIXED:
                    return "fixed";
                case urdf::Joint::FLOATING:
                    return "floating";
                case urdf::Joint::PLANAR:
                    return "planar";
                default:
                    return "unknown";
            }
        }

        void initJointStates(urdf::ModelInterfaceSharedPtr model) {
            joint_states.clear();
            joint_axis_states.clear();
            joint_axis_index.clear();
            joint_details.clear();
            link_to_parent_joint.clear();
            std::function<void(urdf::LinkConstSharedPtr)> collectJoints = [&](urdf::LinkConstSharedPtr link) {
                for (auto& child_link : link->child_links) {
                    auto joint = child_link->parent_joint;
                    if (joint) {
                        link_to_parent_joint[child_link->name] = joint->name;

                        JointState js;
                        js.name      = joint->name;
                        js.position  = 0.0f;
                        js.min_angle = joint->limits ? static_cast<float>(joint->limits->lower) : -3.14f;
                        js.max_angle = joint->limits ? static_cast<float>(joint->limits->upper) : 3.14f;
                        js.revolute  = (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS);
                        joint_states.push_back(js);

                        JointDetailInfo detail;
                        detail.name        = joint->name;
                        detail.parent_link = joint->parent_link_name;
                        detail.child_link  = child_link->name;
                        detail.type        = JointTypeToString(joint->type);
                        detail.revolute    = js.revolute;
                        detail.has_limits  = (joint->limits != nullptr);
                        detail.lower_limit = js.min_angle;
                        detail.upper_limit = js.max_angle;
                        if (joint->limits && joint->limits->velocity > 0.0) {
                            detail.velocity_limit = static_cast<float>(joint->limits->velocity);
                        }
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        if (glm::length(axis) < 1e-6f) {
                            axis = glm::vec3(0.0f, 0.0f, 1.0f);
                        }
                        detail.axis_local          = glm::normalize(axis);
                        joint_details[detail.name] = detail;

                        JointAxisState axis_state;
                        axis_state.name                   = joint->name;
                        axis_state.axis_local             = detail.axis_local;
                        axis_state.revolute               = js.revolute;
                        joint_axis_index[axis_state.name] = joint_axis_states.size();
                        joint_axis_states.push_back(axis_state);
                    }
                    collectJoints(child_link);
                }
            };
            collectJoints(model->getRoot());
        }
    };

    RobotScene::RobotScene() : impl_(std::make_unique<Impl>()) {}
    RobotScene::~RobotScene() = default;

    bool RobotScene::loadURDF(const std::string& urdf_path) {
        impl_->urdf_file_path = urdf_path;
        size_t pos            = urdf_path.find_last_of('/');
        if (pos != std::string::npos) {
            impl_->package_path = urdf_path.substr(0, pos);
        }

        std::ifstream file(urdf_path);
        if (!file.good()) {
            std::cerr << "Failed to open URDF file: " << urdf_path << std::endl;
            return false;
        }

        std::string xml_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        auto model = urdf::parseURDF(xml_str);
        if (!model) {
            std::cerr << "Failed to parse URDF file: " << urdf_path << std::endl;
            return false;
        }

        impl_->urdf_model = model;
        impl_->initJointStates(model);

        impl_->visuals.clear();
        impl_->collision_proxies_local.clear();
        impl_->collision_visuals.clear();
        impl_->transforms.clear();
        impl_->link_parent.clear();
        impl_->link_inertials.clear();
        impl_->pick_mesh_triangles.clear();
        impl_->pick_mesh_cache_dirty         = true;
        impl_->collision_proxies_cache_valid = false;

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&, const std::string&)> traverse =
            [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform, const std::string& parent_name) {
                impl_->transforms[link->name]  = parent_transform;
                impl_->link_parent[link->name] = parent_name;

                if (link->inertial) {
                    LinkInertialLocal inertial;
                    inertial.link_name                = link->name;
                    inertial.local_transform          = poseToTransform(link->inertial->origin);
                    impl_->link_inertials[link->name] = inertial;
                }

                for (size_t i = 0; i < link->visual_array.size(); ++i) {
                    auto visual = link->visual_array[i];
                    if (!visual || !visual->geometry) {
                        continue;
                    }

                    LinkVisual lv;
                    if (visual->geometry->type == urdf::Geometry::MESH) {
                        auto mesh    = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
                        lv.mesh_file = impl_->resolvePath(mesh->filename);
                        lv.scale     = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                    } else {
                        continue;
                    }

                    lv.local_transform  = poseToTransform(visual->origin);
                    lv.parent_link_name = link->name;
                    if (visual->material) {
                        lv.urdf_color =
                            glm::vec3(static_cast<float>(visual->material->color.r), static_cast<float>(visual->material->color.g),
                                      static_cast<float>(visual->material->color.b));
                        lv.has_urdf_color = true;
                    }

                    if (!lv.mesh_file.empty()) {
                        lv.model.loadAssimp(lv.mesh_file);
                        lv.loaded = true;
                        impl_->computeBoundingSphere(&lv);
                    }

                    std::string visual_name = link->name;
                    if (link->visual_array.size() > 1) {
                        visual_name += "_visual_" + std::to_string(i);
                    }
                    impl_->visuals[visual_name] = lv;
                }

                std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
                if (collisions.empty() && link->collision != nullptr) {
                    collisions.push_back(link->collision);
                }
                for (size_t i = 0; i < collisions.size(); ++i) {
                    const auto& collision = collisions[i];
                    if (collision == nullptr || collision->geometry == nullptr) {
                        continue;
                    }
                    std::string collision_name = collision->name;
                    if (collision_name.empty()) {
                        collision_name = link->name + "_collision_" + std::to_string(i);
                    }
                    LocalCollisionProxy local_proxy;
                    CollisionVisual collision_visual;
                    if (!impl_->buildCollisionVisualFromGeometry(collision, link->name, collision_name, &local_proxy, &collision_visual)) {
                        continue;
                    }
                    impl_->collision_proxies_local.push_back(std::move(local_proxy));
                    if (collision_visual.loaded) {
                        impl_->collision_visuals.push_back(std::move(collision_visual));
                    }
                }

                for (auto& child_link : link->child_links) {
                    auto joint                = child_link->parent_joint;
                    glm::mat4 joint_transform = parent_transform;

                    if (joint) {
                        auto joint_origin   = joint->parent_to_joint_origin_transform;
                        glm::vec3 joint_xyz = glm::vec3(joint_origin.position.x, joint_origin.position.y, joint_origin.position.z);

                        double rr = 0.0;
                        double pp = 0.0;
                        double yy = 0.0;
                        joint_origin.rotation.getRPY(rr, pp, yy);

                        glm::mat4 joint_offset = glm::translate(glm::mat4(1.0f), joint_xyz) *
                                                 glm::rotate(glm::mat4(1.0f), static_cast<float>(rr), glm::vec3(1, 0, 0)) *
                                                 glm::rotate(glm::mat4(1.0f), static_cast<float>(pp), glm::vec3(0, 1, 0)) *
                                                 glm::rotate(glm::mat4(1.0f), static_cast<float>(yy), glm::vec3(0, 0, 1));

                        joint_transform = joint_transform * joint_offset;

                        for (const auto& js : impl_->joint_states) {
                            if (js.name != joint->name) {
                                continue;
                            }

                            if (impl_->fixed_base_mode && IsBaseMotionJointName(joint->name)) {
                                break;
                            }

                            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                axis            = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(0, 0, 1);
                                joint_transform = joint_transform * glm::rotate(glm::mat4(1.0f), js.position, axis);
                            } else if (joint->type == urdf::Joint::PRISMATIC) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                axis                  = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(1, 0, 0);
                                glm::vec3 translation = axis * js.position;
                                joint_transform       = joint_transform * glm::translate(glm::mat4(1.0f), translation);
                            }
                            break;
                        }
                    }

                    traverse(child_link, joint_transform, link->name);
                }
            };

        traverse(model->getRoot(), impl_->rootWorldTransform(), "");
        return true;
    }

    void RobotScene::updateTransforms() {
        if (!impl_->urdf_model) {
            return;
        }

        impl_->transforms.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse;
        traverse = [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform) {
            impl_->transforms[link->name] = parent_transform;

            for (auto& child_link : link->child_links) {
                auto joint = child_link->parent_joint;
                if (!joint) {
                    continue;
                }

                urdf::Vector3 p  = joint->parent_to_joint_origin_transform.position;
                urdf::Rotation r = joint->parent_to_joint_origin_transform.rotation;
                double roll      = 0.0;
                double pitch     = 0.0;
                double yaw       = 0.0;
                r.getRPY(roll, pitch, yaw);

                glm::mat4 joint_origin = glm::translate(glm::mat4(1.0f), glm::vec3(p.x, p.y, p.z)) *
                                         glm::mat4_cast(glm::quat(glm::vec3((float)roll, (float)pitch, (float)yaw)));

                glm::mat4 joint_motion(1.0f);

                glm::mat4 joint_frame_world = parent_transform * joint_origin;
                auto axis_it                = impl_->joint_axis_index.find(joint->name);
                if (axis_it != impl_->joint_axis_index.end()) {
                    JointAxisState& axis_state = impl_->joint_axis_states[axis_it->second];
                    axis_state.world_origin    = glm::vec3(joint_frame_world[3]);
                    glm::vec3 axis_world       = glm::mat3(joint_frame_world) * axis_state.axis_local;
                    if (glm::length(axis_world) < 1e-6f) {
                        axis_world = glm::vec3(0.0f, 0.0f, 1.0f);
                    }
                    axis_state.world_axis = glm::normalize(axis_world);
                }

                for (const auto& js : impl_->joint_states) {
                    if (js.name != joint->name) {
                        continue;
                    }

                    if (impl_->fixed_base_mode && IsBaseMotionJointName(joint->name)) {
                        break;
                    }

                    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        if (glm::length(axis) < 1e-6f) {
                            axis = glm::vec3(0, 0, 1);
                        }
                        joint_motion = glm::rotate(glm::mat4(1.0f), js.position, glm::normalize(axis));
                    } else if (joint->type == urdf::Joint::PRISMATIC) {
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        if (glm::length(axis) < 1e-6f) {
                            axis = glm::vec3(1, 0, 0);
                        }
                        joint_motion = glm::translate(glm::mat4(1.0f), js.position * glm::normalize(axis));
                    }
                    break;
                }

                glm::mat4 child_transform = parent_transform * joint_origin * joint_motion;
                traverse(child_link, child_transform);
            }
        };

        traverse(impl_->urdf_model->getRoot(), impl_->rootWorldTransform());
        impl_->pick_mesh_cache_dirty         = true;
        impl_->collision_proxies_cache_valid = false;
    }

    void RobotScene::draw(GLuint shader, const SceneDrawStyle& style) {
        const GLint loc_alpha = glGetUniformLocation(shader, "materialAlpha");

        if (style.show_visual_meshes) {
            if (style.wireframe_visuals) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            }
            if (loc_alpha >= 0) {
                glUniform1f(loc_alpha, 1.0f);
            }
            for (auto& [visual_key, lv] : impl_->visuals) {
                (void)visual_key;
                if (!lv.loaded) {
                    continue;
                }

                auto it = impl_->transforms.find(lv.parent_link_name);
                if (it == impl_->transforms.end()) {
                    continue;
                }

                glm::mat4 link_global = it->second;
                glm::mat4 model_mat   = link_global * lv.local_transform;
                model_mat             = model_mat * glm::scale(glm::mat4(1.0f), lv.scale);

                const bool highlight_visual = linkVisualNeedsHighlight(lv.parent_link_name, style);
                glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
                for (auto& mesh : lv.model.meshes) {
                    if (highlight_visual) {
                        const glm::vec3 highlight_color = linkVisualHighlightColor(lv.parent_link_name, style);
                        mesh.draw(shader, &highlight_color, true);
                    } else if (mesh.hasValidTexture()) {
                        mesh.draw(shader, nullptr, false);
                    } else {
                        const glm::vec3 base_color = lv.has_urdf_color ? lv.urdf_color : mesh.diffuse_color;
                        mesh.draw(shader, &base_color, true);
                    }
                }
            }
            if (style.wireframe_visuals) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
        }

        if (!style.show_collision_bodies) {
            return;
        }

        const glm::vec3 collision_color(0.58f, 0.28f, 0.92f);
        const bool highlight_collision = !style.hovered_link.empty() || !style.selected_link.empty();

        for (auto& cv : impl_->collision_visuals) {
            if (!cv.loaded) {
                continue;
            }
            auto it = impl_->transforms.find(cv.link_name);
            if (it == impl_->transforms.end()) {
                continue;
            }

            glm::mat4 model_mat = it->second * cv.local_transform;
            if (cv.kind == CollisionGeomKind::Box) {
                model_mat = model_mat * glm::scale(glm::mat4(1.0f), cv.params);
            } else if (cv.kind == CollisionGeomKind::Sphere) {
                const float r = std::max(cv.params.x, 1e-4f);
                model_mat     = model_mat * glm::scale(glm::mat4(1.0f), glm::vec3(2.0f * r));
            } else if (cv.kind == CollisionGeomKind::Cylinder) {
                model_mat = model_mat * glm::scale(glm::mat4(1.0f), glm::vec3(2.0f * cv.params.x, 2.0f * cv.params.y, cv.params.z));
            } else if (cv.kind == CollisionGeomKind::Mesh) {
                model_mat = model_mat * glm::scale(glm::mat4(1.0f), cv.params);
            }

            glm::vec3 draw_color = collision_color;
            if (!style.selected_link.empty() && cv.link_name == style.selected_link) {
                draw_color = glm::vec3(0.35f, 0.90f, 1.0f);
            } else if (!style.hovered_link.empty() && cv.link_name == style.hovered_link) {
                draw_color = glm::vec3(1.0f, 0.78f, 0.30f);
            }

            const float alpha =
                (highlight_collision && (cv.link_name == style.hovered_link || cv.link_name == style.selected_link)) ? 0.72f : 0.48f;
            if (loc_alpha >= 0) {
                glUniform1f(loc_alpha, alpha);
            }

            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
            for (auto& mesh : cv.model.meshes) {
                mesh.draw(shader, &draw_color, true);
            }
        }
        if (loc_alpha >= 0) {
            glUniform1f(loc_alpha, 1.0f);
        }
    }

    size_t RobotScene::applyJointSamples(const std::vector<SensorJointSample>& samples, bool only_master_arm) {
        size_t applied = 0;
        for (const auto& sample : samples) {
            if (!sample.has_position || !std::isfinite(sample.position)) {
                continue;
            }
            if (only_master_arm && !IsMasterArmGroup(sample.group)) {
                continue;
            }

            if (setJointPositionByName(sample.name, static_cast<float>(sample.position))) {
                applied++;
            }
        }
        return applied;
    }

    bool RobotScene::setJointPositionByName(const std::string& joint_name, float new_position) {
        for (auto& js : impl_->joint_states) {
            if (js.name == joint_name) {
                if (std::fabs(js.position - new_position) < 1e-7f) {
                    return true;
                }
                js.position             = new_position;
                impl_->joint_pose_dirty = true;
                return true;
            }
        }
        return false;
    }

    bool RobotScene::consumeJointPoseDirty() {
        const bool dirty        = impl_->joint_pose_dirty;
        impl_->joint_pose_dirty = false;
        return dirty;
    }

    bool RobotScene::isJointPoseDirty() const {
        return impl_->joint_pose_dirty;
    }

    bool RobotScene::getJointInfo(const std::string& joint_name, JointInfo* out) const {
        if (!out) {
            return false;
        }

        for (const auto& js : impl_->joint_states) {
            if (js.name == joint_name) {
                out->name      = js.name;
                out->position  = js.position;
                out->min_angle = js.min_angle;
                out->max_angle = js.max_angle;
                out->revolute  = js.revolute;
                return true;
            }
        }
        return false;
    }

    std::vector<RobotScene::JointInfo> RobotScene::getJointInfos() const {
        std::vector<JointInfo> infos;
        infos.reserve(impl_->joint_states.size());

        for (const auto& js : impl_->joint_states) {
            JointInfo info;
            info.name      = js.name;
            info.position  = js.position;
            info.min_angle = js.min_angle;
            info.max_angle = js.max_angle;
            info.revolute  = js.revolute;
            infos.push_back(std::move(info));
        }

        return infos;
    }

    std::vector<RobotScene::JointAxisInfo> RobotScene::getJointAxisInfos(bool revolute_only) const {
        std::vector<JointAxisInfo> infos;
        infos.reserve(impl_->joint_axis_states.size());

        for (const auto& axis_state : impl_->joint_axis_states) {
            if (revolute_only && !axis_state.revolute) {
                continue;
            }
            JointAxisInfo info;
            info.name         = axis_state.name;
            info.world_origin = axis_state.world_origin;
            info.world_axis   = axis_state.world_axis;
            info.revolute     = axis_state.revolute;
            infos.push_back(std::move(info));
        }

        return infos;
    }

    std::vector<RobotScene::LinkTfInfo> RobotScene::getLinkTfInfos() const {
        std::vector<LinkTfInfo> infos;
        infos.reserve(impl_->transforms.size());

        for (const auto& [link_name, tf] : impl_->transforms) {
            LinkTfInfo info;
            info.name      = link_name;
            auto parent_it = impl_->link_parent.find(link_name);
            if (parent_it != impl_->link_parent.end()) {
                info.parent_name = parent_it->second;
            }
            info.world_position = glm::vec3(tf[3]);

            glm::quat q     = glm::quat_cast(tf);
            glm::vec3 euler = glm::eulerAngles(q);
            info.world_rpy  = euler;
            infos.push_back(std::move(info));
        }

        return infos;
    }

    std::vector<RobotScene::LinkCollisionProxy> RobotScene::getLinkCollisionProxies() const {
        if (impl_->collision_proxies_cache_valid) {
            return impl_->cached_collision_proxies;
        }

        impl_->cached_collision_proxies      = impl_->buildLinkCollisionProxies();
        impl_->collision_proxies_cache_valid = true;
        return impl_->cached_collision_proxies;
    }

    void RobotScene::appendLinkWorldCollisionTriangles(const std::string& link_name, std::vector<glm::vec3>* out_triangles) const {
        impl_->AppendWorldCollisionTrianglesForLink(link_name, out_triangles);
    }

    bool RobotScene::getLinkWorldTransform(const std::string& link_name, glm::mat4* out_world_transform) const {
        if (out_world_transform == nullptr) {
            return false;
        }
        const auto it = impl_->transforms.find(link_name);
        if (it == impl_->transforms.end()) {
            return false;
        }
        *out_world_transform = it->second;
        return true;
    }

    bool RobotScene::getLinkParentName(const std::string& link_name, std::string* out_parent_name) const {
        if (out_parent_name == nullptr) {
            return false;
        }
        auto it = impl_->link_parent.find(link_name);
        if (it == impl_->link_parent.end()) {
            return false;
        }
        *out_parent_name = it->second;
        return true;
    }

    bool RobotScene::getParentJointNameForLink(const std::string& link_name, std::string* out_joint_name) const {
        if (out_joint_name == nullptr) {
            return false;
        }
        const auto it = impl_->link_to_parent_joint.find(link_name);
        if (it == impl_->link_to_parent_joint.end()) {
            return false;
        }
        *out_joint_name = it->second;
        return true;
    }

    bool RobotScene::getJointDetail(const std::string& joint_name, JointDetailInfo* out) const {
        if (out == nullptr) {
            return false;
        }
        const auto it = impl_->joint_details.find(joint_name);
        if (it == impl_->joint_details.end()) {
            return false;
        }
        *out = it->second;
        for (const auto& js : impl_->joint_states) {
            if (js.name == joint_name) {
                out->position = js.position;
                break;
            }
        }
        return true;
    }

    std::vector<RobotScene::LinkComInfo> RobotScene::getLinkComWorldPositions() const {
        std::vector<LinkComInfo> out;
        for (const auto& [link_name, inertial] : impl_->link_inertials) {
            const auto it = impl_->transforms.find(link_name);
            if (it == impl_->transforms.end()) {
                continue;
            }
            const glm::vec4 world = it->second * inertial.local_transform * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            LinkComInfo info;
            info.link_name      = link_name;
            info.world_position = glm::vec3(world);
            out.push_back(info);
        }
        return out;
    }

    bool RobotScene::pickLinkByRay(const glm::vec3& ray_origin, const glm::vec3& ray_dir, std::string* out_link_name, float* out_hit_t,
                                   LinkPickMode mode) {
        if (out_link_name == nullptr || glm::length(ray_dir) < 1e-8f) {
            return false;
        }
        const glm::vec3 dir = glm::normalize(ray_dir);

        if (mode == LinkPickMode::Accurate) {
            if (impl_->pick_mesh_cache_dirty) {
                impl_->rebuildPickMeshCache();
                impl_->pick_mesh_cache_dirty = false;
            }
            std::string mesh_best_link;
            float mesh_best_t = 1e9f;
            for (const auto& tri : impl_->pick_mesh_triangles) {
                float hit_t = 0.0f;
                if (!intersectRayTriangle(ray_origin, dir, tri.v0, tri.v1, tri.v2, &hit_t)) {
                    continue;
                }
                if (hit_t < mesh_best_t) {
                    mesh_best_t    = hit_t;
                    mesh_best_link = tri.link_name;
                }
            }
            if (!mesh_best_link.empty()) {
                *out_link_name = mesh_best_link;
                if (out_hit_t != nullptr) {
                    *out_hit_t = mesh_best_t;
                }
                return true;
            }
        }

        auto intersectRaySphere = [](const glm::vec3& ray_o, const glm::vec3& ray_d, const glm::vec3& c, float r, float* out_t) {
            const glm::vec3 oc = ray_o - c;
            const float a      = glm::dot(ray_d, ray_d);
            const float b      = 2.0f * glm::dot(oc, ray_d);
            const float cc     = glm::dot(oc, oc) - r * r;
            const float disc   = b * b - 4.0f * a * cc;
            if (disc < 0.0f) {
                return false;
            }
            const float sqrt_disc = std::sqrt(disc);
            const float t0        = (-b - sqrt_disc) / (2.0f * a);
            const float t1        = (-b + sqrt_disc) / (2.0f * a);
            float t_hit           = -1.0f;
            if (t0 > 0.0f) {
                t_hit = t0;
            } else if (t1 > 0.0f) {
                t_hit = t1;
            }
            if (t_hit <= 0.0f) {
                return false;
            }
            if (out_t != nullptr) {
                *out_t = t_hit;
            }
            return true;
        };

        std::unordered_map<std::string, float> best_t_by_link;
        for (const auto& proxy : getLinkCollisionProxies()) {
            float hit_t             = 0.0f;
            const float pick_radius = std::max(proxy.radius_m * 1.05f, 0.02f);
            if (!intersectRaySphere(ray_origin, dir, proxy.world_center, pick_radius, &hit_t)) {
                continue;
            }
            auto it = best_t_by_link.find(proxy.link_name);
            if (it == best_t_by_link.end() || hit_t < it->second) {
                best_t_by_link[proxy.link_name] = hit_t;
            }
        }

        std::string best_link;
        float best_t = 1e9f;
        for (const auto& kv : best_t_by_link) {
            if (kv.second < best_t) {
                best_t    = kv.second;
                best_link = kv.first;
            }
        }
        if (best_link.empty()) {
            return false;
        }
        *out_link_name = best_link;
        if (out_hit_t != nullptr) {
            *out_hit_t = best_t;
        }
        return true;
    }

    const std::string& RobotScene::urdfFilePath() const {
        return impl_->urdf_file_path;
    }

    void RobotScene::setFixedBaseMode(bool enabled) {
        impl_->fixed_base_mode = enabled;
    }
    bool RobotScene::fixedBaseMode() const {
        return impl_->fixed_base_mode;
    }

    void RobotScene::setVirtualBasePose2D(float x_m, float y_m, float yaw_rad) {
        impl_->virtual_base_x_m     = x_m;
        impl_->virtual_base_y_m     = y_m;
        impl_->virtual_base_yaw_rad = yaw_rad;
    }

    bool RobotScene::getVirtualBasePose2D(float* x_m, float* y_m, float* yaw_rad) const {
        if (x_m == nullptr || y_m == nullptr || yaw_rad == nullptr) {
            return false;
        }
        *x_m     = impl_->virtual_base_x_m;
        *y_m     = impl_->virtual_base_y_m;
        *yaw_rad = impl_->virtual_base_yaw_rad;
        return true;
    }

    glm::vec3 OrbitCamera::eye() const {
        float x = distance * cosf(pitch) * cosf(yaw);
        float y = distance * cosf(pitch) * sinf(yaw);
        float z = distance * sinf(pitch);
        return target + glm::vec3(x, y, z);
    }

    glm::mat4 OrbitCamera::viewMatrix() const {
        return glm::lookAt(eye(), target, glm::vec3(0, 0, 1));
    }

    void OrbitCamera::rotate(float dx, float dy) {
        yaw += rotate_speed * dx;
        pitch += rotate_speed * dy;
        pitch = std::clamp(pitch, -1.55f, 1.55f);
    }

    void OrbitCamera::zoom(float delta) {
        distance *= (1.0f - zoom_scale * delta);
        distance = std::clamp(distance, min_distance, max_distance);
    }

    void OrbitCamera::dolly(float dy) {
        zoom(dolly_scale * dy);
    }

    void OrbitCamera::pan(float dx, float dy) {
        glm::vec3 camera_eye = eye();
        glm::vec3 forward    = glm::normalize(target - camera_eye);
        glm::vec3 world_up(0.0f, 0.0f, 1.0f);

        glm::vec3 right = glm::cross(forward, world_up);
        if (glm::length(right) < 1e-6f) {
            right = glm::vec3(1.0f, 0.0f, 0.0f);
        } else {
            right = glm::normalize(right);
        }

        glm::vec3 up = glm::normalize(glm::cross(right, forward));
        target += (-right * dx + up * dy) * (pan_scale * distance);
    }

}  // namespace rkv
