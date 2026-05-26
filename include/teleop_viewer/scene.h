#pragma once

#include <glad/glad.h>

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

#include "teleop_viewer/types.h"

namespace teleop_viewer {

    class OrbitCamera {
       public:
        float distance   = 3.0f;
        float yaw        = 0.0f;
        float pitch      = 0.0f;
        glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);

        float rotate_speed = 0.005f;
        float zoom_scale   = 0.1f;
        float dolly_scale  = 0.02f;
        float pan_scale    = 0.0015f;
        float min_distance = 0.2f;
        float max_distance = 20.0f;

        glm::vec3 eye() const;
        glm::mat4 viewMatrix() const;

        void rotate(float dx, float dy);
        void zoom(float delta);
        void dolly(float dy);
        void pan(float dx, float dy);
    };

    class RobotScene {
       public:
        struct JointInfo {
            std::string name;
            float position  = 0.0f;
            float min_angle = -3.14f;
            float max_angle = 3.14f;
            bool revolute   = false;
        };
        struct JointAxisInfo {
            std::string name;
            glm::vec3 world_origin = glm::vec3(0.0f);
            glm::vec3 world_axis   = glm::vec3(0.0f, 0.0f, 1.0f);
            bool revolute          = false;
        };
        struct LinkTfInfo {
            std::string name;
            std::string parent_name;
            glm::vec3 world_position = glm::vec3(0.0f);
            glm::vec3 world_rpy      = glm::vec3(0.0f);
        };
        struct LinkCollisionProxy {
            std::string link_name;
            std::string visual_name;
            glm::vec3 world_center = glm::vec3(0.0f);
            float radius_m         = 0.0f;
        };

        struct LinkComInfo {
            std::string link_name;
            glm::vec3 world_position = glm::vec3(0.0f);
        };

        struct SceneDrawStyle {
            bool show_visual_meshes    = true;
            bool show_collision_bodies = false;
            bool wireframe_visuals     = false;
            std::string hovered_link;
            std::string selected_link;
        };

        struct JointDetailInfo {
            std::string name;
            std::string parent_link;
            std::string child_link;
            std::string type;
            glm::vec3 axis_local = glm::vec3(0.0f, 0.0f, 1.0f);
            float position       = 0.0f;
            float lower_limit    = 0.0f;
            float upper_limit    = 0.0f;
            float velocity_limit = -1.0f;  // < 0 means not specified in URDF
            bool revolute        = false;
            bool has_limits      = false;
        };

        RobotScene();
        ~RobotScene();

        bool loadURDF(const std::string& urdf_path);

        void updateTransforms();
        void draw(GLuint shader, const SceneDrawStyle& style);

        size_t applyJointSamples(const std::vector<SensorJointSample>& samples, bool only_master_arm);
        bool setJointPositionByName(const std::string& joint_name, float new_position);
        bool consumeJointPoseDirty();

        bool getJointInfo(const std::string& joint_name, JointInfo* out) const;
        std::vector<JointInfo> getJointInfos() const;
        std::vector<JointAxisInfo> getJointAxisInfos(bool revolute_only = true) const;
        std::vector<LinkTfInfo> getLinkTfInfos() const;
        std::vector<LinkCollisionProxy> getLinkCollisionProxies() const;
        std::vector<LinkComInfo> getLinkComWorldPositions() const;
        bool getLinkWorldTransform(const std::string& link_name, glm::mat4* out_world_transform) const;
        bool getLinkParentName(const std::string& link_name, std::string* out_parent_name) const;
        bool getParentJointNameForLink(const std::string& link_name, std::string* out_joint_name) const;
        bool getJointDetail(const std::string& joint_name, JointDetailInfo* out) const;
        enum class LinkPickMode {
            Fast,      // collision-sphere proxies only (for per-frame hover)
            Accurate,  // mesh triangles when cache is warm; rebuilds cache on pose change
        };

        bool pickLinkByRay(const glm::vec3& ray_origin, const glm::vec3& ray_dir, std::string* out_link_name, float* out_hit_t = nullptr,
                           LinkPickMode mode = LinkPickMode::Fast);
        const std::string& urdfFilePath() const;

        void setFixedBaseMode(bool enabled);
        bool fixedBaseMode() const;
        void setVirtualBasePose2D(float x_m, float y_m, float yaw_rad);
        bool getVirtualBasePose2D(float* x_m, float* y_m, float* yaw_rad) const;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace teleop_viewer
