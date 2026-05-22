#include "kinematic_viewer/kinematic_bootstrap.h"

#include <string>

namespace kinematic_viewer {
    namespace bootstrap_internal {

        std::string ConfigOrUrdfFromArgs(int argc, char** argv) {
            if (argc <= 1) {
                return "config/robot_kinematic_viewer.yaml";
            }
            return argv[1];
        }

        bool IsUrdfFilePath(const std::string& path) {
            return path.size() > 5 && path.substr(path.size() - 5) == ".urdf";
        }

    }  // namespace bootstrap_internal

    bool LoadLaunchConfigFromArgs(int argc, char** argv, LaunchConfig* out, std::string* errorMessage) {
        if (out == nullptr) {
            if (errorMessage != nullptr) {
                *errorMessage = "输出参数为空";
            }
            return false;
        }

        LaunchConfig launch;
        std::string configOrUrdf = bootstrap_internal::ConfigOrUrdfFromArgs(argc, argv);

        if (bootstrap_internal::IsUrdfFilePath(configOrUrdf)) {
            launch.urdfPath = configOrUrdf;
            *out            = launch;
            return true;
        }

        bool loadedOk = false;
        launch.config = KinematicViewerConfig::LoadFromFile(configOrUrdf, &loadedOk);
        if (!loadedOk) {
            if (errorMessage != nullptr) {
                *errorMessage = "[robot_kinematic_viewer] 配置加载失败，请使用独立配置文件: " + configOrUrdf;
            }
            return false;
        }
        launch.configPath = configOrUrdf;
        launch.urdfPath   = launch.config.robot.urdf_path;
        *out              = launch;
        return true;
    }

}  // namespace kinematic_viewer
