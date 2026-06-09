#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace rkv {

    struct SensorJointSample {
        std::string group;
        std::string name;
        double position = 0.0;
        double velocity = 0.0;
        double effort   = 0.0;
        double current  = 0.0;

        bool has_position = false;
        bool has_velocity = false;
        bool has_effort   = false;
        bool has_current  = false;
    };

    struct JoyButtonSample {
        std::string name;
        int status = 0;
    };

    struct JoyAxisSample {
        std::string name;
        double value   = 0.0;
        bool has_value = false;
    };

    struct WbcGroupErrorSample {
        std::string group;
        int joint_count = 0;
        double pos_norm = 0.0;
        double vel_norm = 0.0;
        double eff_norm = 0.0;
    };

    struct RobotErrorSample {
        std::string component;
        int code = 0;
        std::string description;
    };

    inline std::string ToLowerCopy(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    inline bool IsLeftHandleKey(const std::string& name) {
        std::string lower = ToLowerCopy(name);
        if (lower.find("left") != std::string::npos || lower.find("polel_") != std::string::npos ||
            lower.find("triggerl_") != std::string::npos) {
            return true;
        }
        return !name.empty() && (name.back() == 'L' || name.back() == 'l');
    }

    inline bool IsRightHandleKey(const std::string& name) {
        std::string lower = ToLowerCopy(name);
        if (lower.find("right") != std::string::npos || lower.find("poler_") != std::string::npos ||
            lower.find("triggerr_") != std::string::npos) {
            return true;
        }
        return !name.empty() && (name.back() == 'R' || name.back() == 'r');
    }

    inline bool IsMasterArmGroup(const std::string& group_name) {
        return group_name == "left_arm" || group_name == "right_arm";
    }

    inline bool IsBaseMotionJointName(const std::string& joint_name) {
        return joint_name.find("chassis") != std::string::npos || joint_name.find("world") != std::string::npos ||
               joint_name.find("virtual") != std::string::npos || joint_name.find("base") != std::string::npos;
    }

}  // namespace rkv
