#include "kinematic_viewer/kinematic_collision_distance.h"
#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "rkv/scene.h"

#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

    constexpr const char* kGalbotUrdfPath =
        "/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/galbot_description/galbot_one_golf_description/"
        "galbot_one_golf.urdf";

    int g_failures = 0;

    bool InitHeadlessGl() {
        if (!glfwInit()) {
            return false;
        }
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        GLFWwindow* window = glfwCreateWindow(640, 480, "collision_distance_test", nullptr, nullptr);
        if (window == nullptr) {
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(window);
        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
            glfwDestroyWindow(window);
            glfwTerminate();
            return false;
        }
        return true;
    }

    void ExpectNonNegative(float value, const char* message) {
        if (value < 0.0f) {
            ++g_failures;
            std::cerr << "[FAIL] " << message << " (value=" << value << ")\n";
        }
    }

    void ExpectSeparatedWhenCentersAreFar(const kinematic_viewer::CollisionPairDistance& distance, float center_threshold_m,
                                          float surface_threshold_m, const char* message) {
        if (distance.center_distance_m >= center_threshold_m && distance.surface_distance_m < surface_threshold_m) {
            ++g_failures;
            std::cerr << "[FAIL] " << message << " (center=" << distance.center_distance_m << " m, surface=" << distance.surface_distance_m
                      << " m)\n";
        }
    }

    void ExpectTrue(bool condition, const char* message) {
        if (!condition) {
            ++g_failures;
            std::cerr << "[FAIL] " << message << '\n';
        }
    }

    void ExpectGreater(float value, float threshold, const char* message) {
        if (!(value > threshold)) {
            ++g_failures;
            std::cerr << "[FAIL] " << message << " (value=" << value << ", threshold=" << threshold << ")\n";
        }
    }

    std::optional<rkv::RobotScene::LinkCollisionProxy> FindProxyByLinkName(const std::vector<rkv::RobotScene::LinkCollisionProxy>& proxies,
                                                                           const std::string& link_name) {
        for (const auto& proxy : proxies) {
            if (proxy.link_name == link_name) {
                return proxy;
            }
        }
        return std::nullopt;
    }

    kinematic_viewer::CollisionPairDistance DistanceForLinks(const rkv::RobotScene& scene, const std::string& link_a,
                                                             const std::string& link_b) {
        const auto proxies = scene.getLinkCollisionProxies();
        const auto proxy_a = FindProxyByLinkName(proxies, link_a);
        const auto proxy_b = FindProxyByLinkName(proxies, link_b);
        if (!proxy_a || !proxy_b) {
            return {};
        }
        return kinematic_viewer::RefineCollisionPairDistanceWithMesh(scene,
                                                                     kinematic_viewer::BuildCollisionPairDistanceAabb(*proxy_a, *proxy_b));
    }

    void TestSyntheticTriangleSoupDistance() {
        const std::vector<glm::vec3> tri_a = {
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.1f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.1f, 0.0f),
        };
        const std::vector<glm::vec3> tri_b = {
            glm::vec3(0.0f, 0.0f, 0.2f),
            glm::vec3(0.1f, 0.0f, 0.2f),
            glm::vec3(0.0f, 0.1f, 0.2f),
        };
        const auto result = kinematic_viewer::MinDistanceBetweenTriangleSoups(tri_a, tri_b);
        ExpectTrue(std::fabs(result.distance_m - 0.2f) < 1e-3f, "synthetic triangle soups should be 0.2 m apart");
    }

    void TestGalbotDefaultPoseDistances(rkv::RobotScene* scene) {
        const auto leg_link2_vs_leg_link4 = DistanceForLinks(*scene, "leg_link2", "leg_link4");
        ExpectNonNegative(leg_link2_vs_leg_link4.surface_distance_m, "leg_link2 vs leg_link4 surface distance must be non-negative");
        ExpectSeparatedWhenCentersAreFar(leg_link2_vs_leg_link4, 0.25f, 0.001f,
                                         "leg_link2 vs leg_link4 should not collapse to near-zero when centers are far apart");
        std::cout << "  leg_link2 <-> leg_link4: center=" << leg_link2_vs_leg_link4.center_distance_m
                  << " m, surface=" << leg_link2_vs_leg_link4.surface_distance_m << " m\n";

        const auto torso_vs_left_arm = DistanceForLinks(*scene, "torso_base_link", "left_arm_link2");
        ExpectNonNegative(torso_vs_left_arm.surface_distance_m, "torso_base_link vs left_arm_link2 surface distance must be non-negative");
        ExpectSeparatedWhenCentersAreFar(torso_vs_left_arm, 0.15f, 0.001f,
                                         "torso_base_link vs left_arm_link2 should not collapse to near-zero when centers are far apart");
        std::cout << "  torso_base_link <-> left_arm_link2: center=" << torso_vs_left_arm.center_distance_m
                  << " m, surface=" << torso_vs_left_arm.surface_distance_m << " m\n";

        const auto adjacent = DistanceForLinks(*scene, "leg_link1", "leg_link2");
        ExpectNonNegative(adjacent.surface_distance_m, "adjacent leg links should not report negative distance");
        std::cout << "  leg_link1 <-> leg_link2: center=" << adjacent.center_distance_m << " m, surface=" << adjacent.surface_distance_m
                  << " m\n";
    }

    void TestConnectedLinkFilter(rkv::RobotScene* scene) {
        kinematic_viewer::CollisionMonitor monitor;
        kinematic_viewer::CollisionMonitorState state;
        state.enable              = true;
        state.ignore_same_link    = true;
        state.ignore_parent_child = true;

        const auto result = monitor.Evaluate(state, *scene);
        ExpectTrue(result.valid, "collision monitor should produce a closest pair on galbot");

        const bool reports_connected_leg_pair =
            (result.closest_pair.link_a == "omni_chassis_base_link" && result.closest_pair.link_b == "leg_link1") ||
            (result.closest_pair.link_a == "leg_link1" && result.closest_pair.link_b == "omni_chassis_base_link") ||
            (result.closest_pair.link_a == "leg_link4" && result.closest_pair.link_b == "torso_base_link") ||
            (result.closest_pair.link_a == "torso_base_link" && result.closest_pair.link_b == "leg_link4") ||
            (result.closest_pair.link_a == "left_arm_link5" && result.closest_pair.link_b == "left_arm_link7") ||
            (result.closest_pair.link_a == "left_arm_link7" && result.closest_pair.link_b == "left_arm_link5") ||
            (result.closest_pair.link_a == "leg_link2" && result.closest_pair.link_b == "leg_link4") ||
            (result.closest_pair.link_a == "leg_link4" && result.closest_pair.link_b == "leg_link2") ||
            (result.closest_pair.link_a == "torso_base_link" && result.closest_pair.link_b == "head_link2") ||
            (result.closest_pair.link_a == "head_link2" && result.closest_pair.link_b == "torso_base_link");
        ExpectTrue(!reports_connected_leg_pair, "closest pair should not be a filtered connected-link false positive");
        std::cout << "  closest pair: " << result.closest_pair.link_a << " <-> " << result.closest_pair.link_b << " ("
                  << result.closest_pair.surface_distance_m << " m)\n";
    }

}  // namespace

int main() {
    TestSyntheticTriangleSoupDistance();

    if (!InitHeadlessGl()) {
        std::cerr << "[FAIL] unable to initialize headless OpenGL context\n";
        return 1;
    }

    rkv::RobotScene scene;
    if (!scene.loadURDF(kGalbotUrdfPath)) {
        std::cerr << "[FAIL] unable to load URDF: " << kGalbotUrdfPath << '\n';
        return 1;
    }
    scene.updateTransforms();

    std::cout << "Galbot collision distance tests:\n";
    TestGalbotDefaultPoseDistances(&scene);
    TestConnectedLinkFilter(&scene);

    if (g_failures > 0) {
        std::cerr << g_failures << " collision distance test(s) failed\n";
        return 1;
    }

    std::cout << "All collision distance tests passed.\n";
    return 0;
}
