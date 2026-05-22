cmake_minimum_required(VERSION 3.16)

project(robot_kinematic_viewer VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The target C++ standard.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose build type." FORCE)
endif()

if(NOT ARCH_FREFIX)
    set(ARCH_FREFIX ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION})
endif()
if(NOT ARCH_PREFIX)
    set(ARCH_PREFIX ${ARCH_FREFIX})
endif()

if(NOT ROOT_DEVEL_ARCH_PATH)
    set(ROOT_DEVEL_ARCH_PATH /opt/galbot/devel/${ARCH_FREFIX})
endif()
if(NOT CONTROL_DEVEL_ARCH_PATH)
    set(CONTROL_DEVEL_ARCH_PATH /opt/galbot/devel_control/${ARCH_FREFIX})
endif()

list(APPEND CMAKE_PREFIX_PATH ${ROOT_DEVEL_ARCH_PATH} ${CONTROL_DEVEL_ARCH_PATH})

find_package(trac_ik REQUIRED)
find_package(OpenGL REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(GLEW REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tinyxml2 QUIET)
find_package(qpOASES QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ROSCPP REQUIRED IMPORTED_TARGET roscpp)

find_library(YAMLCPP_LIBRARY NAMES yaml-cpp PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib /opt/galbot/devel/${ARCH_FREFIX}/lib)
if(NOT YAMLCPP_LIBRARY)
    message(FATAL_ERROR "libyaml-cpp not found; install libyaml-cpp-dev or set YAMLCPP_LIBRARY")
endif()
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h PATHS /usr/include /usr/local/include /opt/galbot/devel/${ARCH_FREFIX}/include)

if(TARGET TinyXML2::TinyXML2 AND NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 ALIAS TinyXML2::TinyXML2)
endif()
if(NOT TARGET tinyxml2::tinyxml2)
    find_library(TINYXML2_LIBRARY NAMES tinyxml2)
    if(TINYXML2_LIBRARY)
        add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
        set_target_properties(tinyxml2::tinyxml2 PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIBRARY}")
    endif()
endif()

if(NOT qpOASES_FOUND)
    find_library(QPOASES_LIBRARY NAMES qpOASES
                 PATHS /opt/galbot/devel_control/${ARCH_FREFIX}/lib
                       /opt/galbot/devel_control/${ARCH_PREFIX}/lib
                       /opt/galbot/devel_control/x86_64-Linux-GNU-9.4.0/lib)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")

add_subdirectory(deps/imgui)

add_library(imgui_glfw_support SHARED
    deps/imgui/imgui_impl_glfw.cpp
    deps/imgui/imgui_impl_opengl3.cpp
)
target_link_libraries(imgui_glfw_support PUBLIC imgui glfw OpenGL::GL ${CMAKE_DL_LIBS})
target_include_directories(imgui_glfw_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui")

add_library(imguizmo SHARED deps/imguizmo/ImGuizmo.cpp)
target_link_libraries(imguizmo PUBLIC imgui)
target_include_directories(imguizmo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui" "${CMAKE_CURRENT_SOURCE_DIR}/deps/imguizmo")

add_library(flex_ik_core src/ik_backends/flex_ik/flex_ik.cc)
target_include_directories(flex_ik_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/flex_ik")
target_link_libraries(flex_ik_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)

add_library(wbc_chain_ik_core
    src/ik_backends/wbc_chain_ik/robot/traits/chain_ik.cpp
    src/ik_backends/wbc_chain_ik/solver/hqp_solver.cpp
    src/ik_backends/wbc_chain_ik/solver/qp_solver.cpp
    src/ik_backends/wbc_chain_ik/math/helper_functions.cpp
)
target_include_directories(wbc_chain_ik_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/wbc_chain_ik")
target_link_libraries(wbc_chain_ik_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)

# Velocity Planning Library (from trajectory_planner_utils)
add_library(velocity_planning
    deps/vp/src/trapezoidal/trapezoidal_planner.cpp
    deps/vp/src/double_s/double_s_planner.cpp
    deps/vp/src/multi_velocity_planner.cpp
    deps/vp/src/curve_interface/constant_jerk_trajectory.cpp
    deps/vp/src/curve_interface/piecewise_trajectory.cpp
    deps/vp/src/geometry_trajectory/straight_trajectory.cpp
)
target_include_directories(velocity_planning PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/vp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/vp/include/vp"
)
if(EIGEN3_FOUND)
    target_link_libraries(velocity_planning PUBLIC Eigen3::Eigen)
endif()

add_executable(robot_kinematic_viewer
    src/robot_kinematic_viewer.cpp
    src/kinematic_viewer/kinematic_app.cpp
    src/kinematic_viewer/kinematic_ik_controller.cpp
    src/kinematic_viewer/kinematic_string_utils.cpp
    src/kinematic_viewer/kinematic_viewer_config.cpp
    src/kinematic_viewer/kinematic_render_loop.cpp
    src/kinematic_viewer/kinematic_input_handler.cpp
    src/kinematic_viewer/kinematic_config_watcher.cpp
    src/kinematic_viewer/kinematic_playback_state_machine.cpp
    src/kinematic_viewer/kinematic_bootstrap.cpp
    src/kinematic_viewer/kinematic_playback.cpp
    src/kinematic_viewer/kinematic_collision_monitor.cpp
    src/kinematic_viewer/kinematic_initial_pose.cpp
    src/kinematic_viewer/kinematic_marker_target_state.cpp
    src/kinematic_viewer/kinematic_ros_bridge.cpp
    src/kinematic_viewer/kinematic_sidebar_panels.cpp
    src/kinematic_viewer/kinematic_ui_theme.cpp
    src/kinematic_viewer/kinematic_ui_feedback.cpp
    src/kinematic_viewer/kinematic_marker_utils.cpp
    src/kinematic_viewer/kinematic_shader_utils.cpp
    src/kinematic_viewer/kinematic_line_renderer.cpp
    src/kinematic_viewer/kinematic_user_obstacles.cpp
    src/kinematic_viewer/kinematic_path_planner.cpp
    src/teleop_viewer/ik_solver.cpp
    src/teleop_viewer/scene.cpp
    deps/glad/glad.c
)

target_include_directories(robot_kinematic_viewer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/flex_ik"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/wbc_chain_ik"
    /opt/ros/noetic/include
    ${trac_ik_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIR}
)

target_link_libraries(robot_kinematic_viewer PRIVATE
    imgui_glfw_support
    imguizmo
    assimp
    urdfdom_model
    urdfdom_world
    urdfdom_sensor
    ${trac_ik_LIBRARIES}
    flex_ik_core
    wbc_chain_ik_core
    velocity_planning
    OpenGL::GL
    glfw
    GLEW::GLEW
    PkgConfig::ROSCPP
    ${YAMLCPP_LIBRARY}
    pthread
    ${CMAKE_DL_LIBS}
)

if(qpOASES_FOUND)
    target_link_libraries(robot_kinematic_viewer PRIVATE ${qpOASES_LIBRARIES})
elseif(QPOASES_LIBRARY)
    target_link_libraries(robot_kinematic_viewer PRIVATE ${QPOASES_LIBRARY})
else()
    message(WARNING "qpOASES not explicitly linked; full-body IK backends may fail to link.")
endif()

file(COPY config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
