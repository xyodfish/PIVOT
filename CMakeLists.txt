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
find_package(glog REQUIRED) 
find_package(pinocchio REQUIRED)
find_package(tinyxml2 QUIET)
find_package(qpOASES QUIET)
find_package(PkgConfig REQUIRED)

find_package(yaml-cpp REQUIRED)

# FFmpeg / libav
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

# libgif
find_library(GIF_LIBRARY NAMES gif libgif)
find_path(GIF_INCLUDE_DIR NAMES gif_lib.h)
if(NOT GIF_LIBRARY OR NOT GIF_INCLUDE_DIR)
    message(FATAL_ERROR "libgif not found. Please install libgif-dev.")
endif()
if(NOT TARGET yaml-cpp)
    find_library(YAMLCPP_LIBRARY NAMES yaml-cpp yaml-cpp.so.0.7
                 PATHS ${ROOT_DEVEL_ARCH_PATH}/lib NO_DEFAULT_PATH)
    if(NOT YAMLCPP_LIBRARY)
        message(FATAL_ERROR "libyaml-cpp not found under ${ROOT_DEVEL_ARCH_PATH}/lib")
    endif()
    add_library(yaml-cpp UNKNOWN IMPORTED)
    set_target_properties(yaml-cpp PROPERTIES IMPORTED_LOCATION "${YAMLCPP_LIBRARY}")
    set_target_properties(yaml-cpp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ROOT_DEVEL_ARCH_PATH}/include")
endif()

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
                 PATHS /opt/galbot/devel/${ARCH_FREFIX}/lib
                       /opt/galbot/devel/${ARCH_PREFIX}/lib
                       /opt/galbot/devel/x86_64-Linux-GNU-9.4.0/lib)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib;${ROOT_DEVEL_ARCH_PATH}/lib")
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
set_target_properties(flex_ik_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(wbc_chain_ik_core
    src/ik_backends/wbc_chain_ik/robot/traits/chain_ik.cpp
    src/ik_backends/wbc_chain_ik/solver/hqp_solver.cpp
    src/ik_backends/wbc_chain_ik/solver/qp_solver.cpp
    src/ik_backends/wbc_chain_ik/math/helper_functions.cpp
)
target_include_directories(wbc_chain_ik_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/wbc_chain_ik")
target_link_libraries(wbc_chain_ik_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)
set_target_properties(wbc_chain_ik_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Velocity Planning Library (from trajectory_planner_utils)
add_library(velocity_planning
    deps/vp/src/trapezoidal/trapezoidal_planner.cpp
    deps/vp/src/double_s/double_s_planner.cpp
    deps/vp/src/double_s/double_s_multi_planner.cpp
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
set_target_properties(velocity_planning PROPERTIES POSITION_INDEPENDENT_CODE ON)

# PCL is loaded via a separate shared library (dlopen) to avoid heap conflicts with
# pinocchio/trac_ik when all DSOs initialize in the main executable.
set(RKV_PCL_LIB_NAMES
    pcl_common pcl_octree pcl_search pcl_kdtree pcl_sample_consensus
    pcl_filters pcl_io pcl_io_ply lz4 flann_cpp boost_filesystem)
set(RKV_PCL_LIBS "")
set(RKV_PCL_FOUND TRUE)
foreach(_rkv_pcl_lib IN LISTS RKV_PCL_LIB_NAMES)
    find_library(RKV_PCL_LIB_${_rkv_pcl_lib} NAMES ${_rkv_pcl_lib}
        PATHS "${ROOT_DEVEL_ARCH_PATH}/lib" "${CONTROL_DEVEL_ARCH_PATH}/lib"
        NO_DEFAULT_PATH)
    if(NOT RKV_PCL_LIB_${_rkv_pcl_lib})
        set(RKV_PCL_FOUND FALSE)
        message(STATUS "PCL: missing lib ${_rkv_pcl_lib}")
        break()
    endif()
    list(APPEND RKV_PCL_LIBS ${RKV_PCL_LIB_${_rkv_pcl_lib}})
endforeach()

# ---------------------------------------------------------------------------
# rkv_viewer_core — all reusable viewer logic as a shared lib.
# Panel plugins and the main executable both link against it.
# ---------------------------------------------------------------------------
add_library(rkv_viewer_core SHARED
    src/kinematic_viewer/kinematic_ik_controller.cpp
    src/kinematic_viewer/kinematic_string_utils.cpp
    src/kinematic_viewer/kinematic_viewer_config.cpp
    src/kinematic_viewer/kinematic_render_loop.cpp
    src/kinematic_viewer/kinematic_input_handler.cpp
    src/kinematic_viewer/kinematic_config_watcher.cpp
    src/kinematic_viewer/kinematic_playback_state_machine.cpp
    src/kinematic_viewer/kinematic_playback.cpp
    src/kinematic_viewer/kinematic_playback_bundle.cpp
    src/kinematic_viewer/kinematic_collision_monitor.cpp
    src/kinematic_viewer/kinematic_collision_distance.cpp
    src/kinematic_viewer/kinematic_initial_pose.cpp
    src/kinematic_viewer/kinematic_marker_target_state.cpp
    src/kinematic_viewer/kinematic_sidebar_panels.cpp
    src/kinematic_viewer/kinematic_ui_theme.cpp
    src/kinematic_viewer/kinematic_ui_feedback.cpp
    src/kinematic_viewer/kinematic_demo_visual.cpp
    src/kinematic_viewer/kinematic_viewport_hud.cpp
    src/kinematic_viewer/kinematic_marker_utils.cpp
    src/kinematic_viewer/kinematic_shader_utils.cpp
    src/kinematic_viewer/kinematic_line_renderer.cpp
    src/kinematic_viewer/kinematic_user_obstacles.cpp
    src/kinematic_viewer/kinematic_path_planner.cpp
    src/kinematic_viewer/kinematic_link_kinematics.cpp
    src/kinematic_viewer/kinematic_link_inspector.cpp
    src/kinematic_viewer/kinematic_robot_tree_panel.cpp
    src/kinematic_viewer/kinematic_video_panel.cpp
    src/kinematic_viewer/kinematic_video_recorder.cpp
    src/kinematic_viewer/kinematic_teach.cpp
    src/kinematic_viewer/kinematic_teach_panel.cpp
    src/kinematic_viewer/kinematic_point_cloud.cpp
    src/kinematic_viewer/kinematic_viewer_session.cpp
    src/kinematic_viewer/rkv_panel_registry.cpp
    src/rkv/ik_solver.cpp
    src/rkv/scene.cpp
)
target_include_directories(rkv_viewer_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/flex_ik"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ik_backends/wbc_chain_ik"
    ${trac_ik_INCLUDE_DIR}
    ${GIF_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIR}
)
target_link_libraries(rkv_viewer_core PUBLIC
    imgui_glfw_support
    imguizmo
    assimp
    urdfdom_model
    urdfdom_world
    urdfdom_sensor
    ${trac_ik_LIBRARIES}
    flex_ik_core
    wbc_chain_ik_core
    pinocchio::pinocchio
    velocity_planning
    OpenGL::GL
    glfw
    GLEW::GLEW
    yaml-cpp
    PkgConfig::LIBAV
    ${GIF_LIBRARY}
    pthread
    dl
    glog::glog
    ${CMAKE_DL_LIBS}
)
set_target_properties(rkv_viewer_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
    BUILD_RPATH "${ROOT_DEVEL_ARCH_PATH}/lib"
    INSTALL_RPATH "$ORIGIN")

if(qpOASES_FOUND)
    target_link_libraries(rkv_viewer_core PUBLIC ${qpOASES_LIBRARIES})
elseif(QPOASES_LIBRARY)
    target_link_libraries(rkv_viewer_core PUBLIC ${QPOASES_LIBRARY})
endif()

# ---------------------------------------------------------------------------
# Helper macro: one panel .so = rkv_panel_<id>
# ---------------------------------------------------------------------------
macro(rkv_add_panel id src)
    add_library(rkv_panel_${id} SHARED ${src})
    target_link_libraries(rkv_panel_${id} PRIVATE rkv_viewer_core)
    set_target_properties(rkv_panel_${id} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
        BUILD_RPATH "${ROOT_DEVEL_ARCH_PATH}/lib"
        INSTALL_RPATH "$ORIGIN")
endmacro()

set(RKV_PANEL_TARGETS "")
rkv_add_panel(scene       src/panels/panel_scene.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_scene)
rkv_add_panel(ik          src/panels/panel_ik.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_ik)
rkv_add_panel(playback    src/panels/panel_playback.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_playback)
rkv_add_panel(safety      src/panels/panel_safety.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_safety)
rkv_add_panel(joint       src/panels/panel_joint.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_joint)
rkv_add_panel(tf          src/panels/panel_tf.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_tf)
rkv_add_panel(obstacle    src/panels/panel_obstacle.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_obstacle)
rkv_add_panel(planner     src/panels/panel_planner.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_planner)
rkv_add_panel(teach       src/panels/panel_teach.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_teach)
rkv_add_panel(point_cloud src/panels/panel_point_cloud.cpp)
list(APPEND RKV_PANEL_TARGETS rkv_panel_point_cloud)

# ---------------------------------------------------------------------------
# Main executable — only app glue + glad; everything else is in rkv_viewer_core
# ---------------------------------------------------------------------------
add_executable(robot_kinematic_viewer
    src/robot_kinematic_viewer.cpp
    src/kinematic_viewer/kinematic_app.cpp
    src/kinematic_viewer/kinematic_bootstrap.cpp
    deps/glad/glad.c
)

target_link_libraries(robot_kinematic_viewer PRIVATE rkv_viewer_core)
add_dependencies(robot_kinematic_viewer ${RKV_PANEL_TARGETS})
set_target_properties(robot_kinematic_viewer PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib;${ROOT_DEVEL_ARCH_PATH}/lib"
    INSTALL_RPATH "$ORIGIN/../lib")

add_executable(collision_distance_test tests/collision_distance_test.cpp deps/glad/glad.c)
target_link_libraries(collision_distance_test PRIVATE rkv_viewer_core glfw)
set_target_properties(collision_distance_test PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/lib;${ROOT_DEVEL_ARCH_PATH}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

if(RKV_PCL_FOUND)
    find_library(RKV_LIB_GOMP NAMES gomp libgomp PATHS "${ROOT_DEVEL_ARCH_PATH}/lib" NO_DEFAULT_PATH)
    if(RKV_LIB_GOMP)
        list(APPEND RKV_PCL_LIBS ${RKV_LIB_GOMP})
    endif()

    add_library(rkv_point_cloud_pcl SHARED
        src/kinematic_viewer/kinematic_point_cloud_pcl.cpp
        src/kinematic_viewer/rkv_esdf_grid.cpp
        src/kinematic_viewer/rkv_fiesta_esdf.cpp
        src/kinematic_viewer/rkv_raycast.cpp
    )
    target_compile_definitions(rkv_point_cloud_pcl PRIVATE RKV_PCL_LOADER_EXPORTS=1)
    target_include_directories(rkv_point_cloud_pcl PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ROOT_DEVEL_ARCH_PATH}/include"
        "${ROOT_DEVEL_ARCH_PATH}/include/eigen3")
    target_link_libraries(rkv_point_cloud_pcl PRIVATE ${RKV_PCL_LIBS})
    set_target_properties(rkv_point_cloud_pcl PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
        BUILD_RPATH "${ROOT_DEVEL_ARCH_PATH}/lib"
        INSTALL_RPATH "$ORIGIN/../lib")

    target_compile_definitions(rkv_viewer_core PUBLIC RKV_HAS_PCL=1)
    target_compile_definitions(robot_kinematic_viewer PRIVATE RKV_HAS_PCL=1)
    add_dependencies(robot_kinematic_viewer rkv_point_cloud_pcl)
    message(STATUS "Point cloud: PCL plugin rkv_point_cloud_pcl (${RKV_PCL_LIBS})")
else()
    message(WARNING "PCL libraries not found under ${ROOT_DEVEL_ARCH_PATH}/lib — point cloud loading disabled")
endif()

file(COPY config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
