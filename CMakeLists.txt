cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz_wgpu_rt_lidar)

find_package(ament_cmake REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)

#============================================================================
# Corrosion
include(FetchContent)

FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.5 # Optionally specify a commit hash, version tag or branch here
)

FetchContent_MakeAvailable(Corrosion)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================
# Rust Business Logic
corrosion_import_crate(MANIFEST_PATH src/rust_system/Cargo.toml)

#============================================================================
# Wrapper stuff
add_library(wgpu_rt_sensor SHARED
  src/wgpu_sensor.cc
  src/rtsensor.cc
  src/rtmanager.cc)

target_link_libraries(wgpu_rt_sensor PUBLIC
        rust_system
        gz-plugin::gz-plugin
        gz-sim::gz-sim
        gz-common::graphics
        gz-math::gz-math
        gz-sensors::gz-sensors
)

install(TARGETS wgpu_rt_sensor DESTINATION lib)

#============================================================================
# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
