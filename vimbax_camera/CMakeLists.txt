cmake_minimum_required(VERSION 3.8)
project(vimbax_camera)

set(VMB_DIR "" CACHE STRING "Path to VimbaX installation")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(vimbax_camera_node_SRCS
        src/vimbax_camera_node.cpp
        src/loader/library_loader_unix.cpp
        src/loader/vmbc_api.cpp
        src/vimbax_camera.cpp
        src/vimbax_camera_helper.cpp
)

# Add recorder node source
set(stereo_recorder_SRCS
        src/stereo_recorder_node.cpp)

# Add simple node recorder source
set(simple_node_recorder_SRCS
        src/simple_node_recorder.cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(vimbax_camera_msgs REQUIRED)
find_package(vimbax_camera_events REQUIRED)
find_package(vmbc_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)

add_library(${PROJECT_NAME} SHARED ${vimbax_camera_node_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "rclcpp_components"
        "sensor_msgs"
        "image_transport"
        "camera_info_manager"
        "vimbax_camera_msgs"
        "vimbax_camera_events"
        "vmbc_interface"
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "vimbax_camera::VimbaXCameraNode"
    EXECUTABLE vimbax_camera_node
    EXECUTOR MultiThreadedExecutor
)

# Add the recorder node as a shared library
add_library(stereo_recorder_node SHARED ${stereo_recorder_SRCS})
target_include_directories(stereo_recorder_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_recorder_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
        stereo_recorder_node
        "rclcpp"
        "rclcpp_components"
        "sensor_msgs"
        "std_srvs"
        "builtin_interfaces"
        "rosbag2_cpp"
        "rosbag2_storage_default_plugins"
)

# Register the recorder node with rclcpp_components
rclcpp_components_register_node(stereo_recorder_node
    PLUGIN "stereo_recorder::StereoRecorder"
    EXECUTABLE stereo_recorder
)

# Add the simple node recorder as a standalone executable
add_executable(simple_node_recorder ${simple_node_recorder_SRCS})
ament_target_dependencies(
    simple_node_recorder
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_srvs"
    "rosbag2_cpp"
    "rosbag2_storage_default_plugins"
)
install(TARGETS simple_node_recorder
        DESTINATION lib/${PROJECT_NAME})

# Install the shared libraries
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS stereo_recorder_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Add stereo_trig_node executable
add_executable(stereo_trig_node src/stereo_trig_node.cpp)
ament_target_dependencies(stereo_trig_node rclcpp sensor_msgs builtin_interfaces vimbax_camera_msgs)
install(TARGETS
        stereo_trig_node
        DESTINATION lib/${PROJECT_NAME})        

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install parameter files.
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_package()
