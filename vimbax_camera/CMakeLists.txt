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
        src/camera_interface.cpp
)

# find dependencies
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

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Add stereo_trig_node executable
add_executable(stereo_trig_node src/stereo_trig_node.cpp)
ament_target_dependencies(stereo_trig_node rclcpp sensor_msgs builtin_interfaces vimbax_camera_msgs)
install(TARGETS
        stereo_trig_node
        DESTINATION lib/${PROJECT_NAME}) 
        
# Add timestamp_sync_node executable
add_executable(timestamp_sync_node src/timestamp_sync_node.cpp)
ament_target_dependencies(timestamp_sync_node
  rclcpp
  std_msgs
  sensor_msgs
  vimbax_camera_msgs
)

install(TARGETS
        timestamp_sync_node
        DESTINATION lib/${PROJECT_NAME}
)

# Add camera_interface_test_node executable
add_executable(camera_interface_test_node src/camera_interface_test_node.cpp)
ament_target_dependencies(camera_interface_test_node
  rclcpp
  sensor_msgs
)
target_link_libraries(camera_interface_test_node ${PROJECT_NAME})
install(TARGETS
        camera_interface_test_node
        DESTINATION lib/${PROJECT_NAME}
)     

# Add sensors_sync_node executable
add_executable(sensors_sync_node src/sensors_sync_node.cpp)
ament_target_dependencies(sensors_sync_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  camera_info_manager
  vimbax_camera_msgs
)

target_link_libraries(sensors_sync_node ${PROJECT_NAME})
install(TARGETS
  sensors_sync_node
  DESTINATION lib/${PROJECT_NAME}
)


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
