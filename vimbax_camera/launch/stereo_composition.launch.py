import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Get the directory of the vimbax_camera package
vimbax_camera_share_dir = get_package_share_directory('vimbax_camera')

# Construct the absolute path to the settings files
right_settings_file_path = os.path.join(vimbax_camera_share_dir, 'config', 'RIGHT_ZJ_PWM_TRIGED_BIN.xml')
left_settings_file_path = os.path.join(vimbax_camera_share_dir, 'config', 'LEFT_ZH_PWM_TRIGED_BIN.xml')

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='vimbax_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Use multithreaded container
            composable_node_descriptions=[
                ComposableNode(
                    package='vimbax_camera',
                    plugin='vimbax_camera::VimbaXCameraNode',
                    namespace='vimbax_camera_right',
                    name='vimbax_camera_right',
                    parameters=[{
                        "camera_id": "DEV_00012C050ADF",
                        "settings_file": right_settings_file_path,
                        "use_ros_time": False,
                        "autostream": 1,
                        "buffer_count": 30
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='vimbax_camera',
                    plugin='vimbax_camera::VimbaXCameraNode',
                    namespace='vimbax_camera_left',
                    name='vimbax_camera_left',
                    parameters=[{
                        "camera_id": "DEV_00012C050ADD",
                        "settings_file": left_settings_file_path,
                        "use_ros_time": False,
                        "autostream": 1,
                        "buffer_count": 30
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='rosbag2_transport',
                    plugin='rosbag2_transport::Recorder',
                    name='rosbag2_composable_recorder',
                    parameters=[{
                        # 'topics': [
                        #     '/vimbax_camera_right/image_raw',
                        #     '/vimbax_camera_left/image_raw'
                        # ],
                        'output': '/ssd1/bag_db',  # Specify the directory for the bag files
                        'record_all': True,
                        'record_qos_overrides': {}
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
            emulate_tty=True,
            log_cmd=True,
            additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': 'DEBUG'}
        )
    ])

# Function to check if the settings files are accessible
def check_settings_file(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Settings file not found: {path}")
    if not os.access(path, os.R_OK):
        raise PermissionError(f"Settings file is not readable: {path}")

# Check the settings files
try:
    check_settings_file(right_settings_file_path)
    check_settings_file(left_settings_file_path)
except (FileNotFoundError, PermissionError) as e:
    print(e)
