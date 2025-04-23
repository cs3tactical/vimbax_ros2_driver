import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def get_camera_name(calib_file_path):
    try:
        with open(calib_file_path, 'r') as file:
            lines = file.readlines()
            if lines[0].strip().startswith("%YAML:1.0"):
                lines = lines[1:]
            calib_data = yaml.safe_load("\n".join(lines))
            return calib_data.get("camera_name", "UNKNOWN_CAMERA")
    except Exception as e:
        print(f"Error reading {calib_file_path}: {e}")
        return "UNKNOWN_CAMERA"

def generate_launch_description():
    home = os.environ.get("HOME", "/home/user")

    left_yaml = os.path.join(home, "ssd1", "local_configs", "sensors_calib", "left_cam", "camera.yaml")
    right_yaml = os.path.join(home, "ssd1", "local_configs", "sensors_calib", "right_cam", "camera.yaml")

    left_name = get_camera_name(left_yaml)
    right_name = get_camera_name(right_yaml)

    return LaunchDescription([
        Node(
            package='vimbax_camera',
            executable='sensors_sync_node',
            name='sensors_sync_node',
            namespace='vimbax_camera',
            output='screen',
            parameters=[{
                "imu_topic": "/imu/data",
                "pwm_frequency": 105,
                "pwm_divider": 7,
                "pwm_duty_cycle": 50,
                "buffer_count": 15,
                "left_camera_id": left_name,
                "right_camera_id": right_name,
                "left_camera_info_url": f"file://{left_yaml}",
                "right_camera_info_url": f"file://{right_yaml}"
            }]
        )
    ])
