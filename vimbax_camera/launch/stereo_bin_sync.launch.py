# Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Get the directory of the vimbax_camera package
vimbax_camera_share_dir = get_package_share_directory('vimbax_camera')

# Construct the absolute path to the settings files
right_settings_file_path = os.path.join(vimbax_camera_share_dir, 'config', 'RIGHT_072ZH_TRIGD_BIN.xml')#'bw_trig.xml')
print("right_settings_file_path:", right_settings_file_path)
left_settings_file_path = os.path.join(vimbax_camera_share_dir, 'config', 'LEFT_072ZJ_TRIGS_BIN.xml')#'color_trig.xml')
print("left_settings_file_path", left_settings_file_path)

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='vimbax_camera',
            namespace='vimbax_camera_right',
            executable='vimbax_camera_node',
            name='vimbax_camera_right',
            parameters=[{
                "camera_id": "DEV_00012C050ADD",
                "settings_file": right_settings_file_path
            }],
            output='screen',
            emulate_tty=True,
            log_cmd=True,
            # additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': 'DEBUG'}
        ),
        Node(
            package='vimbax_camera',
            namespace='vimbax_camera_left',
            executable='vimbax_camera_node',
            name='vimbax_camera_left',
            parameters=[{
                "camera_id": "DEV_00012C050ADF",
                "settings_file": left_settings_file_path
            }],
            output='screen',
            emulate_tty=True,
            log_cmd=True,
            # additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': 'DEBUG'}
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

