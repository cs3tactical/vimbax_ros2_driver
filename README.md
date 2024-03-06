# Vimba X ROS 2 camera driver

## Prerequisites
- ROS 2 humble is installed on the system as defined by the [ROS 2 installation instructions](https://docs.ros.org/en/humble/Installation.html)
- For running the system tests make sure the package "ros-humble-launch-pytests" is installed on your system.

## Build Instructions
1. Setup the ROS2 environment 
    ```shell
    source /opt/ros/humble/setup.bash 
    ```
   
2. Create a workspace
    ```shell
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
   
3. Clone this repository into the workspace
    ```shell
    git clone ...
    ``` 
   
4. Run the actual build 
    ```shell
    cd ~/ros2_ws/
    colcon build --cmake-args -DVMB_DIR=<path to VimbaX installation>
    ```  
    The optional VMB_DIR cmake argument can be used to specify the path to the Vimba X installation 
    that used for testing.

5. Run unit tests (optional)
    ```shell
    colcon test
    ```  

6. Print test results (optional)
    ```shell
    colcon test-result --verbose
    ```  

## Debug Instructions (VSCode)
* Install ROS extension (from Microsoft) within VSCode
* Type CTRL-SHIFT-p to open the command palette
* Search and execute "ROS: Start"
* Search and execute "ROS: Update C++ properties"
* Type CTRL-SHIFT-b and select "colcon" to build the project
* Type CTRL-SHIFT-d and select "ROS: Lauch" to start debug session
