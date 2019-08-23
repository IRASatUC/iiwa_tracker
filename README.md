# robot_tracker

robot tracker uses [KUKA iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) equiped with a [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) camera to track the mobile robot on the ground. The goal is to keep the mobile robot in the center of the view. The [Vicon](https://www.vicon.com/), a motion capture system, will be leveraged for calculating the accurate position of the mobile robot.  

## Pre-requisites
1. Vicon host machine
2. ROS host machine ([ROS Ubuntu Kinetic](https://note.youdao.com/) is installed) 
3. mobile robot ([zumopi](https://github.com/linZHank/zumo_pi)), and the model in Vicon Tracker
4. KUKA iiwa with ROS control enabled, please refer to my repo [robotic_scan](https://github.com/suneric/robotic_scan)
5. [Intel Realsense SDK](https://github.com/IntelRealSense/librealsense
) is installed on you ROS machine and [ros warpper realsense](https://github.com/IntelRealSense/realsense-ros) is installed.

## Setup
1. Network:
    - Vicon host: 192.168.1.111
    - KUKA iiwa: 192.168.1.161
    - ROS machine: 192.168.1.171
2. Active Vicon Tracker and zumopi, a model is avaialable in Vicon Tracker  
3. Active the ROSSmartServo on KUKA iiwa teach pendant


## Install
1. Install vicon_bridge for position information query on your ROS machine
    ```
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    git clone https://github.com/ethz-asl/vicon_bridge.git
    cd ..
    catkin build
    ```
2. Clone this repo and start tracking
    ```
    cd ~/ros_ws/src
    git clone https://github.com/suneric/robot_tracker.git
    cd ..
    catkin build
    ```

## Play
1. start vicon ```roslaunch vicon_bridge vicon.launch```
2. start robot tracker ```roslaunch robot_tracker iiwa.launch```
3. start realsense2_camera ```roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ```
4. move the zumopi, the robot will track the position of the zumopi.
