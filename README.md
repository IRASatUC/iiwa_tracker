# Getting Started

This repo uses a [KUKA iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) robot manipulting objects tracked by a [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) camera.  

## Pre-requisites
1. [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) or [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
2. [ROS-Melodic](http://wiki.ros.org/melodic) for Ubuntu 18.04 or [ROS-Kinetic](http://wiki.ros.org/kinetic) for Ubuntu 16.04
3. [iiwa_stack](https://github.com/IRASatUC/iiwa_stack)
4. [Intel Realsense SDK](https://github.com/IntelRealSense/librealsense)
5. [pysot](https://github.com/STVIR/pysot)

## Setup
1. Network:
    - ROS machine: 192.168.1.100
    - KUKA iiwa: 192.168.1.160 (>another iiwa's ip is 192.168.1.161)
2. Launch `roscore`
3. Start **ROSSmartServo** application on Kuka's smartPAD
4. run `rostopic list` to verify everything looks similar to the following:
```console
/iiwa/action/move_along_spline/cancel
/iiwa/action/move_along_spline/feedback
/iiwa/action/move_along_spline/goal
/iiwa/action/move_along_spline/result
/iiwa/action/move_along_spline/status
/iiwa/action/move_to_cartesian_pose/cancel
/iiwa/action/move_to_cartesian_pose/feedback
/iiwa/action/move_to_cartesian_pose/goal
/iiwa/action/move_to_cartesian_pose/result
/iiwa/action/move_to_cartesian_pose/status
/iiwa/action/move_to_cartesian_pose_lin/cancel
/iiwa/action/move_to_cartesian_pose_lin/feedback
/iiwa/action/move_to_cartesian_pose_lin/goal
/iiwa/action/move_to_cartesian_pose_lin/result
/iiwa/action/move_to_cartesian_pose_lin/status
/iiwa/action/move_to_joint_position/cancel
/iiwa/action/move_to_joint_position/feedback
/iiwa/action/move_to_joint_position/goal
/iiwa/action/move_to_joint_position/result
/iiwa/action/move_to_joint_position/status
/iiwa/command/CartesianPose
/iiwa/command/CartesianPoseLin
/iiwa/command/CartesianVelocity
/iiwa/command/JointPosition
/iiwa/command/JointPositionVelocity
/iiwa/command/JointVelocity
/iiwa/joint_states
/iiwa/state/CartesianPose
/iiwa/state/CartesianWrench
/iiwa/state/DestinationReached
/iiwa/state/ExternalJointTorque
/iiwa/state/JointPosition
/iiwa/state/JointPositionVelocity
/iiwa/state/JointTorque
/iiwa/state/JointVelocity
/iiwa/state/buttonEvent
/rosout
/rosout_agg
/tf
/tf_static
```

## Example
Track a customer selected object using *siammask* algorithm and convert its 3d position into *iiwa*'s base link: `iiwa_link_0`
1. Launch roscore first,then in another terminal, launch gazebo simulation with *realsense D435*'s transform broadcaster.
```console
roslaunch iiwa_gazebo iiwa_gazebo_with_sunrise.launch
```
2. In a new terminal
```console
rosrun iiwa_tracker poker.py
```
> make sure the script is executable by running `chmod +x $iiwa_tracker/scripts/siammask_dipper.py`


## Contributors
| Developer      | E-mail | Location
| ----------- | ----------- | ----------------------------- |
|Yufeng Sun | sunyf@mail.uc.edu | IRAS Lab @ University of Cincinnati |
|Lin Zhang | lin.zhang@uc.edu | IRAS Lab @ University of Cincinnati |
