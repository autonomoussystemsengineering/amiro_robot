# Gazebo ROS Demos

* Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
* License: GNU General Public License, version 3 (GPL-3.0)

The AMiRo robot model

## Quick Start

Rviz:

    roslaunch amiro_description amiro_rviz.launch

Gazebo:

    roslaunch amiro_gazebo amiro_world.launch

ROS Control (NOT READY YET):

    roslaunch amiro_control amiro_control.launch

Example of Moving Joints of amiro1:

    rostopic pub -r 10 /amiro1/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

Example of teleoperation using `ros-kinetic-turtlebot-teleop`:

    rosrun turtlebot_teleop turtlebot_teleop_key _scale_linear:=0.1 _scale_angular:=0.2 turtlebot_teleop/cmd_vel:=amiro1/cmd_vel


## Reference

* [How to Build a Differential Drive Simulation](http://www.theconstructsim.com/how-to-build-a-differential-drive-simulation/)
* [SDF](http://sdformat.org/spec?ver=1.6)
* [Using Gazebo plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
* [Using a URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)
* [xacro](http://wiki.ros.org/xacro)
* [Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/action/fullsearch/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)
* [gazebo_ros_diff_drive.cpp](http://docs.ros.org/kinetic/api/gazebo_plugins/html/gazebo__ros__diff__drive_8cpp_source.html)
* [How do you run multiple hector_quadrotors without separate urdf files?](http://answers.gazebosim.org/question/4190/how-do-you-run-multiple-hector_quadrotors-without-separate-urdf-files/)
* [Simulating Small Differential Drive Robot in Gazebo](http://answers.ros.org/question/47612/simulating-small-differential-drive-robot-in-gazebo/)