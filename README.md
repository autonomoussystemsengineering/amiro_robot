# AMiRo robot model for Gazebo ROS

* Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
* License: GNU General Public License, version 3 (GPL-3.0)

The AMiRo robot model

## Quick Start

Rviz (untested):

    roslaunch amiro_description amiro_rviz.launch

Gazebo:

    roslaunch amiro_gazebo amiro_world.launch

ROS Control (untested):

    roslaunch amiro_control amiro_control.launch

Example of Moving the Robot:

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
