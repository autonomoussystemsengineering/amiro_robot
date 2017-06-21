# AMiRo robot model for Gazebo ROS

* Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
* License: GNU General Public License, version 3 (GPL-3.0)

The AMiRo robot model (tested with Ubuntu 16.04, ROS Kinetic, Gazebo 7.8)

## Quick Start

Rviz (untested):

    roslaunch amiro_description amiro_rviz.launch

Gazebo:

    roslaunch amiro_gazebo amiro_world.launch

ROS Control (NOT READY YET):

    roslaunch amiro_control amiro_control.launch

Example of Moving the Robot:

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

## Courtesy

* [General introduction](https://github.com/ros-simulation/gazebo_ros_demos)
* [Meshes and scripts for RGBD camera](https://github.com/turtlebot/turtlebot)
* [Meshes and scripts for Hokuyo URG04 laser and generic camera](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/jade-devel/gazebo_plugins/test/multi_robot_scenario)

## Debugging and Hints

### Generate SDF from XACRO

First generate an URDF file and then build the SDF from that:

    rosrun xacro xacro --inorder -o model.urdf model.xacro
    gz sdf -p model.urdf > model.sdf

You might be intrested, if the model was loaded in Gazebo with its proper paramters:

    gz model -m <Your model name e.g. amiro1> -i
    
### The Caster Dilemma or why Surfaces do not Slide

Adding a caster might be intuitive on the first sight, but becomes actually a pain for some reasons:
First, one is always eager to minimize the sum of joints to keep the model simple.
Thus, it is a wise idea to add a rigid sphere to the main body as proposed in [1](http://answers.gazebosim.org/question/5371/differential-drive-caster-wheel-problem/) and [2](http://gazebosim.org/tutorials?tut=build_robot).
Second, after that one has to set the friction and sliding parameters of the ODE physics simulator so that the sphere acts as an ideal caster.
This is actually the official way to go because it is even explained so in the official documentation [2](http://gazebosim.org/tutorials?tut=build_robot), [3](http://gazebosim.org/tutorials/?tut=ros_urdf).
Unfortunately, the model which is designed in such a way don't behave well.
In fact, the caster is more a bumper wich bumpes into the ground and stops the vehicle which can result in numerical problems while using some controllers.
Actualy, if one digs deeper you'll find out that the friction parameters cannot not be set at all [4](http://answers.gazebosim.org/question/12611/urdf-to-gazebo-differs-from-urdf-to-sdf-to-gazebo/), [5](http://answers.gazebosim.org/question/7074/urdf-to-sdf-conversion-using-gzsdf/), [6](http://answers.gazebosim.org/question/7082/using-sdf-tags-in-urdf-gazebo-extension-directly/).
There is only a minimal hint in one answer where one says that friction for ODE is not implemented yet [7](http://answers.gazebosim.org/question/7074/urdf-to-sdf-conversion-using-gzsdf/?answer=7079#post-id-7079).
One solution to this dilemma is to use a sphere with a revolutional joint as caster to get rid of the bumpy behaviour.


### Install the current Gazebo7 simulator

Assuming that you have installed ROS Kinetic on Ubuntu 16.04, you can go to the [Gazebo install page](http://gazebosim.org/tutorials?tut=install_ubuntu) and follow the step-by-step instruction. Do not install gazebo8, but upgrade gazebo7 via:

    sudo apt-get install gazebo7