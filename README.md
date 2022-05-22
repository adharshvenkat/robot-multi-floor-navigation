# robot-multi-floor-navigation

Install

Get dependencies

Jackal Gazebo Dependencies:

For ROS melodic, run:

$ sudo apt-get install ros-melodic-jackal-*

Install package

    Create a ROS workspace, for e.g. catkin_ws
    Place this package inside the src folder of catkin_ws
    In catkin_ws, issue catkin make command
    Source the workspace source devel/setup.bash

Run a sample simulation

To launch the simulation, use the following roslaunch command:

$ roslaunch jackal_elevator jackal_elevator_nav.launch 

To launch the multi floor navigation, use the following rosrun command;
$ rosrun jackal_elevator multi_floor_nav 

