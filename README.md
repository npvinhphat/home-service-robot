# Home Service Robot

This is the fifth and final project for Udacity's [Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209), namely *Home Service Robot*.

## How it works

This repository contains a full-service robot, which can be localized and navigated through a predefined environment, containing the following packages:
* **pick_objects**: This is for the robot to navigate through an environment, with a pickup and dropoff position
* **add_markers**: This is for demonstrating the pickup and dropoff of the above package

This repository also houses several dependencies:
* **gmapping**: With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
* **turtlebot_teleop**: With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
* **turtlebot_rviz_launchers**: With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
* **turtlebot_gazebo**: With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

## Installation

Make sure that you have [installed Catkin](http://www.ros.org/wiki/catkin#Installing_catkin) and sourced your environment. More info about creating your catkin workspace [here](http://www.ros.org/wiki/catkin#Installing_catkin).

Use the following commands to build the modules:

```
cd ~/catkin_ws/src
git pull git@github.com:npvinhphat/home-service-robot.git .
cd ..
catkin_make
```

## Usage

There are several scripts in this respository, which performs different task. To run a script, use the following command:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
./src/scripts/<script-name>
```

### test_slam.sh

Deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz.

### test_navigation.sh

Using the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan the robot trajectory from start to goal position. The ROS navigation stack permits the robot to avoid any obstacle on its path by re-planning a new trajectory once the robot encounters them.

### pick_objects.sh

Script to demonstrate robot automation: the robot has to travel to the desired pickup zone, display a message that it reached its destination, wait 5 seconds, travel to the desired drop off zone, and display a message that it reached the drop off zone.

### add_markers.sh

Script to model a virtual object with markers in rviz. The virtual object is the one being picked and delivered by the robot, thus it should first appear in its pickup zone, and then in its drop off zone once the robot reaches it.

### home_service.sh

The final script, which do both pick_objects and add_markers