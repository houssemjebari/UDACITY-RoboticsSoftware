
[image_0]: ./misc/output1.png
![alt text][image_0] 

# Where Am I ? 

In this project, we used ROS packages to accurately localize a mobile robot inside a provided map in the Gazebo and RViz simulation environments


## 1 - Dependencies

This project can be tested on any ROS version! just install `AMCL` and `MOVE_BASE` packages.
```
$ sudo apt-get install ros-noetic-navigation
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base
$ rospack profile
$ sudo apt-get install ros-noetic-amcl
```
## 2 - Localization algorithm

* The `AMCL` package uses particle filtering to localize the robot in a known map but in addition uses an adaptive number of particles which makes it more computationally efficient that classical algorithms.
the `MOVE_BASE` package contains global and local planners that enables the robot to navigate to any point in the map. 
* The spawned robot is created using URDF files which [documentation](http://wiki.ros.org/urdf) can be found in the link if you want to change to your own robot.
* The hyperparameters are tuned in the config folder and the launch files. 
If you want to further tune the localizer, refer to the [documentation](http://wiki.ros.org/move_base) to know which parameters to tune.


## 3 - Launching the project 

Clone this project into src folder in your ROS workspace then -

```
$ cd ~/catkin_ws
$ catkin_make 
$ devel/setup.bash
```
And finally run the following commands in separate terminals.

```
$ roslaunch udacity_bot udacity_bot
$ roslaunch udacity_bot amcl
```
