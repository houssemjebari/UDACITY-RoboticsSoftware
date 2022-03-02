[//]: # (Image References)
[image_0]: ./misc_images/misc2.png
![alt text][image_0] 

# Robotic arm - Pick & Place project

This project is modeled after the Amazon Robotic Chanllenge consists of a robotic arm that should be programmed to pick objects from a shelf and place them on a bin in a different location.

The Amazon Challenge involves many disciplines like: perception in a clutter environment, safe and versatile grippers, motion planing and kinematics. For this Udacity project we will be focusing on the last one: kinematics.

## 1 - Dependencies
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0, ROS Kinetic and Ubuntu 16.04 to work 


## 2 - Kinematics
In this project the main topic solved is the kinematics for the KUKA-RP210 robot. Forward and Inverse Kinematics are the two main problems for serial manipulators and are hard to solve in general.

[image_1]: ./misc_images/misc5.png

![alt text][image_1] 

### Forward Kinematics 
To solve the forward kinematics problem for this robot the Denavit-Hartenburg parameters need to be determined first. In order to do so we need first to determine the robots links reference frames as shown in the figure below. Secondly, we can determine the parameters using the robot URDF file.

[image_2]: ./misc_images/misc4.png
![alt text][image_2] 

### Inverse Kinematics 
We developed closed form solutions for the inverse kinematics for this project by decoupling the equations into position and orientation equations making solving for the joint values alot easier. In fact, this decoupling is possible because the robot wrist uses a spherical joint.
Solving for the orientation joint angles is straight forward. For the position joint angles we rely on geometry and try to solve for the angles using this figure.

[image_3]: ./misc_images/misc3.png
![alt text][image_3] 

## 3 - Results

[image_4]: ./misc_images/pick-place.gif
![alt text][image_4] 


## 4 - Launching the project
You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```


