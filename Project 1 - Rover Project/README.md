[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
# Search and Sample Return Project


![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) This project is carried in a simulator environment built with the Unity game engine.  

## The Simulator
The first step is to download the simulator build that's appropriate for your operating system.  Here are the links for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).  

You can test out the simulator by opening it up and choosing "Training Mode".  Use the mouse or keyboard to navigate around the environment and see how it looks.

## Recording Data
I've saved some test data for you in the folder called `test_dataset`.  In that folder you'll find a csv file with the output data for steering, throttle position etc. and the pathnames to the images recorded in each run.  I've also saved a few images in the folder called `calibration_images` to do some of the initial calibration steps with.  

The first step of this project is to record data on your own.  To do this, you should first create a new folder to store the image data in.  Then launch the simulator and choose "Training Mode" then hit "r".  Navigate to the directory you want to store data in, select it, and then drive around collecting data.  Hit "r" again to stop data collection.

## Navigating Autonomously
The navigation stack is composed of perception and decision making which are implemented within `perception.py` and `decision.py` files.  `drive_rover.py` is what you will use to navigate the environment in autonomous mode.  This script calls functions from within the other files and sends them directly to the simulator.

## Perception
The perception pipeline takes camera images and calibrates them to obtain a map of the environment in the world coordinates. The results of this perception pipeline are shown in the figure below.

[image_1]: ./output/perception_pipeline.png

![alt text][image_1] 

## Decision making
The decision making relies on a finite state machine and a decision tree models to send control commands to the robot based on the perception data. the models for decision making are show below 

[image_2]: ./output/decision_tree.PNG

![alt text][image_2] 

```sh
python drive_rover.py
```  



