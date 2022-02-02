'''
Author:     Houssem JEBARI
----------------------------------------------------------------------
Institute:  National Institute of Applied Science and Technology, Tunis
-----------------------------------------------------------------------
Description: This file contains the decision making algorithm for the NASA,
Rover simulator. Indeed, the decision making step is based on a decision tree
that takes into account the robot and the environment specificities.
'''

import numpy as np 

# Parameters for Decision Making
STOP_THRESHOLD = 0.2

# Functions representing the actions of the decision tree
def full_throttle(Rover):
    '''
    This function applies full throttle to the rover 
    args:
        - Rover: RoverState object, holding the state of the rover
    '''
    Rover.throttle = Rover.throttle_set
    Rover.brake = 0
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

def coast(Rover):
    '''
    This function sets the throttle of the rover to zero  
    args:
        - Rover: RoverState object, holding the state of the rover
    '''
    Rover.throttle = 0.
    Rover.brake = 0.
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

def brake(Rover):
    '''
    This function brakes the rover.
    args:
        - Rover: RoverState object, holding the state of the rover
    '''
    Rover.throttle = 0.
    Rover.brake = Rover.brake_set

def turn(Rover):
    '''
    This function makes the rover turn when stopped to seek for new free space.
    args:
        - Rover: RoverState object, holding the state of the rover
    '''
    Rover.throttle = 0.
    Rover.brake = 0.
    Rover.steer = -15. 

# Define the decision tree implementation function
def decision_step(Rover):
    '''
    This function implements the decision step that enables the rover 
    to navigate autonomously in MARS !
    
    args:
        - Rover: RoverState object, holding the state of the rover
    '''

    if Rover.nav_angles is not None:
        if len(Rover.nav_angles) >= Rover.stop_forward:
            if Rover.vel < Rover.max_vel:
                full_throttle(Rover)
            else:
                coast(Rover)
        else:
            if Rover.vel > STOP_THRESHOLD:
                brake(Rover)
            else:
                turn(Rover)
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    return Rover