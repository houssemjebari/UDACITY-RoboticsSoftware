'''
Author:     Houssem JEBARI
----------------------------------------------------------------------
Institute:  National Institute of Applied Science and Technology, Tunis
-----------------------------------------------------------------------
Description: This file contains the perception pipeline for the NASA,
Rover simulator. The pipeline is Used to map "Mars" planet environnement in
the simulator.

'''

import numpy as np
import cv2
from drive_rover import RoverState

def color_thresh(img, rgb_thresh=(190, 190, 190)):
    '''
    This function applies color threshold to the given image
    with the given threshold.

    args:
        - img: np.array object, containing np.uint8 numbers
        - rgb_thresh: tuple, contains thresholds for each color canal
    '''
    color_select = np.zeros_like(img[:,:,0])
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    color_select[above_thresh] = 1
    return color_select

def rover_coords(binary_img):
    '''
    Converts the image from image coordinates to the Rover coordinates
    args:
        - binary_img: np.array object, containing np.uint8 numbers
    '''
    ypos, xpos = binary_img.nonzero()
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

def to_polar_coords(x_pixel, y_pixel):
    '''
    This function transforms the given vectors X,Y vectors from radial 
    coordinates to polar coordinates in the Rover space

    args:
        - x_pixel: np.array vector
        - y_pixel: np.array vector
    '''
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

def rotate_pix(xpix, ypix, yaw):
    '''
    This function applies rotation of the given vectors given X,Y
    vectors with respect to the yaw angle.

    args:
        - x_pixel: np.array vector
        - y_pixel: np.array vector
        - yaw: float, yaw angle in degrees
    '''
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))                     
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    '''
    This function applies translation of the given vectors given X,Y
    vectors with respect to the Rover actual position.

    args:
        - x_pixel: np.array vector
        - y_pixel: np.array vector
        - xpos: float, Rover x position in world coordinates
        - ypos: float, Rover y position in world coordinates
        - scale: float, this is the same as the float used for calibration
    '''
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    return xpix_translated, ypix_translated

def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    '''
    This function applies rotation and translation (and clipping) 
    args:
        - xpix: np.array vector
        - ypix: np.array vector
        - yaw: float, yaw angle in degrees
        - xpos: float, Rover x position in world coordinates
        - ypos: float, Rover y position in world coordinates
        - scale: float, this is the same as the float used for calibration
    '''
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    return x_pix_world, y_pix_world

def perspect_transform(img, src, dst):
    '''
    This function applies a perspective transform 
    '''
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped

def perception_step(Rover):
    '''
    This function applies the Whole perception pipeline and updates the Rover state

    args:
        - Rover: RoverState object, holds all the communicated Rover data
    '''
    image = Rover.img # Get the actual rover image
    # Set source and destination points
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                              [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset]])
    # Apply the perception step to get the navigable terrain in world coordinates 
    # and polar robot coordinates 
    warped = perspect_transform(image, source, destination)
    threshed = color_thresh(warped)
    xpix,ypix = rover_coords(threshed)
    navigable_x_world,navigable_y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw,200,10)
    nav_dists,nav_angles = to_polar_coords(xpix, ypix)
    # Update the RoverState instance
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1    
    Rover.nav_dists = nav_dists
    Rover.nav_angles = nav_angles
    return Rover