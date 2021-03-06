import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    channel1_hist = np.histogram(channel_1_vals,bins=32,range=(0,256))
    channel2_hist = np.histogram(channel_2_vals,bins=32,range=(0,256))
    channel3_hist = np.histogram(channel_3_vals,bins=32,range=(0,256))
    # TODO: Concatenate and normalize the histograms
    feature_hist = np.float32(np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0])))
    s = sum(feature_hist)
    normed_hist = feature_hist / np.sum(feature_hist)
    #print('normal1: ', feature_hist[0] / s)
    #print('normal histogram: ', normed_hist)
    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96) 
    return normed_hist 


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    channel1_hist = np.histogram(norm_x_vals,bins=32,range=(min(norm_x_vals),max(norm_x_vals)))
    channel2_hist = np.histogram(norm_y_vals,bins=32,range=(min(norm_y_vals),max(norm_y_vals)))
    channel3_hist = np.histogram(norm_y_vals,bins=32,range=(min(norm_z_vals),max(norm_z_vals)))
    # TODO: Concatenate and normalize the histograms
    features = np.float32(np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0])))
    normed_features = features / np.sum(features)
   # print(normed_features)
    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96)

    return normed_features
