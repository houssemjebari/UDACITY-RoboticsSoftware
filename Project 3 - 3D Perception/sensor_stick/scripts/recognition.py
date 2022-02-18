#!/usr/bin/env python

import numpy as np
import pickle
import sklearn

from sklearn.preprocessing import LabelEncoder
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormal
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
	point_cloud = ros_to_pcl(pcl_msg)
    
	# Voxel Grid Downsampling
	vox = point_cloud.make_voxel_grid_filter()
	LEAF_SIZE = 0.04
	vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
	cloud_filtered = vox.filter()
    
	# PassThrough Filter
	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.76
	axis_max = 2.
	passthrough.set_filter_limits(axis_min,axis_max)
	cloud_filtered = passthrough.filter()
    
	# RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)
	inliers,coefficients = seg.segment()
    
	# Extract inliers and outliers
	table_cloud = cloud_filtered.extract(inliers,negative=False)
	objects_cloud = cloud_filtered.extract(inliers,negative=True)
    
	# Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(objects_cloud)
	tree = white_cloud.make_kdtree()
	ec = white_cloud.make_EuclideanClusterExtraction()
	ec.set_ClusterTolerance(0.07)
	ec.set_MinClusterSize(10)
	ec.set_MaxClusterSize(250)
	ec.set_SearchMethod(tree)
	cluster_indices = ec.Extract()
	cluster_color = get_color_list(len(cluster_indices))
    
	# Create Cluster-Mask Point Cloud 
	color_cluster_point_list = []
	for j,indices in enumerate(cluster_indices):
		for i,indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],rgb_to_float(cluster_color[j])])
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
    
	# Convert PCL data to ROS messages
	objects_cloud_ros = pcl_to_ros(cluster_cloud)
	table_cloud_ros = pcl_to_ros(table_cloud)
    
	# Publish ROS messages
	pcl_objects_pub.publish(objects_cloud_ros)
	pcl_table_pub.publish(table_cloud_ros)

    
	# Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []
    color_features = []
	for i,indices in enumerate(clusters):
		point_cloud_list = []
        for j,indice in enumerate(indices):
			point_cloud_list.append([objects_cloud[indice][0],objects_cloud[indice][1],objects_cloud[indice][2],objects_cloud[indice][3]])
            color  = float_to_rgb(objects_cloud[indice][3])
			color_features.append(color)
        cluster_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_cloud.from_list(point_cloud_list)
        
		# Compute the associated colors feature vector
	    channel_1_vals = []
	    channel_2_vals = []
	    channel_3_vals = []
	    for color in color_features:
		    channel_1_vals.append(color[0])
		    channel_2_vals.append(color[1])
		    channel_3_vals.append(color[2])
	    channel1_hist = np.histogram(channel_1_vals,bins=32,range=(0,256))
	    channel2_hist = np.histogram(channel_2_vals,bins=32,range=(0,256))
	    channel3_hist = np.histogram(channel_3_vals,bins=32,range=(0,256))
	    color_feature_hist = np.float32(np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0])))
	    color_normed_hist = color_feature_hist / np.sum(color_feature_hist)
	    
		# Compute the associated Normal Feature vector
	    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
        normal_features = get_normals(ros_cluster_cloud)
        norm_x_values = []
	    norm_y_values = []
	    norm_z_values = []
	    for norm_component in pc2.read_points(normal_cloud,field_names=('normal_x','normal_y','normal_z'),skip_nans=True):
		    norm_x_vals.append(norm_component[0])
		    norm_y_vals.append(norm_component[1])
		    norm_z_vals.append(norm_component[2])
   	    channel1_hist = np.histogram(norm_x_vals,bins=32,range=(min(norm_x_vals),max(norm_x_vals)))
   	    channel2_hist = np.histogram(norm_y_vals,bins=32,range=(min(norm_y_vals),max(norm_y_vals)))
    	channel3_hist = np.histogram(norm_z_vals,bins=32,range=(min(norm_z_vals),max(norm_z_vals)))
    	normal_features_hist = np.float32(np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0])))
    	normed_normal_hist = features / np.sum(features)
	    
		# Concatenate all the features
        features = np.concatenate((normal_color_features,normal_normed_features))	
        
		# Make the prediction
	    prediction = model.predict(scaler.transform(features).reshape(-1,1))
        
		# Publish a label into RViz
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        label_pos = list(white_cloud[indices[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        
		# Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)
    
	# Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)
    

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering',anonymous=True)
    
	# Create Subscribers
    pcl_sub = rospy.Subscriber("sensor_stick/point_cloud",pc2.PointCloud2,pcl_callback,queue_size=1)
    
	# Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects",pc2.PointCloud2,queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table",pc2.PointCloud2,queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=1)
    object_markers_pub = rospy.Publisher("/object_marker",Marker,queue_size=1)
    
	# Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    
	# Initialize color_list
    get_color_list.color_list = []
    
	# Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()