#!/usr/bin/env python

# Import modules
import numpy as np 
import sklearn 
import pickle

from sklearn.preprocessing import LabelEncoder
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from pcl_helper import *

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
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
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


if __name__ == '__main__':

    # ROS node initialization
	rospy.init_node('clustering',anonymous=True)
    # Create Subscribers
	pcl_sub = rospy.Subscriber("sensor_stick/point_cloud",pc2.PointCloud2,pcl_callback,queue_size=1)
    # Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects",pc2.PointCloud2,queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table",pc2.PointCloud2,queue_size=1)
    # Initialize color_list
    	get_color_list.color_list = []
    # Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()

