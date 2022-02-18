#!/usr/bin/env python

# Import modules
import rospy
import yaml
import tf
import pickle

import matplotlib.colors
import matplotlib.pyplot as plt 
import numpy as np
import sklearn as sk

from sklearn.preprocessing import LabelEncoder
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *


from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    # Initialize variables
    labels = []
    centroids = [] # to be list of tuples (x,y,z)
    dict_object_param = {}
    dict_box_param = {}
    list_yaml = []
    test_scene_num = Int32()
    test_scene_num.data = 1
    yaml_filename = '~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/output.yaml'
    # Get/Read parameters
	# Note: You can add a ros parameter so that you choose which file to open based on that parameter
    object_list_param = rospy.get_param('/object_list_1') 
    dropbox_list_param = rospy.get_param('/dropbox')
    for object,box in zip(object_list_param,dropbox_list_param):
        dict_object_param[object['name']] = object['group']
        dict_box_param[box['group']] = [box['name'],box['position']]    
    # Loop through the pick list
    for object in object_list:
        # Get the PointCloud for a given object and obtain it's centroid
        cloud = object.cloud 
        label = object.label
        centroid = np.mean(ros_to_pcl(cloud).to_array(),axis=0)[:,3]
        centroids.append(centroid)
        labels.append(label)
        # Create 'pick_pose' for the object
        pick_pose = Pose()
        pick_pose.position.x = float(centroid[0])
        pick_pose.position.y = float(centroid[1])
        pick_pose.position.z = float(centroid[2])
        pick_pose.orientation.w = 1.
        # Assign the arm to be used for pick_place
        group = dict_object_param[label]
        which_arm = String()
        which_arm.data = dict_box_param[group][0]
        # Create 'place_pose' for the object
        place_pose = Pose()
        place_pose.position.x = dict_box_param[group][1][0]
        place_pose.position.y = dict_box_param[group][1][1]
        place_pose.position.z = dict_box_param[group][1][2]
        place_pose.orientation.w = 1.0
        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        list_yaml.append(make_yaml_dict(test_scene_num,which_arm,label,pick_pose,place_pose))
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            resp = pick_place_routine(test_scene_num, label, which_arm, pick_pose, place_pose)
            print ("Response: ",resp.success)
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)
    # Output your request parameters into output yaml file
    send_to_yaml(yaml_filename,list_yaml)


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    # Convert ROS msg to PCL data
	point_cloud = ros_to_pcl(pcl_msg)
    # Statistical outlier detection
	outlier_filter = point_cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(10)
	outlier_filter.set_std_dev_mul_thresh(0.1)
	point_cloud = outlier_filter.filter()
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
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_list = []
	detected_objects_labels = []
	for i,indices in enumerate(cluster_indices):
		cloud = objects_cloud.extract(indices)
		# transform each PCL cloud to ros cloud
		ros_cloud = pcl_to_ros(cloud)
		# extract color features
		channel_1 = []
		channel_2 = []
		channel_3 = []
		for point in pc2.read_points(ros_cloud,skip_nans=True):
			color = float_to_rgb(point[2]) 
			color_hsv = rgb_to_hsv(color) * 255
			channel_1.append(color_hsv[0])
			channel_2.append(color_hsv[1])
			channel_3.append(color_hsv[2])
		channel_1_hist = np.histogram(channel_1,bins=32,range=(0,256))
		channel_2_hist = np.histogram(channel_2,bins=32,range=(0,256))
		channel_3_hist = np.histogram(channel_3,bins=32,range=(0,256))
		color_hist = np.float32(np.concatenate((channel_1_hist[0],channel_2_hist[0],channel_3_hist[0])))
		normed_color_hist = color_hist / np.sum(color_hist)
		# Normal Features
		normal_cloud = get_normals(ros_cloud)
		normal_x = []
		normal_y = []
		normal_z = []
		for norm_component in pc2.read_points(normal_cloud,field_names=('normal_x','normal_y','normal_z'),skip_nans=True):
			normal_x.append(norm_component[0])
			normal_y.append(norm_component[1])			
			normal_z.append(norm_component[2])
   		normal_x_hist = np.histogram(normal_x,bins=32,range=(min(normal_x),max(normal_x)))
   		normal_y_hist = np.histogram(normal_y,bins=32,range=(min(normal_y),max(normal_y)))
    	normal_z_hist = np.histogram(normal_z,bins=32,range=(min(normal_z),max(normal_z)))
    	normal_hist = np.float32(np.concatenate((normal_x_hist[0],normal_y_hist[0],normal_z_hist[0])))
    	normed_normal_hist = normal_hist / np.sum(normal_hist)
		normed_color_hist = compute_color_histograms(ros_cloud,using_hsv=True)
		features = np.concatenate((normed_color_hist,normed_normal_hist))
		# Make the prediction
		scaled_features = scaler.transform(features.reshape(1,-1))
		prediction = clf.predict(scaled_features)
        # Publish a label into RViz
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)
		marker = Marker()
		marker.header.frame_id = '/world'
		marker.id = i
		marker.type = marker.TEXT_VIEW_FACING
		marker.text = label
		position = cloud[0]
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.pose.position.x = position[0]
		marker.pose.position.y = position[1]
		marker.pose.position.z = position[2] + 0.4
		marker.pose.orientation.w = 1.
		marker.color.r = 0.8
		marker.color.g = 0.8
		marker.color.b = 0.8
		marker.color.a = 1.
		marker.lifetime = rospy.Duration(5.0)
		label_pos = list(white_cloud[indices[0]])
		marker_pub.publish(marker)
		# declare a detected object
		do = DetectedObject()
		do.label = label
		do.cloud = ros_cloud
		detected_objects_list.append(do)
	rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),detected_objects_labels))
	detected_objects_pub.publish(detected_objects_list)
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass




if __name__ == '__main__':
	# ROS node initialization
	rospy.init_node('clustering',anonymous=True)
    # Create Subscribers
	pcl_sub = rospy.Subscriber("pr2/world/points",pc2.PointCloud2,pcl_callback,queue_size=1)
    # Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects",pc2.PointCloud2,queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table",pc2.PointCloud2,queue_size=1)
	marker_pub = rospy.Publisher("/object_marker",Marker,queue_size=1)
	detected_objects_pub = rospy.Publisher("/object_detects",DetectedObjectsArray,queue_size=1)    
	# Load the model from the disk
	model = pickle.load(open('/home/robond/catkin_ws/model.sav','rb'))
	clf = model['classifier']
	encoder = LabelEncoder()
	encoder.classes_ = model['classes']
	scaler = model['scaler']
	# Initialize color_list
    get_color_list.color_list = []
    # Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
