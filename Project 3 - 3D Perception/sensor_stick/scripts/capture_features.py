#!/usr/bin/env python
import numpy as np
import pickle
import yaml
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('capture_node')
    # Let the user choose which object list he wants to train the SVM for
    choice = input("Enter 1 for Test_world 1.\nEnter 2 for Test_world 2.\nEnter 3 for Test_world 3.:\n")
    choice = int(choice)
    if choice == 1:
        list_name = '/object_list_1'
    elif choice == 2:
        list_name = '/object_list_2'
    elif choice == 3:
        liste_name = '/object_list_3'
    else:
        print("Wrong Choice, terminating the program.")
        pass
    # Parse the chosen objects
    object_list_param = rospy.get_param(list_name)
    models = []
    for obj in object_list_param:
        models.append(obj['name'])
    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []
    for model_name in models:
        spawn_model(model_name)
        for i in range(20):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True
            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])
        delete_model()
    pickle.dump(labeled_features, open('training_set.sav', 'wb'))

