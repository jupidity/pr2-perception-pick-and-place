#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle


from pr2_robot.features import compute_color_histograms
from pr2_robot.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from pr2_robot.srv import GetNormals

from pr2_robot.marker_tools import *
from pr2_robot.msg import DetectedObjectsArray
from pr2_robot.msg import DetectedObject
from pr2_robot.msg import SegmentedClustersArray
from pr2_robot.pcl_helper import *

from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from pr2_robot.srv import PickPlace

def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/pr2_feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your subscriber to the cluster array
def pcl_callback(pcl_msg):


    # TODO Classify the clusters!

    # declare holders for the labels and objects
    detected_objects_labels = []
    detected_objects = []
    labels = []
    centroids = [] # to be list of tuples (x, y, z)

    # get parameters
    object_list_param = rospy.get_param('/object_list')


    for index, pcl_cloud in enumerate(pcl_msg.clusters):

        # Extract histogram features
        chists = compute_color_histograms(pcl_cloud, using_hsv=True)
        normals = get_normals(pcl_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))

        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        points_array = ros_to_pcl(pcl_cloud).to_array()
        centroids.append(np.mean(points_array, axis=0)[:3])
        centroids[index][2] += .4
        object_markers_pub.publish(make_label(label,centroids[index], index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cloud
        detected_objects.append(do)

        # TODO search through the list of pick place items and see if label matches any items

        # if it does, gerate the yaml dictionary entry

        # create the ROS msg types and populate them with the parameters

        # create a Yaml dictionary entry and add the messages to it

        # push the yaml dictionary entry to an array



    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))


    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # TODO save the Yaml dictionary generated above to an output Yaml file

if __name__ == '__main__':

    rospy.init_node('label_node')

    # Create Subscribers
    sub = rospy.Subscriber('pr2_robot/pcl_clusters' , SegmentedClustersArray, pcl_callback )
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray , queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker , queue_size=1)


    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    while not rospy.is_shutdown():
        rospy.spin()
