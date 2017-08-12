
[image1]: ./photos/originalScene.png
[image2]: ./photos/voxelFilter.png
[image3]: ./photos/passthroughY.png
[image4]: ./photos/passthroughZ.png
[image5]: ./photos/noisyPointCloud.png
[image6]: ./photos/ransacFilter.png
[image7]: ./photos/clusters.png
[image8]: ./photos/clusterMarkers.png
[image9]: ./photos/table_2.png
[image10]: ./photos/table_3.png
[image11]: ./photos/outlierFilter.png


### Overview
---

An RGBD perception pipeline for object recognition with the PR2 robot using ROS in Gazebo simulation.
The complete perception pipeline can be broken into two sections, point cloud cluster segmentation and object recognition using SVM. There were three "worlds" with which to test the success of our build, the following is an overview of perception performance in the first world.

After building the environment, the simulation can be lauched by running the following commands in the shell

```sh
roslaunch pr2_robot pick_place_project.launch
```

and in a separate terminal

```sh
roslaunch pr2_robot pick_place_perception.launch
```

## Point Cloud Segmentation
---

In order to visualize the point cloud processing, we can define DEBUG in the `/src/pr2_segmentation.cpp` script. The point cloud after each filtration can be seen in RViz.

The worrld 1 scene is simulated as three objects in close proximity on a table

![alt text][image1]

In order to simulate real world behavior, the input cloud to the segmentation node is a point cloud in the sensor_msgs::PointCloud2 format of the above scene with added noise due to sensor distortion:

![alt text][image5]


code flow during callback is as follows:

In order to reduce computational expense, we perform voxel downgrid sampling.

![alt text][image2]

In order to focus our computational efforts, we perform two passthrough filter operations to narrow field of vision

In the y direction:

![alt text][image3]

in the z direction:

![alt text][image4]

In order to remove noise, we pass the point cloud through a statistical outlier filter to remove points that are above a maximum standard deviation geographical distance away from neighboring points

![alt text][image11]

Once the point cloud is clean and focused, we perform RANSAC plane filtration to remove the table

![alt text][image6]

All thats left is the objects of interest. In order to separate each individual object cloud, we perform Euclidean Cluster Extraction to group points with their closest geographical cluster.

![alt text][image7]

Now that each object of interest is separated into its own point cloud, each point cloud cluster is packaged in a PointCloud2 vector as an ROS message and sent to the python `marker_generation.py` node for classification against our SVM model. After classification, the centroid of the point cloud is computed and a label is sent to RViz containing the result of the classification.

![alt text][image8]

The process can be repeated for worlds 2 and 3

world 2

![alt text][image9]

world 3

![alt text][image10]

The model performed relatively well, achieving perfect object classification in world 1 and 3, 80% correct classification is world 2, and able to be executed in real time.
