
[image1]: ./photos/originalScene.png
[image2]: ./photos/voxelFilter.png
[image3]: ./photos/passthroughY.png
[image4]: ./photos/passthroughZ.png
[image5]: ./photos/noisyPointCloud.png
[image6]: ./photos/ransacFilter.png
[image7]: ./photos/cluters.png
[image8]: ./photos/clusterMarkers.png
[image9]: ./photos/table_2.png
[image10]: ./photos/table_3.png
[image11]: ./photos/outlierFilter.png


### Overview
---

An RGBD perception pipeline for object recognition with the PR2 robot using ROS in Gazebo simulation.

In order to use the pcl library for point cloud processing in c++, and the sklearn Python library for object recognition, the perception pipeline was broken into two nodes: `pr2_segmentation`, a c++ node for point cloud cluster segmentation, and `marker_generation.py` a Python node for object recognition using SVM.

There were three "worlds" with which to test the success of our build, the following is an overview of perception performance in the first world.

After building the environment, the simulation can be launched by running the following commands in the shell in the root directory of your project


```sh
cd pr2_robot/scripts/
./pr2_pick_place_spawner.sh
```

which roslaunches the pick_place_project.launch and pick_place_perception.launch scripts, and rosruns marker_generation.py


## Point Cloud Segmentation
---


The `pr2_segmentation` nodes' basic functionality is to accept a `sensor_msgs::PointCloud2` input and separate individual clusters for object identification in the `marker_generation.py` script.

In order to accomplish this, we can take advantage of the pcl library and its various filter classes.

The main loop of `pr2_segmentation` is quite simple: initialize the node, declare an instance of the `segmentation` class, and sit back.

      int main (int argc, char** argv)
      {
        // Initialize ROS
        ros::init (argc, argv, "segmentation");
        ros::NodeHandle nh;

        // get the segmentaiton object
        segmentation segs(nh);

        while(ros::ok())
        ros::spin ();

      }

The `segmentation` class is in charge of handling point cloud publishing, subscribing, and filtration. It consists of a subscriber, a publisher, and all requisite filters.

      private:

        ros::NodeHandle m_nh;
        ros::Subscriber m_sub;
        ros::Publisher m_clusterPub;



        // Declare the filters
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter; // voxel filter
        pcl::PassThrough<pcl::PointXYZRGB> passY; // passthrough filter in the y dir
        pcl::PassThrough<pcl::PointXYZRGB> passZ; // pcl object to hold the passthrough filtered results in the z dir
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierFilter; // statistical outlier filter
        pcl::SACSegmentation<pcl::PointXYZRGB> ransacSegmentation; // ransac segmentation filter
        pcl::ExtractIndices<pcl::PointXYZRGB> extract; // extraction class for RANSAC segmentation
        std::vector<pcl::PointIndices> cluster_indices; // vector containing the segmented clusters
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; // extraction object for the clusters

The initialization steps for each filter are highlighted beneath its filter descriptions below.

In the class constructor, the publisher to `marker_generation.py` via the `pr2_robot/pcl_cluster` topic, and subscriber to the `/pr2/world/points` RGBD cloud are initialized

      // define the subscriber and publisher
      m_sub = m_nh.subscribe ("/pr2/world/points", 1, &segmentation::cloud_cb, this);
      m_clusterPub = m_nh.advertise<pr2_robot::SegmentedClustersArray> ("pr2_robot/pcl_clusters",1);


The bulk of the computation happens in the callback function to the `pr2/world/points` topic.

       void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)

The callback dynamically allocates memory for the containers for the point clouds before and after filtration,

      // get pointers to new pcl objects
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // // pcl object to hold the conversion from sensor_msgs::PointCloud2 data type
      pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2; // pcl object to hold the voxel filtered cloud
      pcl::PointCloud<pcl::PointXYZRGB> * xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>; // pcl object to hold the conversion from pcl::PointCloud2 data type
      pcl::PointCloud<pcl::PointXYZRGB> * xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>; // pcl object to hold the passthrough filtered data in the y direction


generates shared pointers for inputs to the filter functions,

      // get the shared pointers
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);
      pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

performs conversions from the native `ROS` message type `sensor_msgs::PointCloud2` to the `PCL` data type `pcl::PointCloud<pcl::PointXYZRGB>`,

      // Convert to PCL data type
      pcl_conversions::toPCL(* cloud_msg, * cloud);


and passes the cloud through a series of filters.

In order to visualize the point cloud processing, uncomment `#define DEBUG` in the `/src/pr2_segmentation.cpp` script and recompile with `catkin_make`. The point cloud after each filtration can be seen in RViz. Normal node functionality does not generate these output clouds.

The world 1 scene is simulated as three objects in close proximity on a table

![alt text][image1]

In order to simulate real world behavior, The pr2's RGBD camera publishes noisy point cloud data on the `pr2/world/points/` topic, which serves as the input to the `pr2_segmentation` script.

![alt text][image5]

The initial cloud is too dense to perform calculation in a reasonable amount of time, so we perform voxel sampling to condense all points in a volume `leaf_size`^3 to a single point containing the average of their values.

The voxel filter is a private variable in the `segmentation` class, and is initialized in the class contructor with parameters from the `filter_parameters.yaml` script in the config directory. From the constructor:

From segmentation::segmentation()

      ros::param::get("/filters/voxel_filter/leaf_size", vf_leaf_size );
      ...
      // set voxel filter parameters
      voxelFilter.setLeafSize (vf_leaf_size,vf_leaf_size,vf_leaf_size);


 during the callback, we just need to pass the initial cloud to the filter and specify a smart pointer to the output container.

from segmentation::cloud_cb()

      // Perform voxel grid downsampling filtering
      voxelFilter.setInputCloud (cloudPtr);
      voxelFilter.filter (* cloudFilteredPtr);

Resulting in the more pixelated, less computationally expensive cloud:

![alt text][image2]

Even after downsampling, the cloud is too broad for segmentation and clustering algorithms to work properly and in a short amount of time. Humans have the advantage of specifying a region of focus when analyzing a large region, we should do the same for our robot. Practically this can be achieved by two passthrough filters in the `z` and `y` directions. Just like the voxel filter, both passthough filters were instantiated previously in the class constructor with ranges specified by the `filter_parameters.yaml` script,

From segmentation::segmentation()

      ros::param::get("/filters/passthrough_z/lower_limit", pz_lower_limit);
      ros::param::get("/filters/passthrough_z/upper_limit", pz_upper_limit );
      m_nh.param<double>("/filters/passthrough_y/lower_limit", py_lower_limit , py_lower_limit);
      ros::param::get("/filters/passthrough_y/upper_limit", py_upper_limit );
      ...
      passY.setFilterFieldName ("y");
      passY.setFilterLimits (py_lower_limit, py_upper_limit);
      passZ.setFilterFieldName ("z");
      passZ.setFilterLimits (pz_lower_limit, pz_upper_limit);

so we can achieve a focused cloud by passing them pointers to the input and output point cloud data containers.

from segmentation::cloud_cb()

In the y direction:

      //perform passthrough filtering in the y dir
      passY.setInputCloud (xyzCloudPtr);
      passY.filter (* xyzCloudPtrFiltered);


![alt text][image3]

and in the z direction:

      // passthrough filter in the z dir
      passZ.setInputCloud (xyzCloudPtrFiltered);
      passZ.filter (* xyzCloudPtrFiltered);

![alt text][image4]

Before segmentation and clustering algorithms will function correctly, the noise resulting from faulty measurements or reflective dust must be removed from the cloud with a statistical outlier filter. All parameters are specified in the `filter_parameters.yaml` script and filter is instantiated in the `segmentation` class constructor

From segmentation::segmentation()

      ros::param::get("/filters/outlier_filter/mean_k", of_mean_k );
      ros::param::get("/filters/outlier_filter/std_dev", of_std_dev );
      ...
      outlierFilter.setMeanK (of_mean_k);
      outlierFilter.setStddevMulThresh (of_std_dev);

from segmentation::cloud_cb()

      // perform outlier filtering
      outlierFilter.setInputCloud (xyzCloudPtrFiltered);
      outlierFilter.filter (* xyzCloudPtrFiltered);

![alt text][image11]

Since we are interested in the objects on top of the table, we can perform RANSAC segmentation against a planar model to extract the points that fit a planar equation within a reasonable error and save the rest.

from segmentation::segmentation()

      ros::param::get("/filters/ransac_segmentation/distance_threshold", rs_distance_threshold );
      ...
      // set ransac filter parameters
      ransacSegmentation.setOptimizeCoefficients (true);
      ransacSegmentation.setModelType (pcl::SACMODEL_PLANE);
      ransacSegmentation.setMethodType (pcl::SAC_RANSAC);
      ransacSegmentation.setDistanceThreshold (rs_distance_threshold);
      extract.setNegative (true);

from segmentation::cloud_cb()

      // perform RANSAC segmentation and extract outliers
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      ransacSegmentation.setInputCloud (xyzCloudPtrFiltered);
      ransacSegmentation.segment (* inliers, * coefficients);
      extract.setInputCloud (xyzCloudPtrFiltered);
      extract.setIndices (inliers);
      extract.filter (* xyzCloudPtrFiltered);

![alt text][image6]

All thats left are the objects of interest. In order to separate each individual object cloud, we perform Euclidean Cluster Extraction to group points with their closest cluster and save each cluster to a vector of PCL point clouds.

from segmentation::segmentation()

      // get the parameters from the parameter server
      ros::param::get("/filters/euclidean_cluster/cluster_tolerance", ec_cluster_tolerance );
      ros::param::get("/filters/euclidean_cluster/maximum_cluster_size", ec_maximum_cluster_size );
      ros::param::get("/filters/euclidean_cluster/minimum_cluster_size", ec_minimum_cluster_size);
      ...
      ec.setClusterTolerance (ec_cluster_tolerance); // 2cm
      ec.setMinClusterSize (ec_minimum_cluster_size);
      ec.setMaxClusterSize (ec_maximum_cluster_size);

from segmentation::cloud_cb()

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (xyzCloudPtrFiltered);
      ec.setSearchMethod(tree);
      ec.setInputCloud (xyzCloudPtrFiltered);
      cluster_indices.clear();
      ec.extract (cluster_indices);


Below the clusters are colorized to show individual identification.

![alt text][image7]

Now that each object of interest is separated into its own point cloud, each point cloud cluster is packaged in a PointCloud2 vector as an ROS message and sent to the python `marker_generation.py` via a `pr2_robot::SegmentedClustersArray` message on the `pr2/pcl_clusters` topic.


      // declare an instance of the SegmentedClustersArray message
      pr2_robot::SegmentedClustersArray CloudClusters;

      // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them separately
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {

        // create a pcl object to hold the extracted cluster
        pcl::PointCloud<pcl::PointXYZRGB> * cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

        // now we are in a vector of indices pertaining to a single cluster.
        // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          clusterPtr->points.push_back(xyzCloudPtrFiltered->points[* pit]);

        }

        // populate the output message
        pcl::toPCLPointCloud2( * clusterPtr ,outputPCL); // convert to pcl::PCLPointCloud2
        pcl_conversions::fromPCL(outputPCL, output); // Convert to ROS data type
        CloudClusters.clusters.push_back(output); // add the cluster to the array message

      }

      // publish the clusters
      m_clusterPub.publish(CloudClusters);

And the `pr2_segmentation` callback is completed.


### Object Recognition and Identification
---

`marker_generation.py` is responsible for three tasks:

- perform point cloud object classification against the SVM model.
- generate labels in RViz for each recognized object.
- generate a yaml output file with a server request for the pick and place operation.

Once the `marker_generation.py` script receives the vector of point cloud clusters from the `pr2_segmentation` node, it begins by classifying each cluster. The SVM model in this example is trained with 256 bins for HSV color and normals, with 40 randomly generated orientations of each potential object of interest. A full description of how the SVM model was trained can be found in the repo https://github.com/jupidity/svm_model_generation.

Each point in the cluster is passed into an array of 3 color channels

        for point in pc2.read_points(cloud, skip_nans=True):
            rgb_list = float_to_rgb(point[3])
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)

      for color in point_colors_list:
            channel_1_vals.append(color[0])
            channel_2_vals.append(color[1])
            channel_3_vals.append(color[2])

histograms of the proper bin number are computed

        # Compute histograms for the colors in the point cloud
        channel1_hist = np.histogram(channel_1_vals, bins=numBins, range=(0, 256))
        channel2_hist = np.histogram(channel_2_vals, bins=numBins, range=(0, 256))
        channel3_hist = np.histogram(channel_3_vals, bins=numBins, range=(0, 256))

and concatenated into normalized feature vectors

      hist_features = np.concatenate((channel1_hist[0],channel2_hist[0], channel3_hist[0])).astype(np.float64)
        normed_features = hist_features / np.sum(hist_features)       

to be passed into the svm modle for classification

      prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))

The process is repeated for the surface normals of each cluster across the range (-1,1)

 Once classification is complete, a detected_object message is generated containing the label, label position, and corresponding point cloud index and passed to RViz.  

    object_markers_pub.publish(make_label(label,centroids[index], index))

resulting in the following classification

![alt text][image8]

and finally, the results of the classification for each cluster, its centroid position, and information for the pick and place operation are saved in the ROS service format for a pick and place operation

      item_name = String()
              place_pose = Pose()
              pick_pose = Pose()
              arm_name = String()
              test_scene_num = Int32()

              # populate the msgs with data
              item_name.data = obj['name']
              pick_pose.position.x = np.asscalar(centroids[index][0])
              pick_pose.position.y = np.asscalar(centroids[index][1])
              pick_pose.position.z = np.asscalar(centroids[index][2]-.4)
              test_scene_num.data = scene_num
              arm_name.data = dropbox_data[0]['name']

              if (obj['group'] == dropbox_data[0]['group']):
                  arm_name.data = dropbox_data[0]['name']
                  #rospy.loginfo("{}".format(dropbox_data[0]['position'][0]))
                  place_pose.position.x = dropbox_data[0]['position'][0]
                  place_pose.position.y = dropbox_data[0]['position'][1]
                  place_pose.position.z = dropbox_data[0]['position'][2]
              else:
                  #rospy.loginfo("{}".format(dropbox_data[0]['position'][0]))
                  arm_name.data = dropbox_data[1]['name']
                  place_pose.position.x = dropbox_data[1]['position'][0]
                  place_pose.position.y = dropbox_data[1]['position'][1]
                  place_pose.position.z = dropbox_data[1]['position'][2]

and a yaml output script containing all service requests is saved to an output script for verification

        yaml_dict = make_yaml_dict(test_scene_num, arm_name, item_name,  pick_pose, place_pose,)
                dict_list.append(yaml_dict)  

        ...

        # save the Yaml dictionary generated above to an output Yaml file
          yaml_filename = "output_" + "%s"%(scene_num) + ".yaml"
          if len(dict_list) > 0:
              send_to_yaml(yaml_filename, dict_list)

The yaml output files are located in the ``/scripts`` directory



The process can be repeated for worlds 2 and 3

world 2:

80% (4/5) correct object identification. Here the glue is incorrectly identified as beer.


![alt text][image9]

world 3

87.5% (7/8) correct object identification. All objects are labeled correctly, but the clustering algorithm is unable to identify the glue as an independent cluster, and only 7 objects are recognized.

![alt text][image10]





### Performance and Improvements
---
The model performed relatively well, achieving perfect object classification in world 1, 80% correct classification is world 2, 87.5% classification in world 3, and able to be executed in real time at about 3fps.

In the `pr2_segmentation` callback, code flow could have been sped up by defining each filter as a nodelet and passing smart pointers between filters, so each filter could work independently without passing the entire data structure between nodes. Performance would then be dictated by the execution time of the slowest filter rather than the sum of all filters execution time.

Also, there could be performance improvements examining the optimal tradeoff between voxel filter leaf_size and object recognition accuracy, the value chosen worked but was not necessarily optimal.

In terms of SVM model training, accuracy could have been improved in a number of ways. I was satisfied with performance (~93% accuracy) after 40 iterations of each object, but I imagine better and more consistent performance could have been achieved with more samples per object.

Also after discussing with some other members of the cohort, the data from the normals appeared to be noisy and inconsistent at low sample numbers. Apparently classification accuracy was highly dependent on color histograms with people able to achieve close to 100% object classification more or less ignoring normals in their SVM. I was able to achieve 90% classification accuracy without normals, and 92% accuracy with normals for 40 samples per object in training. This is probably due to the consistent lighting in simulation leading to consistent object coloration across runs, but investigation on how to better include noise resistant geometric data for feature vectors will be useful in future applications of SVM. As a first pass, perhaps increasing the `setRadiusSearch(0.03);` parameter in the `feature-extractor.cpp` node could lead to more consistent planes for normal estimation, or decreasing the voxel filter `leaf_size` on the input cloud. Also investigation into HOG features might be a better way to do geometric classification for SVM training.    
