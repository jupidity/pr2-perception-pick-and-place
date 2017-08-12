/*
ROS node for point cloud cluster based segmentaion of cluttered objects on table
Author: Sean Cassero
7/15/15
*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pr2_robot/SegmentedClustersArray.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define DEBUG

class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("/pr2/world/points", 1, &segmentation::cloud_cb, this);
    m_clusterPub = m_nh.advertise<pr2_robot::SegmentedClustersArray> ("pr2_robot/pcl_clusters",1);

    #ifdef DEBUG // add in the publishers to look at the output through Rviz if debugging
    m_voxelPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_voxel",1);
    m_outliersPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_ouliers",1);
    m_passthroughPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_passthrough",1);
    m_ransacPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_ransac",1);
    m_passthrough2Pub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_passthrough2",1);
    m_coloredClustersPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/color_clusters",1);
    #endif

    // declare the containers for parameters
    float vf_leaf_size;
    float ec_cluster_tolerance;
    int ec_minimum_cluster_size;
    int ec_maximum_cluster_size;
    float of_mean_k;
    float of_std_dev;
    float pz_lower_limit;
    float pz_upper_limit;
    double py_lower_limit;
    float py_upper_limit;
    float rs_distance_threshold;


    // get the parameters from the parameter server
    ros::param::get("/filters/voxel_filter/leaf_size", vf_leaf_size );
    ros::param::get("/filters/euclidean_cluster/cluster_tolerance", ec_cluster_tolerance );
    ros::param::get("/filters/euclidean_cluster/maximum_cluster_size", ec_maximum_cluster_size );
    ros::param::get("/filters/euclidean_cluster/minimum_cluster_size", ec_minimum_cluster_size);
    ros::param::get("/filters/outlier_filter/mean_k", of_mean_k );
    ros::param::get("/filters/outlier_filter/std_dev", of_std_dev );
    ros::param::get("/filters/passthrough_z/lower_limit", pz_lower_limit);
    ros::param::get("/filters/passthrough_z/upper_limit", pz_upper_limit );
    m_nh.param<double>("/filters/passthrough_y/lower_limit", py_lower_limit , py_lower_limit);
    ros::param::get("/filters/passthrough_y/upper_limit", py_upper_limit );
    ros::param::get("/filters/ransac_segmentation/distance_threshold", rs_distance_threshold );


    // set voxel filter parameters
    voxelFilter.setLeafSize (vf_leaf_size,vf_leaf_size,vf_leaf_size);
    // set passthrough filter parameters
    passY.setFilterFieldName ("y");
    //ROS_INFO_STREAM("Lower Limit: "<<py_lower_limit);
    //ROS_INFO_STREAM("Upper Limit: "<<py_upper_limit);
    passY.setFilterLimits (py_lower_limit, py_upper_limit);
    // set z passthrough filter parameters
    passZ.setFilterFieldName ("z");

    passZ.setFilterLimits (pz_lower_limit, pz_upper_limit);
    // set outlier filter parameters
    outlierFilter.setMeanK (of_mean_k);
    outlierFilter.setStddevMulThresh (of_std_dev);
    // set ransac filter parameters
    ransacSegmentation.setOptimizeCoefficients (true);
    ransacSegmentation.setModelType (pcl::SACMODEL_PLANE);
    ransacSegmentation.setMethodType (pcl::SAC_RANSAC);
    ransacSegmentation.setDistanceThreshold (rs_distance_threshold);
    extract.setNegative (true);
    // specify euclidean cluster parameters
    ec.setClusterTolerance (ec_cluster_tolerance); // 2cm
    ec.setMinClusterSize (ec_minimum_cluster_size);
    ec.setMaxClusterSize (ec_maximum_cluster_size);


  }

private:

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
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

  //declare the output containers
  sensor_msgs::PointCloud2 output; // output sensor_msgs
  pcl::PCLPointCloud2 outputPCL;  // output pcl

  #ifdef DEBUG // add the extra publishers if in debug mode
  ros::Publisher m_voxelPub;
  ros::Publisher m_outliersPub;
  ros::Publisher m_passthroughPub;
  ros::Publisher m_ransacPub;
  ros::Publisher m_passthrough2Pub;
  ros::Publisher m_coloredClustersPub;

  #endif

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{


  // get pointers to new pcl objects
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; // // pcl object to hold the conversion from sensor_msgs::PointCloud2 data type
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2; // pcl object to hold the voxel filtered cloud
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>; // pcl object to hold the conversion from pcl::PointCloud2 data type
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>; // pcl object to hold the passthrough filtered data in the y direction

  // get the shared pointers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  voxelFilter.setInputCloud (cloudPtr);
  voxelFilter.filter (*cloudFilteredPtr);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl_conversions::fromPCL(*cloudFilteredPtr, output);
  m_voxelPub.publish(output);
  #endif

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

  //perform passthrough filtering in the y dir
  passY.setInputCloud (xyzCloudPtr);
  passY.filter (*xyzCloudPtrFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthroughPub.publish(output);
  #endif

  // passthrough filter in the z dir
  passZ.setInputCloud (xyzCloudPtrFiltered);
  passZ.filter (*xyzCloudPtrFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthrough2Pub.publish(output);
  #endif

  // perform outlier filtering
  outlierFilter.setInputCloud (xyzCloudPtrFiltered);
  outlierFilter.filter (*xyzCloudPtrFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_outliersPub.publish(output);
  #endif


  // perform RANSAC segmentation and extract outliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  ransacSegmentation.setInputCloud (xyzCloudPtrFiltered);
  ransacSegmentation.segment (*inliers, *coefficients);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.filter (*xyzCloudPtrFiltered);


  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_ransacPub.publish(output);
  #endif

  // perform euclidean cluster segmentation to seporate individual objects
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices

  // Create the KdTree object for the search method of the extraction

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrFiltered);
  ec.setSearchMethod(tree);
  ec.setInputCloud (xyzCloudPtrFiltered);
  cluster_indices.clear();
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  pr2_robot::SegmentedClustersArray CloudClusters;

  #ifdef DEBUG
  uint32_t j =0;
  uint32_t color=0;
  #endif

  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them separately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {

    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrFiltered->points[*pit]);

      #ifdef DEBUG
      xyzCloudPtrFiltered->points[*pit].rgb =  color ;
      #endif
    }


    #ifdef DEBUG
    ++j;
    color += 0x00000e << (j*3) ;
    #endif
    // populate the output message
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL); // convert to pcl::PCLPointCloud2
    pcl_conversions::fromPCL(outputPCL, output); // Convert to ROS data type
    CloudClusters.clusters.push_back(output); // add the cluster to the array message

  }

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_coloredClustersPub.publish(output);
  #endif
  
  // publish the clusters
  m_clusterPub.publish(CloudClusters);

}



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
