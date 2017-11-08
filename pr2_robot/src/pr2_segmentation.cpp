/*
ROS node for point cloud cluster based segmentaion of cluttered objects on table
Author: Sean Cassero
7/15/17
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


class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : nh_(nh)  {

    sub_ = nh_.subscribe ("/pr2/world/points", 1, &segmentation::CloudCallback, this);
    cluster_pub_ = nh_.advertise<pr2_robot::SegmentedClustersArray> ("pr2_robot/pcl_clusters",1);

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
    ros::param::get("/filters/voxel_filter/leaf_size", vf_leaf_size );
    ros::param::get("/filters/euclidean_cluster/cluster_tolerance", ec_cluster_tolerance );
    ros::param::get("/filters/euclidean_cluster/maximum_cluster_size", ec_maximum_cluster_size );
    ros::param::get("/filters/euclidean_cluster/minimum_cluster_size", ec_minimum_cluster_size);
    ros::param::get("/filters/outlier_filter/mean_k", of_mean_k );
    ros::param::get("/filters/outlier_filter/std_dev", of_std_dev );
    ros::param::get("/filters/passthrough_z/lower_limit", pz_lower_limit);
    ros::param::get("/filters/passthrough_z/upper_limit", pz_upper_limit );
    nh_.param<double>("/filters/passthrough_y/lower_limit", py_lower_limit , py_lower_limit);
    ros::param::get("/filters/passthrough_y/upper_limit", py_upper_limit );
    ros::param::get("/filters/ransac_segmentation/distance_threshold", rs_distance_threshold );

    VoxelFilter_.setLeafSize (vf_leaf_size,vf_leaf_size,vf_leaf_size);
    YPassthoughFilter_.setFilterFieldName ("y");
    YPassthoughFilter_.setFilterLimits (py_lower_limit, py_upper_limit);
    ZPassthoughFilter_.setFilterFieldName ("z");
    ZPassthoughFilter_.setFilterLimits (pz_lower_limit, pz_upper_limit);
    OutlierFilter_.setMeanK (of_mean_k);
    OutlierFilter_.setStddevMulThresh (of_std_dev);
    RansacSegmentationFilter_.setOptimizeCoefficients (true);
    RansacSegmentationFilter_.setModelType (pcl::SACMODEL_PLANE);
    RansacSegmentationFilter_.setMethodType (pcl::SAC_RANSAC);
    RansacSegmentationFilter_.setDistanceThreshold (rs_distance_threshold);
    ExtractionBuffer_.setNegative (true);
    EuclideanClusters_.setClusterTolerance (ec_cluster_tolerance); 
    EuclideanClusters_.setMinClusterSize (ec_minimum_cluster_size);
    EuclideanClusters_.setMaxClusterSize (ec_maximum_cluster_size);
  }

private:

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher cluster_pub_;
  pcl::VoxelGrid<pcl::PCLPointCloud2> VoxelFilter_; 
  pcl::PassThrough<pcl::PointXYZRGB> YPassthoughFilter_; 
  pcl::PassThrough<pcl::PointXYZRGB> ZPassthoughFilter_; 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> OutlierFilter_;
  pcl::SACSegmentation<pcl::PointXYZRGB> RansacSegmentationFilter_; 
  pcl::extractIndices<pcl::PointXYZRGB> ExtractionBuffer_; 
  std::vector<pcl::PointIndices> cluster_indices; 
  pcl::EuclideanClusterextraction<pcl::PointXYZRGB> EuclideanClusters_;
  sensor_msgs::PointCloud2 output_pointcloud_; 
  pcl::PCLPointCloud2 output_pointcloud_PCL_;  

  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // segmentation


void segmentation::CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //
  // the PCL lib filter classes need to be passed smart pointers 
  //
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2; 
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>; 
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  //
  // point cloud preprocessing before extraction to focus on known area of interest
  // and remove noise and planar table surface 
  //
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  VoxelFilter_.setInputCloud (cloudPtr);
  VoxelFilter_.filter (*cloudFilteredPtr);
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);
  YPassthoughFilter_.setInputCloud (xyzCloudPtr);
  YPassthoughFilter_.filter (*xyzCloudPtrFiltered);
  ZPassthoughFilter_.setInputCloud (xyzCloudPtrFiltered);
  ZPassthoughFilter_.filter (*xyzCloudPtrFiltered);
  OutlierFilter_.setInputCloud (xyzCloudPtrFiltered);
  OutlierFilter_.filter (*xyzCloudPtrFiltered);
  RansacSegmentationFilter_.setInputCloud (xyzCloudPtrFiltered);
  RansacSegmentationFilter_.segment (*inliers, *coefficients);
  ExtractionBuffer_.setInputCloud (xyzCloudPtrFiltered);
  ExtractionBuffer_.setIndices (inliers);
  ExtractionBuffer_.filter (*xyzCloudPtrFiltered);

  //
  // perform euclidean cluster segmentation to separate individual objects
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  //
  // Create the KdTree object for the search method of the extraction
  //
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrFiltered);
  EuclideanClusters_.setSearchMethod(tree);
  EuclideanClusters_.setInputCloud (xyzCloudPtrFiltered);
  cluster_indices.clear();
  EuclideanClusters_.ExtractionBuffer_ (cluster_indices);

  //
  // here, cluster_indices is a vector of indices for each cluster. 
  // iterate through each indices object to work with them separately
  //
  pr2_robot::SegmentedClustersArray CloudClusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrFiltered->points[*pit]);
    }
    pcl::toPCLPointCloud2( *clusterPtr ,output_pointcloud_PCL_); 
    pcl_conversions::fromPCL(output_pointcloud_PCL_, output_pointcloud_); 
    CloudClusters.clusters.push_back(output_pointcloud_); 
  }

  cluster_pub_.publish(CloudClusters);

} // CloudCallback()



int main (int argc, char** argv)
{
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;

  segmentation segs(nh);

  while(ros::ok())
  ros::spin ();

}
