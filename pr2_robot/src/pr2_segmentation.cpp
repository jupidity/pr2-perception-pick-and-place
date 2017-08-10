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

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  { // constructor for the segmentation class

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("/pr2/world/points", 1, &segmentation::cloud_cb, this);
    m_clusterPub = m_nh.advertise<pr2_robot::SegmentedClustersArray> ("pr2_robot/pcl_clusters",1);

    #ifdef DEBUG // add in the publishers to look at the output through Rviz if debugging
    m_voxelPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_voxel",1);
    m_outliersPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_ouliers",1);
    m_passthroughPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_passthrough",1);
    m_ransacPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_ransac",1);
    m_passthrough2Pub = m_nh.advertise<sensor_msgs::PointCloud2> ("pr2_robot/pcl_passthrough2",1);
    #endif

  } // end constructor declaration

  void voxel_filter(); // voxel filter declaration
  void passthrough_filter

private:

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  ros::Publisher m_clusterPub;

  // define filter function parameters
  float m_y_passthrough_min;
  float m_y_passthrough_max;
  float m_z_passthrough_min;
  float m_z_passthrough_max;
  float m_voxel_leaf;
  float m_outlier_stdev;
  float m_outlier_meanK;
  float m_ransac_dist_thresh;
  float m_euclidean_cluster_tolerance;
  float m_euclidean_min_cluster_size;
  float m_euclidean_max_cluster_size;

  // declare the filter instances, these are initialized with the contructor
  pcl::VoxelGrid<pcl::PCLPointCloud2> m_voxelFilter;
  pcl::PassThrough<pcl::PointXYZRGB> m_passY;
  pcl::PassThrough<pcl::PointXYZRGB> m_passZ;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> m_outlierFilter;
  pcl::SACSegmentation<pcl::PointXYZRGB> m_ransac_segmentation; // segmentation class for RANSAC
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr m_tree; //Ptr to kd tree for Euclidean segmentaiton


  // declare the pcl and ROS point clouds used in filtering functions
  pcl::PCLPointCloud2* m_incoming_cloud; // holds the incoming cloud in pcl::PointCloud2 format
  pcl::PCLPointCloud2ConstPtr m_incomingCloudPtr; // shared ptr to the incoming cloud in pcl::PointCloud2 format
  pcl::PointCloud<pcl::PointXYZRGB> *m_xyz_cloud; // cloud to hold point cloud in pcl::PointCloud<Pcl::PointXYZRGB> format
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_xyzCloudPtr;
  pcl::ModelCoefficients::Ptr m_coefficients; // Ptr to the coefficients for the ransac filtering
  pcl::PointIndices::Ptr m_inliers; // Ptr to PointIndices object to hold results from RANSAC filtering
  std::vector<pcl::PointIndices> m_cluster_indices; // indices for the
  pcl::ExtractIndices<pcl::PointXYZRGB> m_extract; // extraction object for RANSAC
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> m_ec; // object to extract indices from point cloud
  sensor_msgs::PointCloud2 m_output; // holds the ouput cloud in sensor_msgs::PointCloud2 format
  pcl::PCLPointCloud2 m_outputPCL; // holds the output cloud in pcl::PointCloud2 format

  pr2_robot::SegmentedClustersArray m_CloudClusters; // the output message

  #ifdef DEBUG // add the extra publishers if in debug mode

  ros::Publisher m_voxelPub;
  ros::Publisher m_outliersPub;
  ros::Publisher m_passthroughZPub;
  ros::Publisher m_ransacPub;
  ros::Publisher m_passthroughYPub;

  #endif

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{



  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);



  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
  voxelFilter.setInputCloud (cloudPtr);
  voxelFilter.setLeafSize (0.01, 0.01, 0.01);
  voxelFilter.filter (*cloudFilteredPtr);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl_conversions::fromPCL(*cloudFilteredPtr, output);
  m_voxelPub.publish(output);
  #endif


  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results in the y dir
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-.5, .5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthroughPub.publish(output);
  #endif

  // create a pcl object to hold the passthrough filtered results in the z dir
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered2 = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered2 (xyz_cloud_filtered2);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass2;
  pass2.setInputCloud (xyzCloudPtrFiltered);
  pass2.setFilterFieldName ("z");
  pass2.setFilterLimits (.5, 1.1);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (*xyzCloudPtrFiltered2);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFiltered2 ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthrough2Pub.publish(output);
  #endif

  // create a pcl object to hold the outlier filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_outlier_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrOutlierFiltered (xyz_cloud_outlier_filtered);



  // Create the statistical outlier filter filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierFilter;
  outlierFilter.setInputCloud (xyzCloudPtrFiltered2);
  outlierFilter.setMeanK (50);
  outlierFilter.setStddevMulThresh (.0075);
  outlierFilter.filter (*xyzCloudPtrOutlierFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrOutlierFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_outliersPub.publish(output);
  #endif

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.016);

  seg1.setInputCloud (xyzCloudPtrOutlierFiltered);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrOutlierFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_ransacPub.publish(output);
  #endif



  // perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  pr2_robot::SegmentedClustersArray CloudClusters;



  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {


    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);

    }


    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    // add the cluster to the array message
    //clusterData.cluster = output;
    CloudClusters.clusters.push_back(output);

  }

  // publish the clusters
  m_clusterPub.publish(CloudClusters);

}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;

  segmentation segs(nh);

  while(ros::ok())
  ros::spin ();

}
