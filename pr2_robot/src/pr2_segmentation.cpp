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
    #endif

    cloud = new pcl::PCLPointCloud2;
    cloud_filtered = new pcl::PCLPointCloud2;
    xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    xyz_cloud_filteredY = new pcl::PointCloud<pcl::PointXYZRGB>;
    xyz_cloud_filteredZ = new pcl::PointCloud<pcl::PointXYZRGB>;
    xyz_cloud_outlier_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;



    // get the shared pointers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrOutlierFiltered (xyz_cloud_outlier_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered(xyz_cloud_ransac_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFilteredZ (xyz_cloud_filteredZ);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFilteredY (xyz_cloud_filteredY);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


    // set voxel filter parameters
    voxelFilter.setLeafSize (0.01, 0.01, 0.01);
    // set passthrough filter parameters
    passY.setFilterFieldName ("y");
    passY.setFilterLimits (-.5, .5);
    // set z passthrough filter parameters
    passZ.setFilterFieldName ("z");
    passZ.setFilterLimits (.5, 1.1);
    // set outlier filter parameters
    outlierFilter.setMeanK (50);
    outlierFilter.setStddevMulThresh (.0075);
    // set ransac filter parameters
    seg1.setOptimizeCoefficients (true);
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.016);
    extract.setNegative (true);
    // specify euclidean cluster parameters
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);


  }

private:

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  ros::Publisher m_clusterPub;

  // declare the PCL data objects
  pcl::PCLPointCloud2* cloud; // // pcl object to hold the conversion from sensor_msgs::PointCloud2 data type
  pcl::PCLPointCloud2* cloud_filtered; // pcl object to hold the voxel filtered cloud
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud; // pcl object to hold the conversion from pcl::PointCloud2 data type
  sensor_msgs::PointCloud2 output; // output sensor_msgs
  pcl::PCLPointCloud2 outputPCL;  // output pcl
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filteredY; // pcl object to hold the passthrough filtered data in the y direction
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered; // pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filteredZ; // pcl object to hold the passthrough filtered data in the y direction
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_outlier_filtered; //pcl object to hold the outlier filtered results

  // Declare the filters
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter; // voxel filter
  pcl::PassThrough<pcl::PointXYZRGB> passY; // passthrough filter in the y dir
  pcl::PassThrough<pcl::PointXYZRGB> passZ; // pcl object to hold the passthrough filtered results in the z dir
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierFilter; // statistical outlier filter
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1; // ransac segmentation filter
  pcl::ExtractIndices<pcl::PointXYZRGB> extract; // extraction class for RANSAC segmentation
  std::vector<pcl::PointIndices> cluster_indices; // vector containing the segmented clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; // extraction object for the clusters

  // declare the shared pointers to the point cloud data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrOutlierFiltered;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFilteredZ;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFilteredY;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr;
  pcl::PCLPointCloud2ConstPtr cloudPtr;


  #ifdef DEBUG // add the extra publishers if in debug mode
  ros::Publisher m_voxelPub;
  ros::Publisher m_outliersPub;
  ros::Publisher m_passthroughPub;
  ros::Publisher m_ransacPub;
  ros::Publisher m_passthrough2Pub;

  #endif

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{



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
  passY.filter (*xyzCloudPtrFilteredY);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFilteredY ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthroughPub.publish(output);
  #endif

  // passthrough filter in the z dir
  passZ.setInputCloud (xyzCloudPtrFilteredY);
  passZ.filter (*xyzCloudPtrFilteredZ);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrFilteredZ ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_passthrough2Pub.publish(output);
  #endif

  // perform outlier filtering
  outlierFilter.setInputCloud (xyzCloudPtrFilteredZ);
  outlierFilter.filter (*xyzCloudPtrOutlierFiltered);

  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrOutlierFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_outliersPub.publish(output);
  #endif


  // perform RANSAC segmentation and extract outliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  seg1.setInputCloud (xyzCloudPtrOutlierFiltered);
  seg1.segment (*inliers, *coefficients);
  extract.setInputCloud (xyzCloudPtrOutlierFiltered);
  extract.setIndices (inliers);
  extract.filter (*xyzCloudPtrRansacFiltered);


  #ifdef DEBUG // publish the point cloud to RViz if in debug
  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,outputPCL);
  pcl_conversions::fromPCL(outputPCL, output);
  m_ransacPub.publish(output);
  #endif

  // perform euclidean cluster segmentation to seporate individual objects
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  pr2_robot::SegmentedClustersArray CloudClusters;

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
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);

    }


    // populate the output message
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL); // convert to pcl::PCLPointCloud2
    pcl_conversions::fromPCL(outputPCL, output); // Convert to ROS data type
    CloudClusters.clusters.push_back(output); // add the cluster to the array message

  }

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
