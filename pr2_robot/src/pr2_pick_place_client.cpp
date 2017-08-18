/*
ROS node for generating pick place server request from detected objects message and yaml script
Author: Sean Cassero
8/16/15
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
#include <pr2_robot/DetectedObjectsArray.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#define DEBUG

class pick_place_client {

public:

  explicit pick_place_client(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher
    m_sub = m_nh.subscribe ("'/detected_objects'", 1, &pick_place_client::cloud_cb, this);
    //m_clusterPub = m_nh.advertise<pr2_robot::SegmentedClustersArray> ("pr2_robot/pcl_clusters",1);




  }

private:

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;
  //ros::Publisher m_clusterPub;

  void cloud_cb(const pr2_robot::DetectedObjectsArray& detected_objects);

}; // end class definition


// define callback function
void pick_place_client::cloud_cb (const pr2_robot::DetectedObjectsArray& detected_objects)
{


}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pick_place_client");
  ros::NodeHandle nh;

  // get the segmentaiton object
  pick_place_client pp_client(nh);

  while(ros::ok())
  ros::spin ();

}
