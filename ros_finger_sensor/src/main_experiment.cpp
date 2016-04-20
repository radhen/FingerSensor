/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple pick and place for baxter and finger sensor
 * Created   : 2016 - 04 - 14
 */

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>

#include <iostream>

namespace ros_finger_sensor
{

class FingerSensorTest
{
private:
  std::string qr_marker_;

  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform qr_transform_;

  Eigen::Affine3d roi_pose_;
  Eigen::Affine3d qr_pose_;

  double roi_depth_, roi_width_, roi_height_;
  double qr_offset_x_, qr_offset_y_, qr_offset_z_;
  double roi_padding_x_, roi_padding_y_, roi_padding_z_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
  ros::Publisher roi_cloud_pub_;
  ros::Subscriber raw_cloud_sub_;


public:
  // Constructor
  FingerSensorTest(int test)
    : nh_("~")
  {
    qr_marker_ = "ar_marker_0";

    std::cout << test << std::endl;
    ROS_INFO_STREAM_NAMED("constructor","test...");

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "visual_tools"));
    visual_tools_->deleteAllMarkers();


    // Setup to filter workspace
    // TODO: add these values to config file in case setup moves...
    roi_depth_ = 0.75;
    roi_width_ = 0.60;
    roi_height_ = 0.50;
    qr_offset_x_ = -0.05;
    qr_offset_y_ = -0.05;
    qr_offset_z_ = -0.10;
    roi_padding_x_ = 0.05;
    roi_padding_y_ = 0.05;
    roi_padding_z_ = 0.0;
    
    showRegionOfInterest();

    // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    roi_cloud_ = roi_cloud;
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud", 1);
    raw_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &FingerSensorTest::processPointCloud, this);


    while(ros::ok())
    {

    }

  }

  void showRegionOfInterest()
  {
    // get tf to qr code
    // TODO: ar_marker_# needs to go in config file. How are ids chosen?
    ROS_DEBUG_STREAM_NAMED("constructor","waiting for qr transform to be published...");
    tf_listener_.waitForTransform("/base", qr_marker_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform("/base", qr_marker_, ros::Time(0), qr_transform_);
    tf::transformTFToEigen(qr_transform_, qr_pose_);

    // add offsets and display region of interset
    Eigen::Affine3d pose_offset = Eigen::Affine3d::Identity();
    pose_offset.translation()[0] += roi_depth_ / 2.0 + qr_offset_x_;
    pose_offset.translation()[1] += roi_width_ / 2.0 + qr_offset_y_;
    pose_offset.translation()[2] += roi_height_ / 2.0 + qr_offset_z_; // roi_pose_ in qr coord. frame
    roi_pose_ = qr_pose_ * pose_offset; // roi_pose_ in base coord. frame
    
    visual_tools_->publishAxisLabeled(roi_pose_, "roi_pose");
    visual_tools_->publishWireframeCuboid(roi_pose_, roi_depth_, roi_width_, roi_height_, rviz_visual_tools::CYAN);
    visual_tools_->publishWireframeCuboid(roi_pose_, 
                                          roi_depth_ - 2 * roi_padding_x_, 
                                          roi_width_ - 2 * roi_padding_y_, 
                                          roi_height_ - 2 * roi_padding_z_, 
                                          rviz_visual_tools::MAGENTA);
  }

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // get point cloud in qr_marker_ coordinate frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    tf_listener_.waitForTransform(qr_marker_, raw_cloud->header.frame_id, msg->header.stamp, ros::Duration(2.0));
    if (!pcl_ros::transformPointCloud(qr_marker_, *raw_cloud, *roi_cloud_, tf_listener_))
    {
      ROS_ERROR_STREAM_NAMED("processPointCloud","Error converting to desired frame");
    }
    
    segmentRegionOfInterest();



    roi_cloud_pub_.publish(roi_cloud_);

  }

  void segmentRegionOfInterest()
  {   
    // Filter based on bin location
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(roi_cloud_);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(qr_offset_x_ + roi_padding_x_, roi_depth_ + qr_offset_x_ - roi_padding_x_);
    pass_x.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(roi_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(qr_offset_y_ + roi_padding_y_, roi_width_ + qr_offset_y_ - roi_padding_y_);
    pass_y.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(roi_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(qr_offset_z_ + roi_padding_z_, roi_height_ + qr_offset_z_ - roi_padding_z_);
    pass_z.filter(*roi_cloud_);
  }

};

}

int main(int argc, char *argv[])
{
  ROS_INFO_STREAM_NAMED("main","Starting finger sensor pick & place demo");
  ros::init(argc, argv, "finger_sensor_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  ros_finger_sensor::FingerSensorTest tester(test);

  return 0;
}
