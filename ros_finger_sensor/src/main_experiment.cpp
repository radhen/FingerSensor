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
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform table_transform_;
  tf::StampedTransform qr_transform_;
  Eigen::Affine3d new_pose_;
  Eigen::Affine3d table_pose_;
  Eigen::Affine3d qr_pose_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
  ros::Publisher roi_cloud_pub_;
  ros::Subscriber raw_cloud_sub_;


public:
  // Constructor
  FingerSensorTest(int test)
    : nh_("~")
  {

    //bool processing_ = false;

    std::cout << test << std::endl;
    ROS_INFO_STREAM_NAMED("constructor","test...");

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "visual_tools"));
    visual_tools_->deleteAllMarkers();

    // get tf for table
    ROS_INFO_STREAM_NAMED("constructor","waiting for table transform to be published...");
    tf_listener_.waitForTransform("/base", "/table", ros::Time(0), ros::Duration(5.0));
    tf_listener_.lookupTransform("/base", "/table", ros::Time(0), table_transform_);
    tf::transformTFToEigen(table_transform_, table_pose_);
    visual_tools_->publishCuboid(table_pose_, 0.6, 0.8, 0.72, rviz_visual_tools::BLUE);

    // get tf to qr code
    ROS_DEBUG_STREAM_NAMED("constructor","waiting for qr transform to be published...");
    tf_listener_.waitForTransform("/base", "/ar_marker_0", ros::Time(0), ros::Duration(5.0));
    tf_listener_.lookupTransform("/base", "/ar_marker_0", ros::Time(0), qr_transform_);
    tf::transformTFToEigen(qr_transform_, qr_pose_);
    std::cout << qr_pose_.rotation() << std::endl;
    std::cout << qr_pose_.translation() << std::endl;


    // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    roi_cloud_ = roi_cloud;
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud", 1);
    raw_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &FingerSensorTest::processPointCloud, this);


    while(ros::ok())
    {
      // updateTableTransform();


    }

  }

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // get point cloud in /base coordinate frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    static const std::string BASE_LINK = "/base";
    tf_listener_.waitForTransform(BASE_LINK, raw_cloud->header.frame_id, msg->header.stamp, ros::Duration(2.0));

    if (!pcl_ros::transformPointCloud(BASE_LINK, *raw_cloud, *roi_cloud_, tf_listener_))
    {
      ROS_ERROR_STREAM_NAMED("processPointCloud","Error converting to desired frame");
    }


    // set regoin of interest
    double raw_depth_ = 15;
    double raw_width_ = 8;
    double raw_height_ = 8;
    Eigen::Affine3d raw_pose_ = Eigen::Affine3d::Identity();
    //raw_pose_.translation() += Eigen::Vector3d( 0.687 + raw_depth_ / 2.0,
                                                  //-0.438 + raw_width_ / 2.0,
                                                //   0.002 + raw_height_ / 2.0);

    // Filter based on bin location
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(raw_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(raw_pose_.translation()[0]-raw_depth_ / 2.0, raw_pose_.translation()[0] + raw_depth_ / 2.0);
    pass_x.filter(*raw_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(raw_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(raw_pose_.translation()[1] - raw_width_ / 2.0, raw_pose_.translation()[1] + raw_width_ / 2.0);
    pass_y.filter(*raw_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(raw_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(raw_pose_.translation()[2] - raw_height_ / 2.0, raw_pose_.translation()[2] + raw_height_ / 2.0);
    pass_z.filter(*raw_cloud);
    //ROS_DEBUG_STREAM_NAMED("processPointCloud","width = " << raw_cloud->width << ", height = " << raw_cloud->height);

    roi_cloud_pub_.publish(raw_cloud);
    ROS_DEBUG_STREAM_THROTTLE_NAMED(2, "point_cloud_filter","Publishing filtered point cloud");

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
