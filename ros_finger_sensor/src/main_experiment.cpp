/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple pick and place for baxter and finger sensor
 * Created   : 2016 - 04 - 14
 */

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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
  Eigen::Affine3d new_pose_;
  Eigen::Affine3d table_pose_;

public:
  // Constructor
  FingerSensorTest(int test)
    : nh_("~")
  {
    std::cout << test << std::endl;
    ROS_INFO_STREAM_NAMED("constructor","test...");

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "visual_tools"));
    visual_tools_->deleteAllMarkers();

    // get tf for table
    ROS_INFO_STREAM_NAMED("constructor","waiting for table transform to be published...");
    tf_listener_.waitForTransform("/base", "/table", ros::Time(0), ros::Duration(5.0));
    tf_listener_.lookupTransform("/base", "/table", ros::Time(0), table_transform_);
    tf::transformTFToEigen(table_transform_, table_pose_);
    visual_tools_->publishCuboid(table_pose_, 0.6, 1.0, 0.4, rviz_visual_tools::ORANGE);

    while(ros::ok())
    {
      tf_listener_.lookupTransform("/base", "/table", ros::Time(0), table_transform_);
      tf::transformTFToEigen(table_transform_, new_pose_);

      // test if table pose has been updated & update visualization
      if ( !(std::abs((new_pose_.translation() - table_pose_.translation()).sum()) < 0.001 && 
             std::abs((new_pose_.rotation() - table_pose_.rotation()).sum()) < 0.001 ) )
        {
          std::cout << "translation = " << new_pose_.translation() - table_pose_.translation() << std::endl;
          std::cout << "rotation = \n" << new_pose_.rotation() - table_pose_.rotation() << std::endl;
          ROS_DEBUG_STREAM_NAMED("constructor","update cuboid position");
          table_pose_ = new_pose_;
          tf::transformTFToEigen(table_transform_, new_pose_);
          visual_tools_->deleteAllMarkers();
          visual_tools_->publishCuboid(table_pose_, 0.6, 1.0, 0.4, rviz_visual_tools::ORANGE);
        }
      
    }
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
