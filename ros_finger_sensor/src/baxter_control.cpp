/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple position control for baxter
 * Created   : 2016 - 04 - 21
 */

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

namespace ros_finger_sensor
{

class BaxterControlTest
{
private:
  ros::NodeHandle nh_;
  ros::Publisher planning_state_pub_;
  unsigned long int seq_;

public:
  //Constructor
  BaxterControlTest(int test)
  {
    ROS_DEBUG_STREAM_NAMED("constructor","starting test " << test);

    // Set up subs/pubs
    seq_ = 0;
    planning_state_pub_ = nh_.advertise<sensor_msgs::JointState>("my_states", 20);

    while (ros::ok())
    {
      publishPlanningState();
    }
  }

  void publishPlanningState()
  {
    ROS_DEBUG_STREAM_NAMED("publishPlanningState","populating message " << seq_);
    sensor_msgs::JointState planning_msg;
    planning_msg.header.seq = seq_;
    seq_++;
    planning_msg.header.stamp = ros::Time::now();

    planning_msg.name.push_back("head_pan");
    planning_msg.name.push_back("right_s0");
    planning_msg.name.push_back("right_s1");
    planning_msg.name.push_back("right_e0");
    planning_msg.name.push_back("right_e1");
    planning_msg.name.push_back("right_w0");
    planning_msg.name.push_back("right_w1");
    planning_msg.name.push_back("right_w2");
    planning_msg.name.push_back("left_s0");
    planning_msg.name.push_back("left_s1");
    planning_msg.name.push_back("left_e0");
    planning_msg.name.push_back("left_e1");
    planning_msg.name.push_back("left_w0");
    planning_msg.name.push_back("left_w1");
    planning_msg.name.push_back("left_w2");

    /*
      TODO:
      1. get pose of object from perception server
      2. generate simple path from "ready position" to object
      3. visualize for operator
      4. get ok from operator to execute
      5. publish commmand to /robot/limb/(right/left)/joint_command
     */

    for (int i = 0; i < 15; i++)
    {
      planning_msg.position.push_back(0.0);
      planning_msg.velocity.push_back(0.0);
      planning_msg.effort.push_back(0.0);
    }
    
    planning_state_pub_.publish(planning_msg);
  }

};

}

int main(int argc, char *argv[])
{
  ROS_INFO_STREAM_NAMED("main","Starting baxter control test for finger sensor...");
  ros::init(argc, argv, "baxter_control_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  int test = 1;
  ros_finger_sensor::BaxterControlTest tester(test);

}
