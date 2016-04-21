/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple position control for baxter
 * Created   : 2016 - 04 - 21
 */

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

/*
  TODO:
  1. get pose of object from perception server
  2. generate simple path from "ready position" to object
  3. visualize for operator
  4. get ok from operator to execute
  5. publish commmand to /robot/limb/(right/left)/joint_command
*/

namespace ros_finger_sensor
{

class BaxterControlTest
{
private:
  ros::NodeHandle nh_;
  ros::Publisher planning_state_pub_;
  unsigned long int seq_;
  sensor_msgs::JointState planning_msg_;

  double joint_limits[15][2];

public:
  //Constructor
  BaxterControlTest(int test)
  {
    ROS_DEBUG_STREAM_NAMED("constructor","starting test " << test);
    
    // define limits
    joint_limits[0][0] = -1.57079632679;
    joint_limits[0][1] = 1.57079632679; // head_pan 
    joint_limits[1][0] = -1.70167993878; 
    joint_limits[1][1] = 1.7016799387; //right_s0
    joint_limits[2][0] = -2.147; 
    joint_limits[2][1] = 1.047; //right_s1
    joint_limits[3][0] = -3.05417993878; 
    joint_limits[3][1] = 3.05417993878; //right_e0
    joint_limits[4][0] = -0.05; 
    joint_limits[4][1] = 2.618; //right_e1
    joint_limits[5][0] = -3.059; 
    joint_limits[5][1] = 3.059; //right_w0
    joint_limits[6][0] = -1.57079632679; 
    joint_limits[6][1] = 2.094; //right_w1
    joint_limits[7][0] = -3.059; 
    joint_limits[7][1] = 3.059; //right_w2
    joint_limits[8][0] = -1.70167993878; 
    joint_limits[8][1] = 1.70167993878; //left_s0 
    joint_limits[9][0] = -2.147; 
    joint_limits[9][1] = 1.047; //left_s1 
    joint_limits[10][0] = -3.05417993878; 
    joint_limits[10][1] = 3.05417993878; //left_e0 
    joint_limits[11][0] = -0.05; 
    joint_limits[11][1] = 2.618; //left_e1 
    joint_limits[12][0] = -3.059; 
    joint_limits[12][1] = 3.059; //left_w0 
    joint_limits[13][0] = -1.57079632679; 
    joint_limits[13][1] = 2.094; //left_w1 
    joint_limits[14][0] = -3.059; 
    joint_limits[14][1] = 3.059; //left_w2 

    for (int i = 0; i < 15; i++)
    {
      planning_msg_.position.push_back(0.0);
      planning_msg_.velocity.push_back(0.0);
      planning_msg_.effort.push_back(0.0);
    }

    // setup planning message
    planning_msg_.name.push_back("head_pan");
    planning_msg_.name.push_back("right_s0");
    planning_msg_.name.push_back("right_s1");
    planning_msg_.name.push_back("right_e0");
    planning_msg_.name.push_back("right_e1");
    planning_msg_.name.push_back("right_w0");
    planning_msg_.name.push_back("right_w1");
    planning_msg_.name.push_back("right_w2");
    planning_msg_.name.push_back("left_s0");
    planning_msg_.name.push_back("left_s1");
    planning_msg_.name.push_back("left_e0");
    planning_msg_.name.push_back("left_e1");
    planning_msg_.name.push_back("left_w0");
    planning_msg_.name.push_back("left_w1");
    planning_msg_.name.push_back("left_w2");

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
    planning_msg_.header.seq = seq_;
    seq_++;
    planning_msg_.header.stamp = ros::Time::now();

    // TODO: replace
    maxMinLimits();
    
    planning_state_pub_.publish(planning_msg_);
    ros::Duration(0.2).sleep();
  }

  void maxMinLimits()
  {
    // just a simple loop going from min to max joint limits then resetting
    for (std::size_t i = 0; i < 15; i++)
    {
      double delta = (joint_limits[i][1] - joint_limits[i][0]) / 25.0;
      ROS_DEBUG_STREAM_NAMED("maxMinLimits","delta = " << delta);
      planning_msg_.position[i] += delta;
      if (planning_msg_.position[i] > joint_limits[i][1])
      {
        planning_msg_.position[i] = joint_limits[i][0];
      }
      ROS_DEBUG_STREAM_NAMED("maxMinLimits","position = " << i << ", value = " << planning_msg_.position[i]);
    }    
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
