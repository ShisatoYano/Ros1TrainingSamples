/**
 * @file message_subscriber.cc
 * @author Shisato Yano
 * @brief Message subscriber node sample
 * @version 0.1
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "Ros1TrainingSamples/MsgSample.h"

/**
 * @brief Callback function to receive message
 * 
 * @param msg 
 */
void msg_callback(const Ros1TrainingSamples::MsgSample::ConstPtr& msg)
{
  ROS_INFO("receive msg = %d", msg->stamp.sec);

  ROS_INFO("receive msg = %d", msg->stamp.nsec);

  ROS_INFO("receive msg = %d", msg->data);
}

/**
 * @brief Main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "message_subscriber");

  ros::NodeHandle nh;
  ros::Subscriber msg_sample_sub = nh.subscribe("sample_message", 100, msg_callback);

  // wait for receiving message
  // when received, callback function is going to be executed
  ros::spin();

  return 0;
}

