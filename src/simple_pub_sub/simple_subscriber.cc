/**
 * @file simple_subscriber.cc
 * @author Shisato Yano
 * @brief ROS1 subscriber node sample
 * @version 0.1
 * @date 2021-10-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief Callback to subscribe message from publisher
 * 
 * @param msg published string message
 */
void chatterCallback(const std_msgs::String& msg)
{
  ROS_INFO("subscribe: %s", msg.data.c_str());
}

/**
 * @brief Subscriber main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_simple_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("chatter", 10, chatterCallback);

  ros::spin();
  return 0;
}