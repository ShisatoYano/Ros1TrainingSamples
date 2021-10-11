/**
 * @file message_publisher.cc
 * @author Shisato Yano
 * @brief Message publisher node sample
 * @version 0.1
 * @date 2021-10-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "Ros1TrainingSamples/MsgSample.h"

/**
 * @brief Main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "message_publisher");
  
  ros::NodeHandle nh;
  ros::Publisher msg_sample_pub = nh.advertise<Ros1TrainingSamples::MsgSample>("sample_message", 100);

  ros::Rate loop_rate(10); // [Hz]

  Ros1TrainingSamples::MsgSample msg;

  int count = 0;

  while (ros::ok())
  {
    msg.stamp = ros::Time::now();
    msg.data = count;

    ROS_INFO("send msg = %d", msg.stamp.sec);
    ROS_INFO("send msg = %d", msg.stamp.nsec);
    ROS_INFO("send msg = %d", msg.data);

    msg_sample_pub.publish(msg);

    loop_rate.sleep();

    count++;
  }

  return 0;
}