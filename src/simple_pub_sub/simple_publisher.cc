/**
 * @file simple_publisher.cc
 * @author Shisato Yano
 * @brief ROS1 publisher node sample
 * @version 0.1
 * @date 2021-10-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @brief Publisher main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_publisher");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Hello world!!";
    ROS_INFO("publish: %s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}