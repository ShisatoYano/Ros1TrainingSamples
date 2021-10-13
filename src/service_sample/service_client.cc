/**
 * @file service_client.cc
 * @author shisato yano
 * @brief service client node sample
 * @version 0.1
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <cstdlib>
#include <ros/ros.h>
#include "Ros1TrainingSamples/SrvSample.h"

/**
 * @brief main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client"); // initialize node

  if (argc != 3) // input error
  {
    ROS_INFO("cmd : rosrun Ros1TrainingSamples service_client arg0 arg1");
    ROS_INFO("arg0: double number, arg1: double number");
    return 1;
  }

  ros::NodeHandle nh; // declare node handle to communicate with ros system

  // declare service client

  return 0;
}