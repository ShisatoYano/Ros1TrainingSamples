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
  ros::ServiceClient sample_service_client = nh.serviceClient<Ros1TrainingSamples::SrvSample>("sample_srv");

  // declare srv object
  Ros1TrainingSamples::SrvSample srv;

  // set input value by keyboard to service request message
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  // request service
  // when request was accepted, output returned response value
  if (sample_service_client.call(srv))
  {
    ROS_INFO("send srv, srv.Request.a and b: %ld, %ld",
             (long int)srv.request.a, (long int)srv.request.b);
    ROS_INFO("receive srv, srv.Response.result: %ld",
             (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service sample_srv");
    return 1;
  }

  return 0;
}