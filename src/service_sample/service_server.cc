/**
 * @file service_server.cc
 * @author shisato yano
 * @brief service server node sample
 * @version 0.1
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "Ros1TrainingSamples/SrvSample.h"

/**
 * @brief calculate value as response
 * 
 * @param req service request
 * @param res response to service
 * @return true
 */
bool calculate(Ros1TrainingSamples::SrvSample::Request &req,
               Ros1TrainingSamples::SrvSample::Response &res)
{
  // calculate a + b when received service request
  // the result is used as message for response
  res.result = req.a + req.b;

  // output a, b and result
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: %ld", (long int)res.result);

  return true;
}

/**
 * @brief main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_server"); // initialize node

  ros::NodeHandle nh; // declare node handle to communicate with ros system

  // declare service server
  // execute calculate function when received request from client
  ros::ServiceServer sample_service_server = nh.advertiseService("sample_srv", calculate);

  ROS_INFO("ready srv server!");

  ros::spin(); // wait for request

  return 0;
}
