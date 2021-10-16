/**
 * @file service_param_server.cc
 * @author shisato yano
 * @brief service server with parameter node sample
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "Ros1TrainingSamples/ParamSample.h"

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4

int g_operator = PLUS;

/**
 * @brief calculate value as response
 * 
 * @param req service request
 * @param res response to service
 * @return true
 */
bool calculate(Ros1TrainingSamples::ParamSample::Request &req,
               Ros1TrainingSamples::ParamSample::Response &res)
{
  // calculate a and b depend on parameter when received service request
  // the result is used as message for response
  switch(g_operator)
  {
    case PLUS:
      res.result = req.a + req.b; break;
    case MINUS:
      res.result = req.a - req.b; break;
    case MULTIPLICATION:
      res.result = req.a * req.b; break;
    case DIVISION:
      if (req.b == 0)
      {
        res.result = 0; break;
      }
      else
      {
        res.result = req.a / req.b; break;
      }
    default:
      res.result = req.a + req.b; break;
  }

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
  ros::init(argc, argv, "service_param_server"); // initialize node

  ros::NodeHandle nh; // declare node handle to communicate with ros system
  nh.setParam("calculation_method", PLUS); // initialize parameter

  // declare service server
  // execute calculate function when received request from client
  ros::ServiceServer sample_service_server = nh.advertiseService("sample_param_srv", calculate);

  ROS_INFO("ready srv server!");

  ros::Rate r(10); // 10[Hz]

  while (1)
  {
    // switch calculation method depend on parameter
    nh.getParam("calculation_method", g_operator);
    ros::spinOnce(); // process for callback function
    r.sleep();
  }

  return 0;
}
