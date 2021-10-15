/**
 * @file action_server.cc
 * @author shisato yano
 * @brief action server node sample
 * @version 0.1
 * @date 2021-10-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "Ros1TrainingSamples/FibonacciAction.h"

/**
 * @brief action server class
 * 
 */
class FibonacciAction
{
protected:
  // node handle
  ros::NodeHandle nh;

  // action server
  actionlib::SimpleActionServer<Ros1TrainingSamples::FibonacciAction> as_;

  // action name variable
  std::string action_name_;

  // action feedback and action result to publish
  Ros1TrainingSamples::FibonacciFeedback feedback_;
  Ros1TrainingSamples::FibonacciResult result_;
public:
};

/**
 * @brief main process
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  return 0;
}