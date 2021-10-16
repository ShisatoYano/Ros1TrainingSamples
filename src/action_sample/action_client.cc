/**
 * @file action_client.cc
 * @author shisato yano
 * @brief action client node sample
 * @version 0.1
 * @date 2021-10-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "Ros1TrainingSamples/FibonacciAction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_client");

  // declare action client
  actionlib::SimpleActionClient<Ros1TrainingSamples::FibonacciAction> ac("action_sample", true);

  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal");
  Ros1TrainingSamples::FibonacciGoal goal; // declare goal object
  goal.order = 20; // calculate fibonacci sequence until 20
  ac.sendGoal(goal);

  // set time limit: 30[s]
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  // action result was received within time limit
  if (finished_before_timeout)
  {
    // output state of goal
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out");
  }

  return 0;
}