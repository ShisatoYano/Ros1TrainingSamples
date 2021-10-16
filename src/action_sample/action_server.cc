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
  ros::NodeHandle nh_;

  // action server
  actionlib::SimpleActionServer<Ros1TrainingSamples::FibonacciAction> as_;

  // action name variable
  std::string action_name_;

  // action feedback and action result to publish
  Ros1TrainingSamples::FibonacciFeedback feedback_;
  Ros1TrainingSamples::FibonacciResult result_;
public:
  /**
   * @brief Construct a new Fibonacci Action object
   * 
   * @param name action name
   */
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::execute_cb, this, _1),
        false),
    action_name_(name)
    {
      as_.start();
    }
  
  /**
   * @brief Destroy the Fibonacci Action object
   * 
   */
  ~FibonacciAction(void)
  {}

  /**
   * @brief execute specified action when message was received
   * action callback function to calculate fibonacci sequence
   * @param goal 
   */
  void execute_cb(const Ros1TrainingSamples::FibonacciGoalConstPtr &goal)
  {
    ros::Rate r(1); // loop rate: 1Hz
    bool success = true; // succeeded or failed action

    // initialize fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0); // add 1st number
    feedback_.sequence.push_back(1); // add 2nd number

    // output action name, goal, 1st/2nd numbers in fibonacci sequence
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
      action_name_.c_str(), goal->order, feedback_.sequence[0],
      feedback_.sequence[1]);
    
    // action
    for (int i=1; i<=goal->order; i++)
    {
      // check action was canceled by client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str()); // inform action was canceled
        as_.setPreempted(); // cancel action
        success = false;
        break;
      }

      // cancel action or before goal was achieved,
      // record value which current number + last number in feedback
      feedback_.sequence.push_back(
        feedback_.sequence[i] + feedback_.sequence[i-1]);
      as_.publishFeedback(feedback_); // publish feedback

      r.sleep();
    }

    // transfer current value of fibonacci sequence
    if (success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }
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
  ros::init(argc, argv, "action_server");

  FibonacciAction fibonacci("action_sample");

  ros::spin(); // wait until action goal was received

  return 0;
}