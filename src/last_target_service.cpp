/**
 * \file last_target_service.cpp
 * \brief Provides a service to retrieve the last target pose received by the robot.
 * \author Paul Pham Dang
 * \version 1.0
 * \date 11/03/2025
 * 
 * \details
 * 
 * Subscribes to : <BR>
 *  °/reaching_goal/goal
 * 
 * Services : <BR>
 *  °/get_last_target
 * 
 * Description : 
 * 
 * This node provides a service to retrieve the last target pose received by the robot.
 **/

 #include "ros/ros.h"
 #include "geometry_msgs/PoseStamped.h"
 #include "assignment_2_2024/PlanningActionGoal.h"
 #include "rt1_assignment2_1/Target.h"
 
 geometry_msgs::PoseStamped last_target;  ///< Global variable to store the last goal
 
 /**
  * \brief Callback function to update the last goal received.
  * \param msg A pointer to the received PlanningActionGoal message.
  * 
  * \return void
  * 
  * This function is called whenever a new goal is published on the `/reaching_goal/goal` topic.
  * It updates the global variable `last_target` with the received goal.
  */
 void goalCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr& msg) {
     last_target.header = msg->goal.target_pose.header;
     last_target.pose = msg->goal.target_pose.pose;
 }
 
 /**
  * \brief Service callback to provide the last goal.
  * 
  * \param req The service request (unused in this case).
  * \param res The service response containing the last target pose.
  * 
  * \return Always returns true, indicating a successful service call.
  * 
  * This function is called when the `/get_last_target` service is requested.
  * It fills the response with the last target pose stored in the global variable `last_target`.
  */
 bool getLastTarget(rt1_assignment2_1::Target::Request &req, rt1_assignment2_1::Target::Response &res) {
     res.target_pose.header = last_target.header;
     res.target_pose.pose = last_target.pose;
     return true;
 }
 
 /**
  * \brief Main function for the last_target_service node.
  * 
  * \param argc Number of command-line arguments.
  * \param argv Array of command-line arguments.
  * 
  * \return 0 on successful completion.
  * 
  * Initializes the ROS node, subscribes to the `/reaching_goal/goal` topic, and advertises the `/get_last_target` service.
  */
 int main(int argc, char **argv) {
     ros::init(argc, argv, "last_target_service"); /// Initialize the ROS node
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe("/reaching_goal/goal", 1000, goalCallback);      /// Subscribe to the action goal topic
     ros::ServiceServer service = nh.advertiseService("/get_last_target", getLastTarget);      /// Advertise the service
     ROS_INFO("LastTargetService is ready.");

     ros::spin();
 
     return 0;
 }