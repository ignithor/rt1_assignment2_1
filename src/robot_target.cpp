/**
 * \file robot_target.cpp
 * \brief A ROS node to control the robot's target position and handle user input.
 * \author Paul Pham Dang
 * \version 1.0
 * \date 11/03/2025
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *  °/odom
 * 
 * Publishes to: <BR>
 *  °/odom_simplified
 * 
 * Description:
 * 
 * This node allows the user to set a target position for the robot or cancel the current goal.
 * It subscribes to the `/odom` topic to get the robot's current position and publishes a simplified version
 * of the odometry data to `/odom_simplified`. It also provides an interactive command-line interface for
 * setting or canceling goals.
 */

 #include <ros/ros.h>
 #include <nav_msgs/Odometry.h>
 #include <actionlib/client/simple_action_client.h>
 #include <actionlib/client/terminal_state.h>
 #include "rt1_assignment2_1/Odom_simplified.h"
 #include "assignment_2_2024/PlanningAction.h"
 #include <thread>
 #include <iostream>
 
 // Define the type for the Action Client
 typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningActionClient;
 
 /**
  * \class RobotTarget
  * \brief A class to handle robot target operations.
  * 
  * This class provides functionality to set and cancel robot targets, subscribe to odometry data,
  * and publish simplified odometry information. It also includes a thread to handle user input.
  */
 class RobotTarget
 {
 public:
     /**
      * \brief Constructor for the RobotTarget class.
      * 
      * Initializes the ROS node, subscribes to the `/odom` topic, advertises the `/odom_simplified` topic,
      * and starts a thread to handle user input.
      */
     RobotTarget()
         : ac_("reaching_goal", true), stop_thread_(false)
     {
         // Initialize the subscriber and publisher
         odom_sub_ = nh_.subscribe("/odom", 10, &RobotTarget::odomCallback, this);
         odom_simplified_pub_ = nh_.advertise<rt1_assignment2_1::Odom_simplified>("/odom_simplified", 10);
 
         // Wait for the action server to start
         ROS_INFO("Waiting for action server to start...");
         ac_.waitForServer();
         ROS_INFO("Action server started.");
 
         // Start the input handling thread
         input_thread_ = std::thread(&RobotTarget::handleUserInput, this);
     }
 
     /**
      * \brief Destructor for the RobotTarget class.
      * 
      * Ensures the input thread is properly stopped and joined before the object is destroyed.
      */
     ~RobotTarget()
     {
         stop_thread_ = true; // Signal the input thread to stop
         if (input_thread_.joinable())
         {
             input_thread_.join();
         }
     }
 
     /**
      * \brief Sets a new target position for the robot.
      * 
      * \param x The x-coordinate of the target position.
      * \param y The y-coordinate of the target position.
      * 
      * This function sends a new goal to the action server with the specified coordinates.
      */
     void setTarget(double x, double y)
     {
         assignment_2_2024::PlanningGoal goal;
         goal.target_pose.header.stamp = ros::Time::now();
         goal.target_pose.header.frame_id = "odom";
         goal.target_pose.pose.position.x = x;
         goal.target_pose.pose.position.y = y;
 
         ROS_INFO("Sending goal: x=%f, y=%f", x, y);
         ac_.sendGoal(goal,
                      boost::bind(&RobotTarget::doneCb, this, _1)); // Done callback
     }
 
     /**
      * \brief Cancels the current target goal.
      * 
      * This function cancels the current goal being pursued by the robot.
      */
     void cancelTarget()
     {
         ROS_INFO("Cancelling goal");
         ac_.cancelGoal(); // Use the member action client
     }
 
 private:
     /**
      * \brief Callback function for the `/odom` topic.
      * 
      * \param msg A pointer to the received Odometry message.
      * 
      * This function processes the odometry data, simplifies it, and publishes it to `/odom_simplified`.
      */
     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
     {
         // Create a simplified message
         rt1_assignment2_1::Odom_simplified simplified_msg;
         simplified_msg.x = msg->pose.pose.position.x;
         simplified_msg.y = msg->pose.pose.position.y;
         simplified_msg.vel_x = msg->twist.twist.linear.x;
         simplified_msg.vel_z = msg->twist.twist.angular.z;
 
         // Publish the simplified message
         odom_simplified_pub_.publish(simplified_msg);
     }
 
     /**
      * \brief Callback function for when a goal is completed.
      * 
      * \param state The state of the goal (e.g., SUCCEEDED, ABORTED).
      * 
      * This function is called when the action server completes a goal. It logs the result.
      */
     void doneCb(const actionlib::SimpleClientGoalState& state)
     {
         if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
         {
             ROS_INFO("Goal reached!");
         }
         else
         {
             ROS_WARN("Goal did not succeed: %s", state.toString().c_str());
         }
         ROS_INFO("Enter command (set x y / cancel / end):");
     }
 
     /**
      * \brief Handles user input from the command line.
      * 
      * This function runs in a separate thread and processes user commands to set or cancel goals,
      * or to shut down the node.
      */
     void handleUserInput()
     {
         while (ros::ok() && !stop_thread_)
         {
             ROS_INFO("Enter command (set x y / cancel / end):");
             std::string command;
             std::cin >> command;
 
             if (command == "set")
             {
                 double x, y;
                 std::cin >> x >> y;
                 setTarget(x, y);
             }
             else if (command == "cancel")
             {
                 cancelTarget();
             }
             else if (command == "end")
             {
                 ros::shutdown();
             }
             else
             {
                 std::cout << "Unknown command" << std::endl;
             }
         }
     }
 
     ros::NodeHandle nh_; ///< ROS node handle
     ros::Subscriber odom_sub_; ///< Subscriber for the `/odom` topic
     ros::Publisher odom_simplified_pub_; ///< Publisher for the `/odom_simplified` topic
     PlanningActionClient ac_; ///< Action client for the `reaching_goal` action
     std::thread input_thread_; ///< Thread to handle user input
     bool stop_thread_; ///< Flag to stop the input thread
 };
 
 /**
  * \brief Main function for the robot_target node.
  * 
  * \param argc Number of command-line arguments.
  * \param argv Array of command-line arguments.
  * 
  * \return 0 on successful completion.
  * 
  * Initializes the ROS node and starts the RobotTarget object.
  */
 int main(int argc, char **argv)
 {
     ros::init(argc, argv, "robot_target");
     RobotTarget robot_target;
     ros::spin();
     return 0;
 }