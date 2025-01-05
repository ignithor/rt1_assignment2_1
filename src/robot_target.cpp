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

class RobotTarget
{
public:
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

    // Destructor
    ~RobotTarget()
    {
        stop_thread_ = true; // Signal the input thread to stop
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

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

    void cancelTarget()
    {
        ROS_INFO("Cancelling goal");
        ac_.cancelGoal(); // Use the member action client
    }

private:
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
        std::cout << "Enter command (set x y / cancel / end): \n";
    }

    void handleUserInput()
    {
        while (ros::ok() && !stop_thread_)
        {
            std::cout << "Enter command (set x y / cancel / end): \n";
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

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_simplified_pub_;
    PlanningActionClient ac_;
    std::thread input_thread_;
    bool stop_thread_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_target");
    RobotTarget robot_target;
    ros::spin();
    return 0;
}
