#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "assignment_2_2024/PlanningActionGoal.h"
#include "rt1_assignment2_1/Target.h"

// Global variable to store the last goal
geometry_msgs::PoseStamped last_target;

// Callback to update the last goal received
void goalCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr& msg) {
    last_target.header = msg->goal.target_pose.header;
    last_target.pose = msg->goal.target_pose.pose;
}

// Service callback to provide the last goal
bool getLastTarget(rt1_assignment2_1::Target::Request &req, rt1_assignment2_1::Target::Response &res) {
    res.target_pose.header = last_target.header;
    res.target_pose.pose = last_target.pose;
    return true;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "last_target_service");
    ros::NodeHandle nh;

    // Subscribe to the action goal topic
    ros::Subscriber sub = nh.subscribe("/reaching_goal/goal", 1000, goalCallback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("/get_last_target", getLastTarget);

    ROS_INFO("LastTargetService is ready.");
    ros::spin();

    return 0;
}
