#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "assignment_2_2024/PlanningActionGoal.h"
#include "rt1_assignment2_1/Distance.h"
#include <cmath>

// Global variables to store the last goal and current position
geometry_msgs::PoseStamped last_target;
geometry_msgs::Point current_position;
bool goal_received = false;  // Flag to check if a goal has been received

// Callback to update the last goal received
void goalCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr& msg) {
    last_target.header = msg->goal.target_pose.header;
    last_target.pose = msg->goal.target_pose.pose;
    goal_received = true;
}

// Callback to update the robot's current position
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = msg->pose.pose.position;
}

// Service callback to provide the distance to the last target
bool getDistance(rt1_assignment2_1::Distance::Request &req, rt1_assignment2_1::Distance::Response &res) {
    if (!goal_received) {
        res.distance = 0.0;
        ROS_WARN("No goal has been set yet. Returning distance 0.");
    } else {
        double dx = last_target.pose.position.x - current_position.x;
        double dy = last_target.pose.position.y - current_position.y;
        double dz = last_target.pose.position.z - current_position.z;
        res.distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return true;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "distance_to_target_service");
    ros::NodeHandle nh;

    // Subscribe to the action goal topic
    ros::Subscriber sub_goal = nh.subscribe("/reaching_goal/goal", 1000, goalCallback);
    
    // Subscribe to the odometry topic
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1000, odomCallback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("/get_distance_to_target", getDistance);

    ROS_INFO("DistanceToTargetService is ready.");
    ros::spin();

    return 0;
}
