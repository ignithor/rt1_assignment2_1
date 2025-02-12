#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include <algorithm>

// Publisher for the warning topic
ros::Publisher warning_pub;

// Callback function for the LaserScan topic
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Find the minimum distance from the scan data
    double min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());

    // Check if an obstacle is too close (less than 1 meter)
    std_msgs::Bool warning_msg;
    warning_msg.data = (min_distance < 1.0);

    // Publish the warning message
    warning_pub.publish(warning_msg);

    // Log a warning if an obstacle is too close
    if (warning_msg.data) {
        ROS_WARN("Obstacle too close! Distance: %.2f meters", min_distance);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_warning_publisher");
    ros::NodeHandle nh;

    // Create the publisher for the warning topic
    warning_pub = nh.advertise<std_msgs::Bool>("/warning", 10);

    // Subscribe to the /scan topic
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scanCallback);

    ROS_INFO("Obstacle Warning Node Started...");
    ros::spin();

    return 0;
}
