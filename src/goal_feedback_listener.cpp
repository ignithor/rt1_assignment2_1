#include "ros/ros.h"
#include "assignment_2_2024/PlanningActionFeedback.h"

// Callback function to process feedback
void feedbackCallback(const bool& msg) {
    if (msg.success) {
        ROS_INFO("Reached");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_feedback_listener");
    ros::NodeHandle nh;

    // Subscribe to the feedback topic
    ros::Subscriber sub = nh.subscribe("/go_to_point_switch", 1000, feedbackCallback);

    ROS_INFO("Listening for goal feedback...");
    ros::spin();

    return 0;
}
