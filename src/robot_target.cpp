#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rt1_assignment2_1/Odom_simplified.h"

class RobotTarget
{
public:
    RobotTarget()
    {
        // Initialize the subscriber and publisher
        odom_sub_ = nh_.subscribe("/odom", 10, &RobotTarget::odomCallback, this);
        odom_simplified_pub_ = nh_.advertise<rt1_assignment2_1::Odom_simplified>("/odom_simplified", 10);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Create a simplified message
        rt1_assignment2_1::Odom_simplified simplified_msg;
        simplified_msg.x = msg->pose.pose.position.x;
        simplified_msg.y = msg->pose.pose.position.y;
        simplified_msg.vel_x = msg->twist.twist.linear.x;
        simplified_msg.vel_z = msg->twist.twist.linear.z;

        // Publish the simplified message
        odom_simplified_pub_.publish(simplified_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_simplified_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_target");
    RobotTarget robot_target;
    ros::spin();
    return 0;
}