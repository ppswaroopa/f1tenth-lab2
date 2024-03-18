#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        car_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 1, std::bind(&Safety::drive_callback, this, std::placeholders::_1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",1);
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        auto ranges = scan_msg->ranges;
        float incr_ang = scan_msg->angle_increment;
        // float min_ang = scan_msg->angle_min;
        // float max_ang = scan_msg
        int t = 1;
        for (int i = 0; i < ranges.size(); i++)
        {
            ranges[i] = ranges[i] * ( t * scan_msg->angle_increment );
            if (t<=1080)
            {
                t++;
            }
            else
                break;
        }
        
        /// TODO: publish drive/brake message
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_odom_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}