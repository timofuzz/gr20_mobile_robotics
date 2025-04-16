#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class BaselinkTf : public rclcpp::Node {
public:
    BaselinkTf() : Node("baselink_tf") {
        this->declare_parameter<std::string>("topic_name", "/ego_vel_twist");
        std::string topic_name = this->get_parameter("topic_name").as_string();

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            topic_name, 10, std::bind(&BaselinkTf::twistCallback, this, std::placeholders::_1)
        );

        // Initialize orientation only (position will be set at first message)
        orientation_.x = 0.0;
        orientation_.y = 0.0;
        orientation_.z = 0.0;
        orientation_.w = 1.0;
        
        // Initialize yaw angle (for tracking rotation)
        yaw_ = 0.0;
        
        // DON'T initialize last_time_ here! Wait for first message
    }

private:
    bool first_msg_ = true;
    double yaw_ = 0.0;  // Track yaw angle for integration

    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;

        if (first_msg_) {
            // Initialize position and time at first message
            position_.x = 0.0;
            position_.y = 0.0;
            position_.z = 0.0;
            last_time_ = current_time;
            first_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "First message received, initializing odometry");
        } else {
            double dt = (current_time - last_time_).seconds();
            
            // Sanity check for large time gaps
            if (dt > 1.0) {
                RCLCPP_WARN(this->get_logger(), "Large time gap detected: %f seconds", dt);
                dt = 0.1; // Limit integration step
            }
            
            // Integrate angular velocity to update orientation
            yaw_ += msg->twist.angular.z * dt;
            
            // Convert yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_);
            q.normalize();
            
            // Update orientation
            tf2::convert(q, orientation_);
            
            // Transform velocity from body frame to world frame
            double vx_world = msg->twist.linear.x * cos(yaw_) - msg->twist.linear.y * sin(yaw_);
            double vy_world = msg->twist.linear.x * sin(yaw_) + msg->twist.linear.y * cos(yaw_);
            
            // Integrate world-frame velocity to update position
            position_.x += vx_world * dt;
            position_.y += vy_world * dt;
            position_.z += msg->twist.linear.z * dt;
            
            // Debug print
            RCLCPP_DEBUG(this->get_logger(), 
                       "Vel: [%.2f, %.2f] World vel: [%.2f, %.2f] Yaw: %.2f", 
                       msg->twist.linear.x, msg->twist.linear.y, 
                       vx_world, vy_world, yaw_);
            
            last_time_ = current_time;
        }

        // Publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position = position_;
        odom_msg.pose.pose.orientation = orientation_;
        odom_msg.twist.twist = msg->twist;
        odom_publisher_->publish(odom_msg);

        // Publish dynamic transform
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = position_.x;
        tf_msg.transform.translation.y = position_.y;
        tf_msg.transform.translation.z = position_.z;
        tf_msg.transform.rotation = orientation_;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Point position_;
    geometry_msgs::msg::Quaternion orientation_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaselinkTf>());
    rclcpp::shutdown();
    return 0;
}