#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf_ros/buffer.h>
#include <tf2_ros/tranform_listener.h>

namespace motion_planning
{
    class PDMotionPlanner : public rclcpp::Node
    {
        public:
            PDMotionPlanner();
        
        private:
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            rclcpp:Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::shared_ptr<tf2_ros:Buffer> tf_buffer_;
            rclcpp::Timerbase::SharedPtr control_loop_;
            
            double kp_;
            double kd_;
            double step_size_;
            double max_linear_velocity_;
            double max_angular_velocity;
            nav_msgs::msg::Path global_plan;

            void controlLoop();
            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    };
}