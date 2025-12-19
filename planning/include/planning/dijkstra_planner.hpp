#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namspace planning
{
class DijkstraPlanner : public rclcpp:Node
{
    public:
        DijkstraPlanner();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

        std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void mapCallback(const nav_msgs::msg::OccupancyGrid:SharedPtr msg);
        void poseCallback(const geometry_msgs::msg::PoseStamped:SharedPtr msg);
        
        nav_msgs::msg::Path Plan(const geometry_msgs:msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal);

};
}