#ifndef PLANNING__DIJKSTRA_PLANNER_HPP
#define PLANNING__DIJKSTRA_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace planning
{
class DijkstraPlanner : public rclcpp::Node
{
    struct GraphNode
    {
        int x;
        int y;
        int cost;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x, int in_y) : x(in_x), y(in_y),cost(0)
        {
        }

        GraphNode() : GraphNode(0,0) 
        {
        }

        bool operator > (const GraphNode & other)const
        {
            return cost > other.cost;
        }

        bool operator == ( const GraphNode & other) const 
        {
            return cost == other.cost;
        }
    };
    public:
        DijkstraPlanner();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

        std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_;
        std::shared_ptr<nav_msgs::msg::OccupancyGrid> visited_map_;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
        geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
        bool poseOnMap(const GraphNode & node);
        unsigned int poseToCell(const GraphNode & node);
        
        nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

};
}

#endif