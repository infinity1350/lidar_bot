#include "planning/dijkstra_planner.hpp"


namespace planning
{
    public:
        DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_planner")
        {
            rclcpp::Qos map_qos(10);
            map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            map_sub_ = this->createSucbcription<nav_msgs::msg::OccupancyGrid>(
                "/map", map_qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));

            pose_sub_ = this->createSubscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, std::bind(&DijkstraPlanner::poseCallback, this, placeholders::_1)
            );
            
            path_pub_ = this->createPublisher<nav_msgs::msg::Path>(
                "/dijkstra_path", 10);

            map_pub_ = this->createPublisher<nav_msgs::msg::OccupancyGrid>(
                "/dijkstra/visited_map", 10);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }

        void DijsktraPlanner::mapCallback(const nav_msgs::msg::OccuoancyGrid::SharedPtr msg)
        {
            map_ = map;
            visited_map_.header.frame_id = map_->header.frame_id;
            visited_map_.info = map_->info;
            visited_map_.data = ;
        }
}