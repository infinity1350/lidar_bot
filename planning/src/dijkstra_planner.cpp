#include "planning/dijkstra_planner.hpp"
#include <vector>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <queue>


namespace planning
{
    struct GraphNode
    {
        int x;
        int y;
        int cost;
        std::shared_ptr<GraphNode> prev;

        GraphNode(int in_x, int in_y) : x(in_x), y(in_y),cost(0);
        {
        }

        GraphNode() : GraphNodede(0,0) 
        {
        }

        bool operator > (const GraphNode& other)const
        {
            return cost > other.cost;
        }

        bool operator == ( const GraphNode& node) const 
        {
            return cost == other.cost;
        }
    }
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

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }

        void DijsktraPlanner::mapCallback(const nav_msgs::msg::OccuoancyGrid::SharedPtr msg)
        {
            map_ = map;
            visited_map_.header.frame_id = map_->header.frame_id;
            visited_map_.info = map_->info;
            visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1);
        }

        void DijkstraPlanner::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose){
            
            if(!map){
                RCLCPP_ERROR(get_logger(), "No map received");
                return;
            }
            visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1) 
            geometery_msgs::msg::TransformStamped map_to_base_tf;
            
            try
            {
                map_to_base_tf = tf_buffer_.->lookupTransform(
                    map_->header.frame_id, "base_footprint", tf2::TimePointZero);
            }
            catch(const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
                return;
            }

            geometry_msgs::msg::Pose map_to_base_pose;
            map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
            map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
            map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

            auto path = plan(map_to_base_pose, pose->pose);
            if(!path.poses.empty())
            {
                RCLCPP_ERROR(get_logger(), "shortest path to the goal is found");
                path->publish(path)
            }
        }
        nav_msgs::msg::Path DijkstraPlanner(const nav_msgs::smg::PoseStamped& start, 
            nav_msgs::msg::PoseStamped& stop)
        {
            std::vector<std::pair<int, int>> explore_directions = {{-1, 0}, {1, 0}, { 1, 1}, {-1, -1}};
            std::priority_queue <GraphNode, std::vector<GraphNode>, std::greater(GraphNode)> pending_nodes;
            std::priority_queue <GraphNode> visited_nodes; 
            
            nav_msgs::msg::Path path;

            
            pending_nodes.push(worldToGrid(start))
            while(!pending_nodes.empty() && rclcpp::ok)
            {
                GraphNode active_node;
                active_node.pending_nodes.top();
                pending_nodes.pop()

                if(active_node == goal)
                {
                    break;
                }

                for(const auto dir : explore_directions)
                {
                    GraphNode new_node = active_node + dir;
                    if        

                }

            }
            
        }

        GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose )
        {
            int grid_x = (pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
            int grid_y = (pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

            return GraphNode(grid_x, grid_y);
        }

}

int main()
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning::DijkstraPlanner>();
    rclcpp::spin();
    rclcpp::shutdown();
    return 0;
}