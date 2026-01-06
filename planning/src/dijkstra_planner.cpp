#include "planning/dijkstra_planner.hpp"
#include <vector>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <queue>


namespace planning
{
    
    DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_planner")
    {
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&DijkstraPlanner::poseCallback, this, std::placeholders::_1));
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/dijkstra_path", 10);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/dijkstra/visited_map", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

    void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = msg;
        visited_map_.header.frame_id = map_->header.frame_id;
        visited_map_.info = map_->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1);
        RCLCPP_INFO(get_logger(), "Map received: %dx%d, resolution: %.3f", 
                    map_->info.width, map_->info.height, map_->info.resolution);
    }

    void DijkstraPlanner::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        
        if(!map_){
            RCLCPP_ERROR(get_logger(), "No map received");
            return;
        }
        RCLCPP_INFO(get_logger(), "Goal pose received");
        visited_map_.data = std::vector<int8_t>(visited_map_.info.width * visited_map_.info.height, -1); 
        geometry_msgs::msg::TransformStamped map_to_base_tf;
        
        try
        {
            map_to_base_tf = tf_buffer_->lookupTransform(
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

        RCLCPP_INFO(get_logger(), "Start pose (world): (%.2f, %.2f)", 
                    map_to_base_pose.position.x, map_to_base_pose.position.y);
        RCLCPP_INFO(get_logger(), "Goal pose (world): (%.2f, %.2f)", 
                    msg->pose.position.x, msg->pose.position.y);

        auto path = plan(map_to_base_pose, msg->pose);
        if(!path.poses.empty())
        {
            RCLCPP_INFO(get_logger(), "Shortest path to the goal is found with %zu waypoints", 
                        path.poses.size());
            path_pub_->publish(path);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No path found to the goal");
        }
    }

    nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose & start, 
        const geometry_msgs::msg::Pose & goal)
    {
        // Using std::pair as in your original code
        std::vector<std::pair<int, int>> explore_directions = {
            {1, 0},     // right
            {-1, 0},    // left
            {0, 1},     // up
            {0, -1},    // down
            {1, 1},     // diagonal: right-up
            {-1, 1},    // diagonal: left-up
            {1, -1},    // diagonal: right-down
            {-1, -1}    // diagonal: left-down
        };
        
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        std::vector<GraphNode> visited_nodes; 
        nav_msgs::msg::Path path;

        // Convert poses to grid coordinates FIRST
        GraphNode start_grid = worldToGrid(start);
        GraphNode goal_grid = worldToGrid(goal);

        RCLCPP_INFO(get_logger(), "Planning started from grid (%d, %d) to grid (%d, %d)", 
                    start_grid.x, start_grid.y, goal_grid.x, goal_grid.y);

        // Validate start position
        if(!poseOnMap(start_grid)) {
            RCLCPP_ERROR(get_logger(), "Start position (%d, %d) is OUT OF MAP BOUNDS! Map size: %dx%d", 
                         start_grid.x, start_grid.y, map_->info.width, map_->info.height);
            return path;
        }

        // Validate goal position
        if(!poseOnMap(goal_grid)) {
            RCLCPP_ERROR(get_logger(), "Goal position (%d, %d) is OUT OF MAP BOUNDS! Map size: %dx%d", 
                         goal_grid.x, goal_grid.y, map_->info.width, map_->info.height);
            return path;
        }

        RCLCPP_INFO(get_logger(), "Start on map: true, Goal on map: true");

        // Check start cell value
        int start_cell = poseToCell(start_grid);
        int start_value = map_->data.at(start_cell);
        RCLCPP_INFO(get_logger(), "Start cell value: %d (0=free, >0=obstacle, -1=unknown)", start_value);
        
        if(start_value != 0) {
            RCLCPP_ERROR(get_logger(), "Start position is NOT FREE! Cell value: %d", start_value);
            return path;
        }

        // Check goal cell value
        int goal_cell = poseToCell(goal_grid);
        int goal_value = map_->data.at(goal_cell);
        RCLCPP_INFO(get_logger(), "Goal cell value: %d (0=free, >0=obstacle, -1=unknown)", goal_value);
        
        if(goal_value != 0) {
            RCLCPP_ERROR(get_logger(), "Goal position is NOT FREE! Cell value: %d", goal_value);
            return path;
        }

        RCLCPP_INFO(get_logger(), "Explore directions size: %zu", explore_directions.size());

        // Initialize start node
        start_grid.cost = 0;
        start_grid.prev = nullptr;
        pending_nodes.push(start_grid);
        visited_nodes.push_back(start_grid);

        RCLCPP_INFO(get_logger(), "Starting Dijkstra search...");

        int iterations = 0;
        const int MAX_ITERATIONS = 100000;
        bool goal_found = false;
        GraphNode active_node;
        
        while(!pending_nodes.empty() && rclcpp::ok() && iterations < MAX_ITERATIONS)
        {
            iterations++;
            
            active_node = pending_nodes.top();
            pending_nodes.pop();

            // Log progress every 1000 iterations
            if(iterations % 1000 == 0) {
                RCLCPP_INFO(get_logger(), "Iteration %d: Active node (%d, %d), pending: %zu, visited: %zu", 
                            iterations, active_node.x, active_node.y, 
                            pending_nodes.size(), visited_nodes.size());
            }

            // Check if goal is reached
            if(goal_grid == active_node)
            {
                RCLCPP_INFO(get_logger(), "GOAL REACHED after %d iterations!", iterations);
                goal_found = true;
                break;
            }

            // Explore all directions using your operator+ overload
            for(const auto & dir : explore_directions)
            {
                GraphNode new_node = active_node + dir;  // Using your overloaded operator
                
                // Check if already visited
                if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) != visited_nodes.end())
                    continue;
                
                // Check if on map
                if(!poseOnMap(new_node))
                    continue;
                
                // Check if free space (0 = free)
                int cell_value = map_->data.at(poseToCell(new_node));
                if(cell_value != 0)
                    continue;
                
                // Add to search
                new_node.cost = active_node.cost + 1;
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }

            // Mark visited cell on visualization map
            visited_map_.data.at(poseToCell(active_node)) = 50;
        }

        // Publish final visited map
        map_pub_->publish(visited_map_);

        // Check if we found the goal
        if(iterations >= MAX_ITERATIONS) {
            RCLCPP_WARN(get_logger(), "Max iterations (%d) reached!", MAX_ITERATIONS);
            return path;
        }

        if(!goal_found) {
            RCLCPP_WARN(get_logger(), "Search ended but goal not reached. Explored %d nodes", iterations);
            return path;
        }

        // Reconstruct path
        RCLCPP_INFO(get_logger(), "Reconstructing path...");
        path.header.frame_id = map_->header.frame_id;
        path.header.stamp = this->now();

        // Build path from goal to start
        std::vector<GraphNode> path_nodes;
        GraphNode current = active_node;
        
        while(current.prev != nullptr && rclcpp::ok())
        {
            path_nodes.push_back(current);
            current = *(current.prev);
        }
        path_nodes.push_back(start_grid);  // Add start node

        RCLCPP_INFO(get_logger(), "Path has %zu nodes", path_nodes.size());

        // Reverse to get start->goal order
        std::reverse(path_nodes.begin(), path_nodes.end());

        // Convert to PoseStamped messages
        for(const auto & node : path_nodes)
        {
            geometry_msgs::msg::Pose pose = gridToWorld(node);
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map_->header.frame_id;
            pose_stamped.header.stamp = this->now();
            pose_stamped.pose = pose;
            pose_stamped.pose.orientation.w = 1.0;  // Set valid orientation
            
            path.poses.push_back(pose_stamped);
        }

        RCLCPP_INFO(get_logger(), "Path successfully created with %zu waypoints", path.poses.size());
        return path;
    }

    DijkstraPlanner::GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
        int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
        pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }

    bool DijkstraPlanner::poseOnMap(const GraphNode & node)
    {
        return node.x >= 0 && node.x < static_cast<int>(map_->info.width) &&
               node.y >= 0 && node.y < static_cast<int>(map_->info.height);
    }

    unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
    {
        return node.y * map_->info.width + node.x;
    }

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}