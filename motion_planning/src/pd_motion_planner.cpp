#include <motion_planning/pd_motion_planner.hpp>

motion_planning
{
    PDMotionPlanner::PDMotionPlanner() : Node("motion_planner"), 
        kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0)
    {
        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_anular_velocity", max_angular_velocity_);

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
        

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "dijkstra_path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));

        cmd_sub_ = this->create_publisher<geometry_msgs::msg::twist>(
            "/cmd_vel", 10);

        next_pose_pub_ = this->create_publisher<geometry_msg::msg::PoseStamped>("/pd/next_pose", 10);

        tf_buffer_ = std::make_shared<tf_ros::Buffer>(get_clock());
        tf_listener_ std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        control_loop_ = create_wall_timer(std::chrono::milliseconds(100), );
 
    }

    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        global_plan_ = path
    }
}