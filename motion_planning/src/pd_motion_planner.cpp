#include <motion_planning/pd_motion_planner.hpp>
#include <geometry_msgs/msg/transfrom_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

motion_planning
{
    PDMotionPlanner::PDMotionPlanner() : Node("motion_planner"), 
        kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0), 
        prev_angular_error_(0.0), prev_linear_error_(0.0)
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
        tf_listener_ std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        control_loop_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PdMotionPlanner::controlLoop, this));

        last_cycle_time_ = get_clock()->now;
 
    }

    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        global_plan_ = *path;
    }

    void PDMotionPlanner::controlLoop()
    {
        if(gobal_plan_.poses.empty())
        {
            RCLCPP_ERROR(get_logger(), "There is no path found");
            return;
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        try 
        {
            tf_buffer_->lookupTranform(
                "odom", "base_footprint", tf2::TimePointZero);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Frame ID of the robot pose are : %s", robot_pose.header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "Frame ID of the global plan  are : %s", global_plan_.header.frame_id.c_str());

        if(!transformPlan(robot_pose.header.frame_id))
        {
            RCLCPP_ERROR(get_logger(), "Unable to transfrom plan in robot's frame");
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transfrom.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transfrom.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

        auto next_pose = getNextPose(robot_pose_stamped);
        double dx = next_pose.pose.postion.x - robot_pose_stamped.pose.position.x;
        double dy = next_pose.pose.postion.y - robot_pose_stamped.pose.position.y;

        double distance = std::sqrt(dx * dx + dy * dy);

        if(distance <= 0.1)
        {
            RCLCPP_INFO(get_logger(), "Goal Reached");
            global_plan_.poses.clear();
            return;
        }
        
        next_pose_pub->publisher(next_pose);
        tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
        tf2::fromMsg(robot_pose_stamped, robot_tf);
        tf2::fromMsg(next_pose, next_pose_tf);

        next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;
        double linear_error = next_pose_robot_tf.getOrigin().getX();
        double angular_error = next_pose_robot_tf.getOrigin().getY();
    }

    bool PDMotionPlanner::transformPlan(const std::string & frame)
    {
        if(global_frame_.header.frame_id == frame)
        {
            return true;
        }
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException &ex)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " << global_plan_.header.frame_id << "to " << frame);
            return false;
        }

        for(auto & pose : global_plan_.poses)
        {
            tf2::doTransform(pose, pose, transform);
        }

        global_plan_.poses.header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        auto next_pose = global_pose_.poses.back();
        for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(), ++pose_it)
        {
            double dx = pose_it->pose.postion.x - robot_pose.pose.position.x;
            double dy = pose_it->pose.postion.y - robot_pose.pose.position.y;

            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance > step_size_)
            {
                next_pose = *pose_it;
            }
            else
            {
                break;
            }
        }
        return next_pose;     
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motion_planning::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}