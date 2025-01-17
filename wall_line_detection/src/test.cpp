#include "wall_line_detection/test.hpp"

namespace wall_line_detection_pkg
{

WallLineTest::WallLineTest(const rclcpp::NodeOptions & options):
rclcpp::Node("wall_line_test", options)
{
        RCLCPP_INFO(this->get_logger(), "wall line test node construction");
        this->init_params();

        // init tf2
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

        // subs
        auto cb_group_type = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_type;
        wall_line_sub_ = this->create_subscription<wall_line_detection_msgs::msg::WallLinesStamped>(this->msg_topic_name_, 
                rclcpp::SensorDataQoS(), std::bind(&WallLineTest::wall_line_sub_callback, this, std::placeholders::_1), sub_options );
        
        wall_line_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("wall_line_path", rclcpp::QoS(10).best_effort());

        // follow_path action client
        follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");

}

WallLineTest::~WallLineTest()
{
        RCLCPP_INFO(this->get_logger(), "wall line test node destruction");
}

void WallLineTest::init_params()
{
        this->declare_parameter<std::string>("msg_topic_name", std::string("wall_lines_stamped"));
        this->declare_parameter<float>("msg_time_tolerance", 1.0);
        this->declare_parameter<bool>("use_offset", false);
        this->declare_parameter<float>("path_offset", 0.5);
        this->declare_parameter<bool>("only_get_msg_once", false);
        
        this->msg_topic_name_ = this->get_parameter_or<std::string>("msg_topic_name", "wall_lines_stamped");
        this->msg_time_tolerance_ = this->get_parameter_or<float>("msg_time_tolerance", 1.0);
        this->use_offset_ = this->get_parameter_or<bool>("use_offset", false);
        this->path_offset_ = this->get_parameter_or<float>("path_offset", 0.5);
        this->only_get_msg_once_ = this->get_parameter_or<bool>("only_get_msg_once", false);
}

void WallLineTest::get_map_robot_tf()
{
        std::string errMsg;
        std::string refFrame = std::string("map");
        std::string childFrame = std::string("base_link");
        geometry_msgs::msg::TransformStamped transformStamped;

        if (!this->tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
		    tf2::durationFromSec(0.5), &errMsg))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get TF from " 
                << refFrame << " to " << childFrame << ": " << errMsg);
        } 
        else 
        {
            try 
            {
                transformStamped = this->tf_buffer_->lookupTransform( refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(0.5));
                tf2::fromMsg(transformStamped.transform, this->map_robot_tf);               
            } 
            catch (const tf2::TransformException & e) 
            {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
            }
        }
}

bool WallLineTest::is_current(wall_line_detection_msgs::msg::WallLinesStamped msg)
{
        bool ret = false;

        rclcpp::Duration delta_time = now() - rclcpp::Time(msg.header.stamp);
        double delta_time_seconds = delta_time.seconds();
        RCLCPP_DEBUG(get_logger(), "delta_time_seconds: %f", delta_time_seconds);
        
        if (delta_time_seconds < this->msg_time_tolerance_)
        {
                ret = true;
        }

        return ret;
}

void WallLineTest::wall_line_sub_callback(const wall_line_detection_msgs::msg::WallLinesStamped::ConstSharedPtr msg)
{
        // (void) msg;
        

        if (this->only_get_msg_once_)
        {
                if (!this->get_msg_)
                {
                        if (is_current(*msg) && msg->line_selected != -1)
                        {
                                this->msg_ = *msg;
                                this->get_msg_ = true;
                                RCLCPP_DEBUG(get_logger(), "start process (only once)......");
                                process_(msg_.wall_lines[msg_.line_selected], this->path_offset_);
                        }
                }
                else
                {
                        // do nothing                        
                }

        }
        else
        {
                if (is_current(*msg) && msg->line_selected != -1)
                {
                        this->msg_ = *msg;
                        RCLCPP_DEBUG(get_logger(), "start process ......");
                        process_(msg_.wall_lines[msg_.line_selected], this->path_offset_);
                }
        }

}

void WallLineTest::process_(wall_line_detection_msgs::msg::WallLine wall_line, float offset)
{
        get_map_robot_tf();
        auto path = generate_path(wall_line, map_robot_tf, offset);
        auto goal = nav2_msgs::action::FollowPath::Goal();
        goal.path = path;

        RCLCPP_DEBUG(get_logger(), "publish /wall_line_path topic");
        RCLCPP_DEBUG(get_logger(), "path's poses size: %zd", path.poses.size());
        wall_line_path_pub_->publish(path);
        follow_path_client_->async_send_goal(goal);
}

nav_msgs::msg::Path WallLineTest::generate_path(wall_line_detection_msgs::msg::WallLine wall_line, tf2::Transform tf_robot, float offset)
{
      float resolution = 0.05;

      double robot_x, robot_y, robot_theta;
      robot_x = tf_robot.getOrigin().getX();
      robot_y = tf_robot.getOrigin().getY();

      robot_theta = tf2::getYaw(tf_robot.getRotation());

      // 判断wall_line的起点和终点
      point start, end;
      double angle1, angle2, dist_angle1, dist_angle2;
      angle1 = std::atan2(wall_line.y2 - wall_line.y1, wall_line.x2 - wall_line.x1);
      angle2 = std::atan2(wall_line.y1 - wall_line.y2, wall_line.x1 - wall_line.x2);

      dist_angle1 = std::abs(angles::shortest_angular_distance(robot_theta, angle1));
      dist_angle2 = std::abs(angles::shortest_angular_distance(robot_theta, angle2));

      if (dist_angle1 < dist_angle2)
      {
        start.x = wall_line.x1;
        start.y = wall_line.y1;
        end.x = wall_line.x2;
        end.y = wall_line.y2;
      }
      else
      {
        start.x = wall_line.x2;
        start.y = wall_line.y2;
        end.x = wall_line.x1;
        end.y = wall_line.y1;
      }
      double angle_to_end = std::atan2(end.y - start.y, end.x - start.x);
      double distance = std::hypot(end.y - start.y, end.x - start.x);
      
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.header.stamp = this->msg_.header.stamp;

      geometry_msgs::msg::Pose pose;
      double distance_generate;

      double angle_to_robot = std::atan2(robot_y - start.y, robot_x - start.x);
      double angle_rotation = angles::shortest_angular_distance(angle_to_end, angle_to_robot);
      if (use_offset_)
      {   
        double angle_offset;
        if (angle_rotation > 0)
        {
                angle_offset = angle_to_end + M_PI / 2.0;
        }
        else
        {
                angle_offset = angle_to_end - M_PI / 2.0;
        }

        start.x = start.x + std::cos(angle_offset) * offset;
        start.y = start.y + std::sin(angle_offset) * offset;
        end.x = end.x + std::cos(angle_offset) * offset;
        end.y = end.y + std::sin(angle_offset) * offset;
      }

      pose.position.x = start.x;
      pose.position.y = start.y;
      distance_generate = std::hypot(pose.position.y - start.y, pose.position.x - start.x);
      
      while (distance_generate < distance)
      {
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.pose = pose;
        path.poses.push_back(poseStamped);

        // 更新 pose
        pose.position.x = pose.position.x + resolution * std::cos(angle_to_end);
        pose.position.y = pose.position.y + resolution * std::sin(angle_to_end);
        distance_generate = std::hypot(pose.position.y - start.y, pose.position.x - start.x);
      }
      
      return path;
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(wall_line_detection_pkg::WallLineTest)
