#include "wall_line_detection/test.hpp"

namespace wall_line_detection_pkg
{

WallLineTest::WallLineTest(const rclcpp::NodeOptions & options):
rclcpp::Node("wall_line_test", options)
{
        RCLCPP_INFO(this->get_logger(), "wall line test node construction");
        this->init_params();

        time_action_last_send_goal_ = now() - rclcpp::Duration::from_seconds(100000);

        // init tf2
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

        // pubs
        teb_obstacles_pub_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("obstacles", rclcpp::SystemDefaultsQoS());
        wall_line_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("wall_line_path", rclcpp::QoS(10).best_effort());

        // subs
        auto cb_group_type = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_type;
        wall_line_sub_ = this->create_subscription<wall_line_detection_msgs::msg::WallLinesStamped>(this->msg_topic_name_, 
                rclcpp::SensorDataQoS(), std::bind(&WallLineTest::wall_line_sub_callback, this, std::placeholders::_1), sub_options );

        // camera2_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera2/color/image_raw", rclcpp::SensorDataQoS(), 
        //         std::bind(&WallLineTest::camera2_color_sub_callback, this, std::placeholders::_1), sub_options );

        start_sub_ = this->create_subscription<std_msgs::msg::Bool>("/wall_line", 1, std::bind(&WallLineTest::start_sub_callback, this, std::placeholders::_1));
        

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
        this->declare_parameter<float>("action_frequency", 2.0);
        this->declare_parameter<float>("path_add", 2.5);
        this->declare_parameter<float>("tmp_goal_x", 2.5);
        this->declare_parameter<float>("tmp_goal_y", 2.5);
        this->declare_parameter<float>("tmp_goal_x2", 2.5);
        this->declare_parameter<float>("tmp_goal_y2", 2.5);
        this->declare_parameter<float>("path_pose_distance_max", 0.2);
        this->declare_parameter<float>("path_rotate_offset", 1.0);
        
        this->msg_topic_name_ = this->get_parameter_or<std::string>("msg_topic_name", "wall_lines_stamped");
        this->msg_time_tolerance_ = this->get_parameter_or<float>("msg_time_tolerance", 1.0);
        this->use_offset_ = this->get_parameter_or<bool>("use_offset", false);
        this->path_offset_ = this->get_parameter_or<float>("path_offset", 0.5);
        this->only_get_msg_once_ = this->get_parameter_or<bool>("only_get_msg_once", false);
        this->action_frequency_ = this->get_parameter_or<float>("action_frequency", 2.0);
        this->path_add_= this->get_parameter_or<float>("path_add", 2.5);
        this->tmp_goal_x_= this->get_parameter_or<float>("tmp_goal_x", 2.5);
        this->tmp_goal_y_= this->get_parameter_or<float>("tmp_goal_y", 2.5);
        this->tmp_goal_x2_= this->get_parameter_or<float>("tmp_goal_x2", 2.5);
        this->tmp_goal_y2_= this->get_parameter_or<float>("tmp_goal_y2", 2.5);
        this->path_pose_distance_max_ = this->get_parameter_or<float>("path_pose_distance_max", 0.2);
        this->path_rotate_offset_ = this->get_parameter_or<float>("path_rotate_offset", 1.0);
}

void WallLineTest::camera2_color_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
        rclcpp::Time color_time = msg->header.stamp;

        std::string refFrame = "map";
        std::string childFrame = "camera2_depth_optical_frame";
        std::string errMsg;

        tf2::Transform tf2_color2;
        double timeout = 0.5;

        if (!this->tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
		    tf2::durationFromSec(timeout), &errMsg))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get TF from " 
                << refFrame << " to " << childFrame << ": " << errMsg);
        } 
        else 
        {
            try 
            {
                auto tf2_color2_msg = this->tf_buffer_->lookupTransform( refFrame, childFrame, color_time, rclcpp::Duration::from_seconds(timeout));
                
                tf2::fromMsg(tf2_color2_msg.transform, tf2_color2);               
            } 
            catch (const tf2::TransformException & e) 
            {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
            }
        }

        RCLCPP_INFO(get_logger(), "position => x: %f, y: %f", tf2_color2.getOrigin().getX(), tf2_color2.getOrigin().getY());
        RCLCPP_INFO(get_logger(), "yaw: %f", tf2::getYaw(tf2_color2.getRotation()));
}

void WallLineTest::start_sub_callback(std_msgs::msg::Bool::ConstSharedPtr msg)
{
        this->start_ = msg->data;
        if (action_started && !this->start_)
        {
                RCLCPP_WARN(get_logger(), "async cancel the goal");
                // follow_path_client_->async_cancel_goal(goal_future.get());
                action_started = false;
                follow_path_client_->async_cancel_all_goals();
        }

}


void WallLineTest::get_map_robot_tf(rclcpp::Time laser_scan_time)
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
                transformStamped = this->tf_buffer_->lookupTransform( refFrame, childFrame, laser_scan_time, rclcpp::Duration::from_seconds(0.1));
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
        RCLCPP_DEBUG(get_logger(), "delta_time_seconds: %f, msg_time_tolerance: %f", delta_time_seconds, msg_time_tolerance_);
        
        if (delta_time_seconds < this->msg_time_tolerance_)
        {
                ret = true;
        }

        return ret;
}

void WallLineTest::wall_line_sub_callback(const wall_line_detection_msgs::msg::WallLinesStamped::ConstSharedPtr msg)
{
        // (void) msg;
        this->laserscan_ = msg->laser_scan;

        if (!sin_cos_map_generated)
        {
                sin_map.clear();
                cos_map.clear();
                int range_size = (int)laserscan_.ranges.size();
                sin_map.resize(range_size);
                cos_map.resize(range_size);

                for (int i = 0; i < range_size; i++)
                {
                        float angle = laserscan_.angle_min + i * laserscan_.angle_increment;
                        sin_map[i] = sin(angle);
                        cos_map[i] = cos(angle);
                }
                sin_cos_map_generated = true;
        }
        
        scan_point_vec.clear();
        scan_point_vec.resize(laserscan_.ranges.size());

        if (this->only_get_msg_once_)
        {
                if (!this->get_msg_)
                {
                        if (is_current(*msg) && msg->line_selected != -1)
                        {
                                this->msg_ = *msg;
                                this->get_msg_ = true;
                                RCLCPP_DEBUG(get_logger(), "start process (only once)......");
                                rclcpp::Time laser_scan_time = msg->header.stamp;
                                process_(msg_.wall_lines[msg_.line_selected], laser_scan_time, this->path_offset_);
                        }
                }
                else
                {
                        // do nothing                        
                }

        }
        else
        {
                auto current_ = is_current(*msg);
                if (current_ && msg->line_selected != -1)
                {
                        this->msg_ = *msg;
                        RCLCPP_DEBUG(get_logger(), "start process ......");
                        rclcpp::Time laser_scan_time = msg->header.stamp;
                        process_(msg_.wall_lines[msg_.line_selected], laser_scan_time, this->path_offset_);
                }
                else
                {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "current: %s, line_selected: %d", 
                                 current_?"true":"false", msg->line_selected);
                }
        }

}

void WallLineTest::process_(wall_line_detection_msgs::msg::WallLine wall_line, rclcpp::Time laser_scan_time, float offset)
{
        get_map_robot_tf(laser_scan_time);
        auto path = generate_path(wall_line, map_robot_tf, offset);
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "path size: %ld", path.poses.size());
        if (path.poses.size() < 20)
        {
                return;
        }
        auto goal = nav2_msgs::action::FollowPath::Goal();
        goal.path = path;

        RCLCPP_DEBUG(get_logger(), "publish /wall_line_path topic");
        RCLCPP_DEBUG(get_logger(), "path's poses size: %zd", path.poses.size());
        wall_line_path_pub_->publish(path);

        rclcpp::Time now_time = now();
        if ((now_time - time_action_last_send_goal_).seconds() > (1.0 / action_frequency_))
        {
                // get_map_robot_tf(laser_scan_time);
                // auto robot_x = this->map_robot_tf.getOrigin().getX();
                // auto robot_y = this->map_robot_tf.getOrigin().getY();
                // auto path_end_x = path.poses[path.poses.size()-1].pose.position.x;
                // auto path_end_y = path.poses[path.poses.size()-1].pose.position.y;
                // auto path_start_x = path.poses[0].pose.position.x;
                // auto path_start_y = path.poses[0].pose.position.y;

                if (start_)
                {
                        // RCLCPP_INFO(get_logger(), "now_time: %f, last_time: %f, delta: %f", now_time.seconds(), time_action_last_send_goal_.seconds(), (now_time - time_action_last_send_goal_).seconds());
                        
                        // double distance_pass = std::hypot(-2.0 - robot_x, -3.0 - robot_y);
                        // if (distance_pass < 1.5)
                        // {
                                RCLCPP_INFO(get_logger(), "follow_path_client_ async_send_goal");
                                goal_future = follow_path_client_->async_send_goal(goal);
                                time_action_last_send_goal_ = now_time;
                                action_started = true;
                        // }
                        // else
                        // {
                        //         RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "diatance_pass: %f", distance_pass);
                        // }
                }
                else
                {
                //         if (action_started)
                //         {
                //                 // follow_path_client_->async_cancel_goal(goal_future.get());
                //                 follow_path_client_->async_cancel_all_goals();
                //                 action_started = false;
                //         }
                }

                // double distance_remain;
                // if (path.poses.size() > 0)
                // {
                        
                //         distance_remain = std::hypot(robot_x - path_end_x, robot_y - path_end_y);
                //         if (distance_remain < 0.15)
                //         {
                //                 follow_path_client_->async_cancel_goal(goal_future.get());
                //                 action_started = false;
                //         }
                // }
                
        }
}

nav_msgs::msg::Path WallLineTest::generate_path(wall_line_detection_msgs::msg::WallLine wall_line, tf2::Transform tf_robot, float offset)
{
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
        RCLCPP_DEBUG(get_logger(), "start x: %f, y: %f", start.x, start.y);
        RCLCPP_DEBUG(get_logger(), "end   x: %f, y: %f", end.x, end.y);

        double angle_to_end = std::atan2(end.y - start.y, end.x - start.x);

        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->msg_.header.stamp;

        // geometry_msgs::msg::Pose pose;
        // double distance = std::hypot(end.y - start.y, end.x - start.x);
        // double distance_generate;
        // float resolution = 0.05;

        double angle_to_robot = std::atan2(robot_y - start.y, robot_x - start.x);
        double angle_rotation = angles::shortest_angular_distance(angle_to_end, angle_to_robot);

        double angle_offset;
        if (angle_rotation > 0)
        {
                wall_line_orientation = wall_line_LR::RIGHT;
                angle_offset = angle_to_end + M_PI / 2.0;
        }
        else
        {
                wall_line_orientation = wall_line_LR::LEFT;
                angle_offset = angle_to_end - M_PI / 2.0;
        }


        switch(wall_line_orientation)
        {
                case wall_line_LR::RIGHT:
                {
                        RCLCPP_DEBUG(get_logger(), "wall_line => right");
                        size_t index_start = 0, index_end = 0;
                        float dis_start_min = std::numeric_limits<float>::max();
                        float dis_end_min = std::numeric_limits<float>::max();
                        float dis_start_current, dis_end_current;

                        for (size_t i = 0; i < laserscan_.ranges.size(); i++)
                        {
                                float scan_point_x, scan_point_y;
                                if (std::isinf(laserscan_.ranges[i]) || laserscan_.ranges[i] <= laserscan_.range_min || laserscan_.ranges[i] >= laserscan_.range_max)
                                {
                                        continue;
                                }
                                else
                                {
                                        scan_point_x = robot_x + laserscan_.ranges[i] * (cos(robot_theta) * cos_map[i] - sin(robot_theta) * sin_map[i]);
                                        scan_point_y = robot_y + laserscan_.ranges[i] * (sin(robot_theta) * cos_map[i] + cos(robot_theta) * sin_map[i]);
                                        dis_start_current = std::hypot(start.y - scan_point_y, start.x - scan_point_x);
                                        dis_end_current = std::hypot(end.y - scan_point_y, end.x - scan_point_x);
                                        if (dis_start_current < dis_start_min)
                                        {
                                                dis_start_min = dis_start_current;
                                                index_start = i;
                                        }
                                        if (dis_end_current < dis_end_min)
                                        {
                                                dis_end_min = dis_end_current;
                                                index_end = i;
                                        }
                                }
                        }

                        RCLCPP_DEBUG(get_logger(), "index_start: %zd, index_end: %zd", index_start, index_end);
                        
                        
                        float resolution = 0.05;
                        bool path_inited = false;
                        geometry_msgs::msg::PoseStamped pose_last;
                        RCLCPP_DEBUG(get_logger(), "--------------------------------");
                        for (size_t index = index_start; index <= index_end; index++)
                        {
                                if (std::isinf(laserscan_.ranges[index]) || laserscan_.ranges[index] <= laserscan_.range_min || laserscan_.ranges[index] >= laserscan_.range_max)
                                {
                                        continue;
                                }
                                else
                                {
                                        geometry_msgs::msg::PoseStamped poseStamped;
                                        poseStamped.header.frame_id = "map";
                                        poseStamped.pose.position.x = robot_x + laserscan_.ranges[index] * (cos(robot_theta) * cos_map[index] - sin(robot_theta) * sin_map[index]);
                                        poseStamped.pose.position.y = robot_y + laserscan_.ranges[index] * (sin(robot_theta) * cos_map[index] + cos(robot_theta) * sin_map[index]);
                                        poseStamped.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_offset);
                                        if (use_offset_)
                                        {
                                                poseStamped.pose.position.x = poseStamped.pose.position.x + offset * cos(angle_offset);
                                                poseStamped.pose.position.y = poseStamped.pose.position.y + offset * sin(angle_offset);
                                        }
                                        else
                                        {
                                        // do nothing   
                                        }

                                        if (!path_inited)
                                        {
                                                path.poses.push_back(poseStamped);
                                                pose_last = poseStamped;
                                                path_inited = true;
                                        }
                                        else
                                        {
                                                float distance = std::hypot(poseStamped.pose.position.x - pose_last.pose.position.x,
                                                        poseStamped.pose.position.y - pose_last.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "pose_current => x: %f, y: %f", poseStamped.pose.position.x, poseStamped.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "pose_last    => x: %f, y: %f", pose_last.pose.position.x, pose_last.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "distance: %f, resolution: %f", distance, resolution);
                                                if (distance > this->path_pose_distance_max_)
                                                {
                                                        break;
                                                }
                                                if (distance >= resolution)
                                                {
                                                        path.poses.push_back(poseStamped);
                                                        pose_last = poseStamped; 
                                                }
                                                else
                                                {
                                                        continue;
                                                }
                                        }
                                }                             
                        }

                        RCLCPP_DEBUG(get_logger(), "path's size: %zd", path.poses.size());

                        // pub "obstacle" topic
                        geometry_msgs::msg::Point32 point_start, point_end;
                        double range_start, range_end, angle_start, angle_end;

                        range_start = this->laserscan_.ranges[index_start];
                        angle_start = this->laserscan_.angle_min + static_cast<double>(index_start) * this->laserscan_.angle_increment;
                        point_start.x = range_start * cos(angle_start);
                        point_start.y = range_start * sin(angle_start);

                        range_end = this->laserscan_.ranges[index_end];
                        angle_end = this->laserscan_.angle_min + static_cast<double>(index_end) * this->laserscan_.angle_increment;
                        point_end.x = range_end * cos(angle_end);
                        point_end.y = range_end * sin(angle_end);

                        costmap_converter_msgs::msg::ObstacleArrayMsg obstacle_array_msg;
                        obstacle_array_msg.header = this->laserscan_.header;

                        costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;
                        obstacle_msg.header = this->laserscan_.header;
                        obstacle_msg.polygon.points.push_back(point_start);
                        obstacle_msg.polygon.points.push_back(point_end);

                        obstacle_array_msg.obstacles.push_back(obstacle_msg);

                        teb_obstacles_pub_->publish(obstacle_array_msg);

                        break;
                }
                case wall_line_LR::LEFT:
                {
                        RCLCPP_DEBUG(get_logger(), "wall_line => left");
                        size_t index_start = 0, index_end = 0;
                        float dis_start_min = std::numeric_limits<float>::max();
                        float dis_end_min = std::numeric_limits<float>::max();
                        float dis_start_current, dis_end_current;

                        for (size_t i = 0; i < laserscan_.ranges.size(); i++)
                        {
                                float scan_point_x, scan_point_y;
                                if (std::isinf(laserscan_.ranges[i]) || laserscan_.ranges[i] <= laserscan_.range_min || laserscan_.ranges[i] >= laserscan_.range_max)
                                {
                                        continue;
                                }
                                else
                                {
                                        scan_point_x = robot_x + laserscan_.ranges[i] * (cos(robot_theta) * cos_map[i] - sin(robot_theta) * sin_map[i]);
                                        scan_point_y = robot_y + laserscan_.ranges[i] * (sin(robot_theta) * cos_map[i] + cos(robot_theta) * sin_map[i]);
                                        dis_start_current = std::hypot(start.y - scan_point_y, start.x - scan_point_x);
                                        dis_end_current = std::hypot(end.y - scan_point_y, end.x - scan_point_x);
                                        if (dis_start_current < dis_start_min)
                                        {
                                                dis_start_min = dis_start_current;
                                                index_start = i;
                                        }
                                        if (dis_end_current < dis_end_min)
                                        {
                                                dis_end_min = dis_end_current;
                                                index_end = i;
                                        }
                                }
                        }

                        RCLCPP_DEBUG(get_logger(), "index_start: %zd, index_end: %zd", index_start, index_end);
                        
                        float resolution = 0.05;
                        bool path_inited = false;
                        geometry_msgs::msg::PoseStamped pose_last;
                        RCLCPP_DEBUG(get_logger(), "--------------------------------");
                        for (size_t index = index_start; index >= index_end; index -= 1)
                        {
                                if (std::isinf(laserscan_.ranges[index]) || laserscan_.ranges[index] <= laserscan_.range_min || laserscan_.ranges[index] >= laserscan_.range_max)
                                {
                                        continue;
                                }
                                else
                                {
                                        geometry_msgs::msg::PoseStamped poseStamped;
                                        poseStamped.header.frame_id = "map";
                                        poseStamped.pose.position.x = robot_x + laserscan_.ranges[index] * (cos(robot_theta) * cos_map[index] - sin(robot_theta) * sin_map[index]);
                                        poseStamped.pose.position.y = robot_y + laserscan_.ranges[index] * (sin(robot_theta) * cos_map[index] + cos(robot_theta) * sin_map[index]);
                                        poseStamped.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_offset);
                                        if (use_offset_)
                                        {
                                                poseStamped.pose.position.x = poseStamped.pose.position.x + offset * cos(angle_offset);
                                                poseStamped.pose.position.y = poseStamped.pose.position.y + offset * sin(angle_offset);
                                        }
                                        else
                                        {
                                        // do nothing   
                                        }
                                        if (!path_inited)
                                        {
                                                path.poses.push_back(poseStamped);
                                                pose_last = poseStamped;
                                                path_inited = true;
                                        }
                                        else
                                        {
                                                float distance = std::hypot(poseStamped.pose.position.x - pose_last.pose.position.x,
                                                        poseStamped.pose.position.y - pose_last.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "pose_current => x: %f, y: %f", poseStamped.pose.position.x, poseStamped.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "pose_last    => x: %f, y: %f", pose_last.pose.position.x, pose_last.pose.position.y);
                                                RCLCPP_DEBUG(get_logger(), "distance: %f, resolution: %f", distance, resolution);
                                                if (distance > this->path_pose_distance_max_)
                                                {
                                                        break;
                                                }
                                                if (distance >= resolution)
                                                {
                                                        path.poses.push_back(poseStamped);
                                                        pose_last = poseStamped; 
                                                }
                                                else
                                                {
                                                        continue;
                                                }
                                        }
                                }
                        }
                        RCLCPP_DEBUG(get_logger(), "path's size: %zd", path.poses.size());


                        // pub "obstacle" topic
                        geometry_msgs::msg::Point32 point_start, point_end;
                        double range_start, range_end, angle_start, angle_end;

                        range_start = this->laserscan_.ranges[index_start];
                        angle_start = this->laserscan_.angle_min + static_cast<double>(index_start) * this->laserscan_.angle_increment;
                        point_start.x = range_start * cos(angle_start);
                        point_start.y = range_start * sin(angle_start);

                        range_end = this->laserscan_.ranges[index_end];
                        angle_end = this->laserscan_.angle_min + static_cast<double>(index_end) * this->laserscan_.angle_increment;
                        point_end.x = range_end * cos(angle_end);
                        point_end.y = range_end * sin(angle_end);

                        costmap_converter_msgs::msg::ObstacleArrayMsg obstacle_array_msg;
                        obstacle_array_msg.header = this->laserscan_.header;

                        costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;
                        obstacle_msg.header = this->laserscan_.header;
                        // swap start and end order
                        obstacle_msg.polygon.points.push_back(point_end);
                        obstacle_msg.polygon.points.push_back(point_start);

                        obstacle_array_msg.obstacles.push_back(obstacle_msg);

                        teb_obstacles_pub_->publish(obstacle_array_msg);
                        break;
                }
        }

        if (use_offset_)
        { 
                start.x = start.x + std::cos(angle_offset) * offset;
                start.y = start.y + std::sin(angle_offset) * offset;
                end.x = end.x + std::cos(angle_offset) * offset;
                end.y = end.y + std::sin(angle_offset) * offset;
        }

      // path 只包含起点和终点
//       (void) resolution;
//       (void) distance_generate;
//       (void) distance;
//       geometry_msgs::msg::PoseStamped poseStamped;
//       poseStamped.header.frame_id = "map";

//       pose.position.x = start.x;
//       pose.position.y = start.y;
//       pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_end);

//       poseStamped.pose = pose;
//       path.poses.push_back(poseStamped);

//       pose.position.x = end.x;
//       pose.position.y = end.y;
//       pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_end);

//       poseStamped.pose = pose;
//       path.poses.push_back(poseStamped);
//       RCLCPP_DEBUG(get_logger(), "path size: %zd", path.poses.size());

      // 起点和终点之间生成连续点的path
//       pose.position.x = start.x;
//       pose.position.y = start.y;
//       pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_end);
//       distance_generate = std::hypot(pose.position.y - start.y, pose.position.x - start.x);

//       geometry_msgs::msg::PoseStamped poseStamped;
//       poseStamped.header.frame_id = "map";
      
//       while (distance_generate < distance)
//       {
//         poseStamped.pose = pose;
//         path.poses.push_back(poseStamped);

//         // 更新 pose
//         pose.position.x = pose.position.x + resolution * std::cos(angle_to_end);
//         pose.position.y = pose.position.y + resolution * std::sin(angle_to_end);
//         pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle_to_end);
//         distance_generate = std::hypot(pose.position.y - start.y, pose.position.x - start.x);
//       }
//       poseStamped.pose = pose;
//       path.poses.push_back(poseStamped);
        
        // add orientation for path's poses
        if (path.poses.size() > 1)
        {
                for (size_t i = 0; i < path.poses.size(); ++i)
                {
                        if (i == 0)
                        {
                                double theta = std::atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y,
                                                        path.poses[1].pose.position.x - path.poses[0].pose.position.x);
                                tf2::Quaternion orientation;
                                orientation.setRPY(0, 0, theta);
                                path.poses[0].pose.orientation.w = orientation.w();
                                path.poses[0].pose.orientation.z = orientation.z();
                                path.poses[0].pose.orientation.y = orientation.y();
                                path.poses[0].pose.orientation.x = orientation.x();
                        }
                        else
                        {
                                double theta = std::atan2(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,
                                                        path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
                                tf2::Quaternion orientation;
                                orientation.setRPY(0, 0, theta);
                                path.poses[i].pose.orientation.w = orientation.w();
                                path.poses[i].pose.orientation.z = orientation.z();
                                path.poses[i].pose.orientation.y = orientation.y();
                                path.poses[i].pose.orientation.x = orientation.x();
                        }
                }
        }

        if (path.poses.size() > 0)
        {

                //  add path's length, plan A

                // auto quat = path.poses[path.poses.size() - 1].pose.orientation;
                // double theta = tf2::getYaw(quat);
                // double resolution = 0.05;
                // for (float distance = path_add_; distance > 0.0; distance -= resolution)
                // {
                //         geometry_msgs::msg::PoseStamped pose;
                //         pose.pose.orientation = quat;
                //         pose.pose.position.x = path.poses[path.poses.size() -1].pose.position.x + resolution * cos(theta);
                //         pose.pose.position.y = path.poses[path.poses.size() -1].pose.position.y + resolution * sin(theta);
                //         pose.pose.position.z = 0.0;
                //         path.poses.push_back(pose);
                // }    

                //  add path's length, plan B         
                // auto quat = path.poses[path.poses.size() - 1].pose.orientation;
                // double theta = tf2::getYaw(quat);
                auto pose_start = path.poses[0];
                auto pose_end = path.poses[path.poses.size() - 1];
                double theta = std::atan2(pose_end.pose.position.y - pose_start.pose.position.y, pose_end.pose.position.x - pose_start.pose.position.x);

                double resolution = 0.05;

                float distance_current = 100.0, distance_last = 100.0;
                float tmp_goal_x, tmp_goal_y;
                // if (angle_to_end > 2.3)
                if (angle_to_end < 0.0 && angle_to_end > -0.8)
                {
                        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,"angle_to_end: %f", angle_to_end);
                        tmp_goal_x = tmp_goal_x_;
                        tmp_goal_y = tmp_goal_y_;
                }
                else
                {
                        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,"angle_to_end: %f", angle_to_end);
                        tmp_goal_x = tmp_goal_x2_;
                        tmp_goal_y = tmp_goal_y2_;
                }
                while (true)
                {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose.position.x = path.poses[path.poses.size() -1].pose.position.x + resolution * cos(theta);
                        pose.pose.position.y = path.poses[path.poses.size() -1].pose.position.y + resolution * sin(theta);
                        pose.pose.position.z = 0.0;

                        tf2::Quaternion orientation;
                        orientation.setRPY(0, 0, theta);
                        pose.pose.orientation.w = orientation.w();
                        pose.pose.orientation.z = orientation.z();
                        pose.pose.orientation.y = orientation.y();
                        pose.pose.orientation.x = orientation.x();

                        distance_current = std::hypot(pose.pose.position.x - tmp_goal_x, pose.pose.position.y - tmp_goal_y);
                        RCLCPP_DEBUG(get_logger(), "tmp_goal_x: %f, pose_x: %f", tmp_goal_x, pose.pose.position.x);
                        RCLCPP_DEBUG(get_logger(), "distance_current: %f, distance_last: %f", distance_current, distance_last);
                        if (distance_current > distance_last)
                        {
                                RCLCPP_DEBUG(get_logger(), "break");
                                break;
                        }
                        else
                        {
                                path.poses.push_back(pose);
                                distance_last = distance_current;
                        }
                        
                }

                // add rotate_path
                // float path_rotate = angles::normalize_angle(angle_offset + M_PI);
                // float path_rotate = angles::normalize_angle(theta - M_PI/2.0);
                float path_rotate = angles::normalize_angle(theta + M_PI/2.0);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "path_rotate_angle: %f", path_rotate);
                for (float distance = this->path_rotate_offset_; distance > 0.0; distance -= resolution)
                {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose.position.x = path.poses[path.poses.size() -1].pose.position.x + resolution * cos(path_rotate);
                        pose.pose.position.y = path.poses[path.poses.size() -1].pose.position.y + resolution * sin(path_rotate);
                        pose.pose.position.z = 0.0;

                        tf2::Quaternion orientation;
                        orientation.setRPY(0, 0, path_rotate);
                        pose.pose.orientation = tf2::toMsg(orientation);
                        // pose.pose.orientation.w = orientation.w();
                        // pose.pose.orientation.z = orientation.z();
                        // pose.pose.orientation.y = orientation.y();
                        // pose.pose.orientation.x = orientation.x();
                        path.poses.push_back(pose);                
                }

        }  
        
        return path;
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(wall_line_detection_pkg::WallLineTest)
