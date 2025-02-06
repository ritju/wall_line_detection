#ifndef WALL_LINE_DETECTION_PKG_TEST_HPP_
#define WALL_LINE_DETECTION_PKG_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include "pluginlib/class_loader.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "wall_line_detection_msgs/msg/wall_lines_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

// costmap_converter
#include "costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "costmap_converter/costmap_converter_interface.h"

#include "nav2_util/geometry_utils.hpp"

namespace wall_line_detection_pkg
{

struct point
{
        float x;
        float y;
};

enum class wall_line_LR
{
        LEFT,    // 墙线在机器人的左手边
        RIGHT    // 墙线在机器人的右手边
};

/**
 *  墙线测试类
*/
class WallLineTest : public rclcpp::Node
{
public:
/**
 * 构造函数
*/
explicit WallLineTest(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

/**
 * 析构函数
*/
~WallLineTest();

// params
std::string msg_topic_name_;
float msg_time_tolerance_;
bool use_offset_;
float path_offset_;
bool only_get_msg_once_;
float action_frequency_;

rclcpp::Time time_action_last_send_goal_;

void init_params();

void get_map_robot_tf(rclcpp::Time laser_scan_time);

bool is_current(wall_line_detection_msgs::msg::WallLinesStamped msg); // 判断 msg_ 是否在允许的容差范围内

void process_(wall_line_detection_msgs::msg::WallLine wall_line, rclcpp::Time laser_scan_time, float offset);

nav_msgs::msg::Path generate_path(wall_line_detection_msgs::msg::WallLine wall_line, tf2::Transform tf_robot_pose, float offset);

// subs
rclcpp::Subscription<wall_line_detection_msgs::msg::WallLinesStamped>::SharedPtr wall_line_sub_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera2_color_sub_;

// pubs
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wall_line_path_pub_;
rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr teb_obstacles_pub_;

void wall_line_sub_callback(const wall_line_detection_msgs::msg::WallLinesStamped::ConstSharedPtr msg);

void camera2_color_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

// action client
rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;

wall_line_detection_msgs::msg::WallLinesStamped msg_;

bool current_; // 判断 msg_ 是否为最新
bool get_msg_; // 测试时，只获取一次有效line的情况

// tf2
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
tf2::Transform map_robot_tf;

wall_line_LR wall_line_orientation;
sensor_msgs::msg::LaserScan laserscan_;

std::vector<float> sin_map;
std::vector<float> cos_map;
std::vector<point> scan_point_vec;
bool sin_cos_map_generated{false};

pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;

}; // end of class

} // end of namespace

#endif





