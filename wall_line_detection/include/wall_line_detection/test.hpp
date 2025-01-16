#ifndef WALL_LINE_DETECTION_PKG_TEST_HPP_
#define WALL_LINE_DETECTION_PKG_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "wall_line_detection_msgs/msg/wall_lines_stamped.hpp"

namespace wall_line_detection_pkg
{

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


// subs
rclcpp::Subscription<wall_line_detection_msgs::msg::WallLinesStamped>::SharedPtr wall_line_sub_;

void wall_line_sub_callback(const wall_line_detection_msgs::msg::WallLinesStamped::ConstSharedPtr msg);

// action client
rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;

}; // end of class

} // end of namespace

#endif





