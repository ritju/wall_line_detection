#include "wall_line_detection/test.hpp"

namespace wall_line_detection_pkg
{

WallLineTest::WallLineTest(const rclcpp::NodeOptions & options):
rclcpp::Node("wall_line_test", options)
{
        RCLCPP_INFO(this->get_logger(), "wall line test node construction");

        // subs
        auto cb_group_type = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = cb_group_type;
        wall_line_sub_ = this->create_subscription<wall_line_detection_msgs::msg::WallLinesStamped>("wall_lines_stamped", 
                rclcpp::SensorDataQoS(), std::bind(&WallLineTest::wall_line_sub_callback, this, std::placeholders::_1), sub_options );

        // follow_path action client
        follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");

}

WallLineTest::~WallLineTest()
{
        RCLCPP_INFO(this->get_logger(), "wall line test node destruction");
}

void WallLineTest::wall_line_sub_callback(const wall_line_detection_msgs::msg::WallLinesStamped::ConstSharedPtr msg)
{
        (void) msg;
}


} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(wall_line_detection_pkg::WallLineTest)
