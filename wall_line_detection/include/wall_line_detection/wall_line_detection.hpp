#ifndef WALL_LINE_DETECTION_PKG_WALL_LINE_DETECTION_HPP_
#define WALL_LINE_DETECTION_PKG_WALL_LINE_DETECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>

#include <queue>

namespace wall_line_detection_pkg
{

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;

struct MapPose
{
        int x;
        int y;
};

/**
 *  墙线检测类，通过激光雷达或深度相机数据检测墙线。
*/
class WallLineDetection : public rclcpp::Node
{



public:
/**
 * 构造函数
*/
explicit WallLineDetection(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

/**
 * 析构函数
*/
~WallLineDetection();

// subs
rclcpp::Subscription<LaserScanMsg>::SharedPtr laserscan_sub_;
rclcpp::Subscription<OccupancyGridMsg>::SharedPtr map_sub_;

// pubs
rclcpp::Publisher<LaserScanMsg>::SharedPtr laserscan_pub_;

// callback
void laserscan_sub_callback_(const LaserScanMsg::SharedPtr msg);
void map_sub_callback_(const OccupancyGridMsg::SharedPtr msg);

// params
std::string laserscan_topic_sub_name;
std::string laserscan_topic_pub_name;
std::string map_topic_sub_name;
int laserscan_queue_size_;
std::string map_frame;
std::string base_link_frame;
std::string laser_link_frame;

// member functions
typedef std::recursive_mutex mutex_t;
mutex_t * getMutex()
{
        return laserscan_queue_access_;
}

sensor_msgs::msg::LaserScan process_laserscan_queue(std::queue<LaserScanMsg> queue);

/**
 * @brief 初始化类所有parameters
 * 
 * @param void
 * @return void
*/
void init_params();

/**
 * @brief 得到map到laser_link的tf
 * 
 * @param void
 * @return void
*/
void get_map_laser_link_tf();

/**
 * @brief 得到map到robot的tf
 * 
 * @param void
 * @return void
*/
void get_map_robot_tf();

/**
 * @brief 把世界坐标系下坐标转换成图片坐标系下坐标
 * 
 * @param w_x 世界坐标系下x坐标
 * @param w_y 世界坐标系下y坐标
 * @return 图片坐标系下坐标
*/
MapPose word_to_picture_bounded(double w_x, double w_y);

// member variables
OccupancyGridMsg map_grid_;
LaserScanMsg laserscan_input_current_;
LaserScanMsg laserscan_output_;
std::queue<LaserScanMsg> laserscan_queue_;
mutex_t * laserscan_queue_access_;
cv::Mat img_map_;
int map_width_;
int map_height_;
int map_resolution_;
geometry_msgs::msg::Pose map_origin_;

// tf2
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
tf2::Transform map_robot_tf;
tf2::Transform map_laser_link_tf;

std::vector<MapPose> laserscan_points_vector;

}; // end of class

} // end of namespace

#endif




