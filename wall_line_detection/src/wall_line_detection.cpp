#include "wall_line_detection/wall_line_detection.hpp"

using std::placeholders::_1, std::placeholders::_2;

namespace wall_line_detection_pkg
{

WallLineDetection::WallLineDetection(const rclcpp::NodeOptions & options):
        rclcpp::Node("wall_line_detection", options)
{
        RCLCPP_INFO(this->get_logger(), "wall line detection node construction");

        this->laserscan_queue_access_ = new mutex_t();
        this->init_params();

        // init tf2
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

        this->get_map_laser_link_tf();

        // subs
        auto sub_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options1 = rclcpp::SubscriptionOptions();
        sub_options1.callback_group = sub_group1;
        this->laserscan_sub_ = this->create_subscription<LaserScanMsg>
                (this->laserscan_topic_sub_name, 30, std::bind(&WallLineDetection::laserscan_sub_callback_, this, _1), sub_options1);
        
        auto sub_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options2 = rclcpp::SubscriptionOptions();
        sub_options2.callback_group = sub_group2;
        this->map_sub_ = this->create_subscription<OccupancyGridMsg>
                (this->map_topic_sub_name, rclcpp::QoS(1).reliable().transient_local(), std::bind(&WallLineDetection::map_sub_callback_, this, _1), sub_options1);

        // pubs
        auto pub_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto pub_options1 = rclcpp::PublisherOptions();
        pub_options1.callback_group = pub_group1;
        this->laserscan_pub_ = this->create_publisher<LaserScanMsg>("/scan_converged", rclcpp::SensorDataQoS(), pub_options1);
}

WallLineDetection::~WallLineDetection()
{
        RCLCPP_INFO(this->get_logger(), "wall line detection node destruction");
        delete laserscan_queue_access_;
}

void WallLineDetection::init_params()
{
        this->declare_parameter<std::string>("laserscan_topic_sub_name", std::string("/scan"));
        this->declare_parameter<std::string>("laserscan_topic_pub_name", std::string("/scan_converged"));
        this->declare_parameter<std::string>("map_topic_sub_name", std::string("/map"));
        this->declare_parameter<int>("laserscan_queue_size", 5);
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("laser_link_frame", "laser_link");
        this->declare_parameter<std::string>("map_frame", "map");

        this->laserscan_topic_sub_name = this->get_parameter_or<std::string>("laserscan_topic_sub_name", std::string("/scan"));
        this->laserscan_topic_pub_name = this->get_parameter_or<std::string>("laserscan_topic_pub_name", std::string("/scan_converged"));
        this->map_topic_sub_name = this->get_parameter_or<std::string>("map_topic_sub_name", std::string("/map"));
        this->laserscan_queue_size_  = this->get_parameter_or<int>("laserscan_queue_size", 5);
        this->base_link_frame = this->get_parameter_or<std::string>("base_link_frame", "base_link");
        this->laser_link_frame = this->get_parameter_or<std::string>("laser_link_frame", "laser_link");
        this->map_frame = this->get_parameter_or<std::string>("map_frame", "map");
}

void WallLineDetection::get_map_laser_link_tf()
{
        std::string errMsg;
        std::string refFrame = this->map_frame;
        std::string childFrame = this->laser_link_frame;
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
                tf2::fromMsg(transformStamped.transform, this->map_laser_link_tf);               
            } 
            catch (const tf2::TransformException & e) 
            {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
            }
        }
}

void WallLineDetection::get_map_robot_tf()
{
        std::string errMsg;
        std::string refFrame = this->map_frame;
        std::string childFrame = this->base_link_frame;
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

void WallLineDetection::laserscan_sub_callback_(const LaserScanMsg::SharedPtr msg)
{
        this->laserscan_input_current_ = *msg;

        std::lock_guard<mutex_t> (*this->getMutex());
        size_t queue_size = this->laserscan_queue_.size();
        if ((int)queue_size >= this->laserscan_queue_size_)
        {
                this->laserscan_queue_.pop();
        }
        this->laserscan_queue_.push(this->laserscan_input_current_);

        queue_size = this->laserscan_queue_.size();
        if ((int)queue_size == this->laserscan_queue_size_)
        {
                this->laserscan_output_ = this->process_laserscan_queue(this->laserscan_queue_);
                this->laserscan_pub_->publish(this->laserscan_output_);
                std::vector<MapPose>().swap(this->laserscan_points_vector);
                double yaw = this->laserscan_output_.range_min;
                double yaw_delta = this->laserscan_output_.angle_increment;
                for (int i = 0; i < (int) this->laserscan_output_.ranges.size(); i++, yaw += yaw_delta)
                {
                        double range = laserscan_output_.ranges[i];
                        if(std::isinf(range) || range < laserscan_output_.range_min || range > laserscan_output_.range_max)
                        {
                                continue;
                        }
                        else
                        {
                                tf2::Transform tf_laserscan_point, tf_map_point;
                                tf_laserscan_point.setIdentity();
                                double x, y, w_x, w_y;
                                x = std::cos(yaw) * range;
                                y = std::sin(yaw) * range;
                                tf_laserscan_point.setOrigin(tf2::Vector3(x,y,0));
                                tf_map_point = this->map_laser_link_tf * tf_laserscan_point;
                                w_x = tf_map_point.getOrigin().getX();
                                w_y = tf_map_point.getOrigin().getY();
                                MapPose map_pose = this->word_to_picture_bounded(w_x, w_y);
                                this->laserscan_points_vector.push_back(map_pose);
                                RCLCPP_INFO(this->get_logger(), "x: %d, y: %d", map_pose.x, map_pose.y);
                        }
                }

                auto img_map_clone = this->img_map_.clone();
                RCLCPP_INFO(this->get_logger(), "size: %zu", this->laserscan_points_vector.size());
                for(int i = 0; i< (int)this->laserscan_points_vector.size(); i++)
                {
                        MapPose map_pose = this->laserscan_points_vector[i];                        
                        img_map_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[0] = 0;
                        img_map_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[1] = 0;
                        img_map_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[0] = 255;
                }

                cv::namedWindow("laser_scan", cv::WindowFlags::WINDOW_NORMAL);
                cv::imshow("laser_scan", img_map_clone);
                cv::waitKey(10);
        }
}

sensor_msgs::msg::LaserScan WallLineDetection::process_laserscan_queue(std::queue<LaserScanMsg> queue)
{
      if (queue.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Error occurs, queue is empty.");        
      }

      LaserScanMsg output_msg = queue.back();
      size_t laserscan_size = output_msg.ranges.size() ;
      
      for (size_t i = 0; i < queue.size() - 1; i++)
      {
              LaserScanMsg tmp_msg = queue.front();
              for (size_t j = 0; j < laserscan_size; j++)
              {
                if (output_msg.ranges[j] > tmp_msg.ranges[j])
                {
                        output_msg.ranges[j] = tmp_msg.ranges[j];
                }                
              }
      }
      return output_msg;
}

void WallLineDetection::map_sub_callback_(const OccupancyGridMsg::SharedPtr msg)
{
        this->map_grid_ = *msg;
        this->map_width_ = this->map_grid_.info.width;
        this->map_height_ = this->map_grid_.info.height;
        this->map_origin_ = this->map_grid_.info.origin;
        this->map_resolution_ = this->map_grid_.info.resolution;
        RCLCPP_INFO(this->get_logger(), "map width: %d, height: %d", this->map_width_, this->map_height_);

        int channel = 3;
        auto data = this->map_grid_.data;
        uint8_t *data_ptr = new uint8_t[data.size() * channel];
        for (int i = 0; i < (int)data.size(); i++)
        {
                int row, col, index;
                row = this->map_height_ - 1 - i / this->map_width_;
                col = i % this->map_width_;
                index = (row * this->map_width_  + col) * channel;

                switch(data[i])
                {
                        case 0:
                        {
                                data_ptr[index] = 255;
                                data_ptr[index + 1] = 255;
                                data_ptr[index + 2] = 255;
                                break;
                        }
                        case 100:
                        {
                                data_ptr[index] = 0;
                                data_ptr[index + 1] = 0;
                                data_ptr[index + 2] = 0;
                                break;
                        }
                        case -1:
                        {
                                data_ptr[index] = 100;
                                data_ptr[index + 1] = 100;
                                data_ptr[index + 2] = 100;
                                break;
                        }
                        default:
                        {
                                data_ptr[index] = 255;
                                data_ptr[index + 1] = 255;
                                data_ptr[index + 2] = 255;
                        }
                }
                
        }

        this->img_map_ = cv::Mat(this->map_height_, this->map_width_, CV_8UC3, data_ptr);
        cv::namedWindow("map", cv::WindowFlags::WINDOW_NORMAL);
        // cv::imshow("map", this->img_map_);
        // cv::waitKey(0);
}

MapPose WallLineDetection::word_to_picture_bounded(double w_x, double w_y)
{
        MapPose map_pose;
        map_pose.x = (w_x - this->map_origin_.position.x) / this->map_resolution_;
        map_pose.y = (w_y - this->map_origin_.position.y) / this->map_resolution_;
        
        map_pose.x = std::min(std::max(0, map_pose.x), this->map_width_ - 1);
        map_pose.y = std::min(std::max(0, map_pose.y), this->map_height_ - 1);

        map_pose.y = this->map_height_ - 1 - map_pose.y;

        return map_pose;
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(wall_line_detection_pkg::WallLineDetection)

