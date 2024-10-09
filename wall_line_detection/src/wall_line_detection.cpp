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
        this->declare_parameter<double>("canny_thr1", 100.0);
        this->declare_parameter<double>("canny_thr2", 200.0);
        this->declare_parameter<double>("hough_rho", 1.0);
        this->declare_parameter<double>("hough_theta", M_PI / 180.0);
        this->declare_parameter<int>("hough_thr", 80);
        this->declare_parameter<double>("hough_min_line_length", 30.0);
        this->declare_parameter<double>("hough_max_line_gap", 10.0);
        this->declare_parameter<int>("imshow_wait", 10);
        this->declare_parameter<double>("vertical_thr", 5.0);
        this->declare_parameter<int>("horizontal_thr", 5);
        this->declare_parameter<double>("theta_thr", 0.087266);
        this->declare_parameter<bool>("imshow_map", false);
        this->declare_parameter<bool>("imshow_laser", false);
        this->declare_parameter<bool>("imshow_edge", false);
        this->declare_parameter<bool>("imshow_line", false);
        this->declare_parameter<int>("window_type", 0);

        this->laserscan_topic_sub_name = this->get_parameter_or<std::string>("laserscan_topic_sub_name", std::string("/scan"));
        this->laserscan_topic_pub_name = this->get_parameter_or<std::string>("laserscan_topic_pub_name", std::string("/scan_converged"));
        this->map_topic_sub_name = this->get_parameter_or<std::string>("map_topic_sub_name", std::string("/map"));
        this->laserscan_queue_size_  = this->get_parameter_or<int>("laserscan_queue_size", 5);
        this->base_link_frame = this->get_parameter_or<std::string>("base_link_frame", "base_link");
        this->laser_link_frame = this->get_parameter_or<std::string>("laser_link_frame", "laser_link");
        this->map_frame = this->get_parameter_or<std::string>("map_frame", "map");
        this->canny_thr1 = this->get_parameter_or<double>("canny_trh1", 100.0);
        this->canny_thr2 = this->get_parameter_or<double>("canny_trh2", 200.0);
        this->hough_rho = this->get_parameter_or<double>("hough_rho", 1.0);
        this->hough_theta = this->get_parameter_or<double>("hough_theta", M_PI / 180.0);
        this->hough_thr = this->get_parameter_or<int>("hough_thr", 80);
        this->hough_min_line_length = this->get_parameter_or<double>("hough_min_line_length", 30.0);
        this->hough_max_line_gap = this->get_parameter_or<double>("hought_max_line_gap", 10.0);
        this->imshow_wait = this->get_parameter_or<int>("imshow_wait", 10);
        this->vertical_thr = this->get_parameter_or<double>("vertical_thr", 5.0);
        this->horizontal_thr = this->get_parameter_or<int>("horizontal_thr", 5.0);
        this->theta_thr = this->get_parameter_or<double>("theta_thr", 0.087266);
        this->imshow_map = this->get_parameter_or<bool>("imshow_map", false);
        this->imshow_laser = this->get_parameter_or<bool>("imshow_laser", false);
        this->imshow_edge = this->get_parameter_or<bool>("imshow_edge", false);
        this->imshow_line = this->get_parameter_or<bool>("imshow_line", false);
        this->window_type = this->get_parameter_or<int>("window_type", 0);
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
                double yaw = this->laserscan_output_.angle_min;
                double yaw_delta = this->laserscan_output_.angle_increment;
                for (int i = 0; i < (int) this->laserscan_output_.ranges.size(); i++, yaw += yaw_delta)
                {
                        // RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);
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
                                // RCLCPP_INFO(this->get_logger(), "range: %f, yaw: %f", range, yaw);
                                // RCLCPP_INFO(this->get_logger(), "w_x: %f, w_y: %f", w_x, w_y);
                                // RCLCPP_INFO(this->get_logger(), "x: %d, y: %d", map_pose.x, map_pose.y);
                        }
                }

                auto img_map_empty_clone = this->img_map_empty_.clone();
                // RCLCPP_INFO(this->get_logger(), "size: %zu", this->laserscan_points_vector.size());
                for(int i = 0; i< (int)this->laserscan_points_vector.size(); i++)
                {
                        MapPose map_pose = this->laserscan_points_vector[i];                        
                        img_map_empty_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[0] = 0;
                        img_map_empty_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[1] = 0;
                        img_map_empty_clone.at<cv::Vec3b>(map_pose.y, map_pose.x)[2] = 255;
                }

                cv::Mat img_map_clone2 = img_map_empty_clone.clone();
                cv::Mat gray_img;
                cv::cvtColor(img_map_clone2, gray_img, cv::COLOR_BGR2GRAY);
                cv::Mat edges_img;
                cv::Canny(gray_img, edges_img, this->canny_thr1, this->canny_thr2);
                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP(edges_img, lines, this->hough_rho, this->hough_theta, this->hough_thr, this->hough_min_line_length, this->hough_max_line_gap);

                // process lines
                std::vector<LineInfo>().swap(this->lines_);
                for (size_t i = 0; i < lines.size(); i++)
                {
                        LineInfo line_info;
                        line_info.line = lines[i];
                        cv::Point2i pt1, pt2, pt0;
                        pt1.x = lines[i][0];
                        pt1.y = lines[i][1];
                        pt2.x = lines[i][2];
                        pt2.y = lines[i][3];
                        if (pt1.x > pt2.x)
                        {
                                pt1.x = pt1.x + pt2.x;
                                pt2.x = pt1.x - pt2.x;
                                pt1.x = pt1.x - pt2.x;

                                pt1.y = pt1.y + pt2.y;
                                pt2.y = pt1.y - pt2.y;
                                pt1.y = pt1.y - pt2.y;
                        }
                        line_info.theta = std::atan2(pt2.y - pt1.y, pt2.x - pt1.x);

                        pt0.x = 0, pt0.y = 0;
                        line_info.rho = calculate_height(pt0.x, pt0.y, pt1.x, pt1.y, pt2.x, pt2.y);

                        this->lines_.push_back(line_info);
                }

                std::sort(lines_.begin(), lines_.end(), [](LineInfo li1, LineInfo li2){return li1.theta < li2.theta;});

                this->lines_filter(this->lines_);
                
                size_t color_size = this->colors.size();
                cv::Mat img_map_empty_clone2 = this->img_map_empty_.clone();
                for (size_t i = 0; i < this->lines_.size(); i++)
                {
                        cv::Vec4i l = this->lines_[i].line;
                        cv::line(img_map_empty_clone2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), this->colors[ i % (color_size + 1)], 1, cv::LINE_AA);
                }

                if (this->imshow_laser)
                {
                        cv::namedWindow("laser_scan", this->window_type);
                        cv::imshow("laser_scan", img_map_empty_clone);
                }
                
                if (this->imshow_edge)
                {
                        cv::namedWindow("edges", this->window_type);
                        cv::imshow("edges", edges_img);
                }

                if (this->imshow_line)
                {
                        cv::namedWindow("lines", this->window_type);
                        cv::imshow("lines", img_map_empty_clone2);
                }

                if (this->imshow_laser || this->imshow_edge || this->imshow_line)
                {
                        cv::waitKey(this->imshow_wait);
                }
                
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
        RCLCPP_INFO(this->get_logger(), "map origin=> x: %f, y: %f.", this->map_origin_.position.x, this->map_origin_.position.y);
        RCLCPP_INFO(this->get_logger(), "map resolution: %f", this->map_resolution_);

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
        this->img_map_empty_ = cv::Mat(this->map_height_, this->map_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        
        if (this->imshow_map)
        {
                cv::namedWindow("map", this->window_type);
                cv::imshow("map", this->img_map_);
                cv::waitKey(10);
        }
        
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

void WallLineDetection::lines_filter(std::vector<LineInfo> &lines_infos)
{
        size_t size = lines_infos.size();
        RCLCPP_INFO(get_logger(), "\nbefore filter, size: %zu", size);
        int index = 0;
        for (auto iter = lines_infos.begin(); iter != lines_infos.end(); iter++)
        {
                RCLCPP_INFO(get_logger(), "line%d: [(%d, %d), (%d, %d)]", index,
                        iter->line[0], iter->line[1], iter->line[2], iter->line[3]);
                index++;
        }
        
        index = 0;
        for (auto iter1 = lines_infos.begin(); (iter1 != lines_infos.end() - 1) && (iter1 != lines_infos.end()); iter1++)
        {               
               RCLCPP_INFO(get_logger(), "----------- %d -----------", index);
               RCLCPP_INFO(get_logger(), "line%d: [(%d, %d), (%d, %d)]", index++,
                        iter1->line[0], iter1->line[1], iter1->line[2], iter1->line[3]);
               int index2 = 0;
               for (auto iter2 = iter1 + 1; iter2 != lines_infos.end();)
               {
                        RCLCPP_INFO(get_logger(), "line%d: [(%d, %d), (%d, %d)]", (index + index2++),
                                iter2->line[0], iter2->line[1], iter2->line[2], iter2->line[3]);
                        if (std::abs(iter1->theta - iter2->theta) > this->theta_thr)
                        {
                                RCLCPP_INFO(get_logger(), "break for theta threshold.");
                                break;
                        }
                        else
                        {
                                double vertical_distance = std::min(
                                        std::min(this->calculate_height(iter1->line[0], iter1->line[1], iter2->line[0], iter2->line[1], iter2->line[2], iter2->line[3]), 
                                                this->calculate_height(iter1->line[2], iter1->line[3], iter2->line[0], iter2->line[1], iter2->line[2], iter2->line[3])),
                                        this->calculate_height((iter1->line[0] + iter1->line[2]) / 2, 
                                        (iter1->line[1] + iter1->line[3]) / 2, iter2->line[0], iter2->line[1], iter2->line[2], iter2->line[3]));
                                if (vertical_distance > this->vertical_thr)
                                {
                                RCLCPP_INFO(get_logger(), "break for vertical disntance threshold.");
                                RCLCPP_INFO(get_logger(), "vertical_distance: %f, thre: %f", vertical_distance, this->vertical_thr);
                                break;
                                }
                                else
                                {
                                        cv::Point line1_pt1, line1_pt2, line2_pt1, line2_pt2;
                                        double theta1, theta2;
                                        double rho1, rho2;
                                        line1_pt1.x = iter1->line[0];
                                        line1_pt1.y = iter1->line[1];
                                        line1_pt2.x = iter1->line[2];
                                        line1_pt2.y = iter1->line[3];
                                        theta1 = iter1->theta;
                                        rho1 = iter1->rho;

                                        line2_pt1.x = iter2->line[0];
                                        line2_pt1.y = iter2->line[1];
                                        line2_pt2.x = iter2->line[2];
                                        line2_pt2.y = iter2->line[3];
                                        theta2 = iter2->theta;
                                        rho2 = iter2->rho;

                                        cv::Point line1_pt1_copy, line1_pt2_copy;
                                        cv::Point line2_pt1_copy, line2_pt2_copy;
                                        line1_pt1_copy = line1_pt1;
                                        line1_pt2_copy = line1_pt2;
                                        line2_pt1_copy = line2_pt1;
                                        line2_pt2_copy = line2_pt2;

                                        bool merge_occurred = this->merge_lines(line1_pt1, line1_pt2, line2_pt1, line2_pt2, (theta1 + theta2) / 2.0);

                                        if(merge_occurred)
                                        {
                                                (*iter1).line[0] = line1_pt1.x;
                                                (*iter1).line[1] = line1_pt1.y;
                                                (*iter1).line[2] = line1_pt2.x;
                                                (*iter1).line[3] = line1_pt2.y;
                                                RCLCPP_INFO(get_logger(), "[(%d, %d), (%d, %d)]", line1_pt1.x, line1_pt1.y, line1_pt2.x, line1_pt2.y);
                                                RCLCPP_INFO(get_logger(), "merge line [(%d, %d), (%d, %d)] and line [(%d, %d), (%d, %d)] to line [(%d, %d), (%d, %d)].",
                                                        line1_pt1_copy.x, line1_pt1_copy.y, line1_pt2_copy.x, line1_pt2_copy.y,
                                                        line2_pt1_copy.x, line2_pt1_copy.y, line2_pt2_copy.x, line2_pt2_copy.y,
                                                        iter1->line[0], iter1->line[1], iter1->line[2], iter1->line[3]);
                                                RCLCPP_INFO(get_logger(), "line1 => rho: %f, theta: %f", rho1, theta1);
                                                RCLCPP_INFO(get_logger(), "line2 => rho: %f, theta: %f", rho2, theta2);
                                                iter2 = lines_infos.erase(iter2);
                                                if (iter2 == lines_infos.end())
                                                {
                                                        break;
                                                }
                                        }
                                        else
                                        {
                                                RCLCPP_INFO(get_logger(), "line [(%d, %d), (%d, %d)] and line [(%d, %d), (%d, %d)] are different lines.",
                                                        line1_pt1_copy.x, line1_pt1_copy.y, line1_pt2_copy.x, line1_pt2_copy.y,
                                                        line2_pt1_copy.x, line2_pt1_copy.y, line2_pt2_copy.x, line2_pt2_copy.y);
                                                iter2++;
                                        }
                                }
                        }                        
                }
        }
        size = lines_infos.size();
        RCLCPP_INFO(get_logger(), "after filter, size: %zu", size);
        index = 0;
        for (auto iter = lines_infos.begin(); iter != lines_infos.end(); iter++)
        {
                RCLCPP_INFO(get_logger(), "line%d: [(%d, %d), (%d, %d)]", index,
                        iter->line[0], iter->line[1], iter->line[2], iter->line[3]);
                index++;
        }
}

bool  WallLineDetection::merge_lines(cv::Point2i& pt1, cv::Point2i& pt2, cv::Point2i pt3, cv::Point2i pt4, double theta)
{
        bool ret = false;
        
        int delta_x, delta_y;
        delta_x = std::abs(pt1.x - pt2.x);
        delta_y = std::abs(pt1.y - pt2.y);

        if (delta_x > delta_y) // major axis => X
        {
                if (pt1.x < pt3.x)
                {
                        if (pt3.x < pt2.x + this->horizontal_thr / cos(theta))  // Meet the merger conditions
                        {
                                // start point, assignment
                                pt1 = pt1; // just for understand clearly
                                // end point, assignment
                                pt2 = pt2.x > pt4.x ? pt2 : pt4;
                                
                                ret = true;                          
                        }
                        else
                        {
                                ret = false; // don't chang anything
                        }
                }
                else
                {
                        if (pt1.x < pt4.x + this->horizontal_thr / cos(theta)) // Meet the merger conditions
                        {
                                // start point, assignment
                                pt1 = pt3; 
                                // end point, assignment
                                pt2 = pt2.x > pt4.x ? pt2 : pt4; 

                                ret = true;
                        }
                        else
                        {
                                ret = false; // don't chang anything
                        }
                }
        }
        else // major axis => Y
        {
                if (pt1.y < pt3.y)
                {
                        if (pt3.y < pt2.y + this->horizontal_thr / sin(theta))  // Meet the merger conditions
                        {
                                // start point, assignment
                                pt1 = pt1; // just for understand clearly
                                // end point, assignment
                                pt2 = pt2.y > pt4.y ? pt2 : pt4;   

                                ret = true;                             
                        }
                        else
                        {
                                ret = false; // don't chang anything
                        }
                }
                else
                {
                        if (pt1.y < pt4.y + this->horizontal_thr / sin(theta)) // Meet the merger conditions
                        {
                                // start point, assignment
                                pt1 = pt3; 
                                // end point, assignment
                                pt2 = pt2.y > pt4.y ? pt2 : pt4; 

                                ret = true;
                        }
                        else
                        {
                                ret = false; // don't chang anything
                        }
                }
        }

        return ret;
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(wall_line_detection_pkg::WallLineDetection)

