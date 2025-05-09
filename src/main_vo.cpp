#include <fstream>
#include <iostream>
using namespace std;

#include "ratslam/utils.h"
#include "ratslam/visual_odometry.h"

#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace ratslam;

class RatSLAMVisualOdometry : public rclcpp::Node
{
public:
  RatSLAMVisualOdometry() : rclcpp::Node("ratslam_visual_odometry")
  {
    RCLCPP_INFO(get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    RCLCPP_INFO(get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    RCLCPP_INFO(get_logger(), "Distributed under the GNU GPL v3, see the included license file.");
  
    std::string config_file;
    declare_parameter<std::string>("config_file", "");
    get_parameter("config_file", config_file);
  
    boost::property_tree::ptree settings, general_settings, vo_settings;
    read_ini(config_file, settings);

    std::string topic_root;
    ratslam::get_setting_child(vo_settings, settings, "visual_odometry", true);
    ratslam::get_setting_child(general_settings, settings, "general", true);
    ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
  
    vo_ = new ratslam::VisualOdometry(vo_settings);
  
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10, std::bind(&RatSLAMVisualOdometry::image_callback, this, std::placeholders::_1));

    visual_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      topic_root + "/odom", 1);

      counter_ = 0;
  }

private:

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // msg (CompressedImage) does not have enconding neither width or height
    counter_++;
    // RCLCPP_INFO(this->get_logger(), "VO:image_callback{%d}", counter_);

    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::msg::Image::SharedPtr image_msg;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    image_msg = cv_ptr->toImageMsg();
    nav_msgs::msg::Odometry odom_output;
    
    vo_->on_image(&image_msg->data[0], (image_msg->encoding == "bgr8" ? false : true), 
      image_msg->width, image_msg->height, &odom_output.twist.twist.linear.x, 
      &odom_output.twist.twist.angular.z);

    odom_output.header.stamp = image_msg->header.stamp;

    visual_odom_pub_->publish(odom_output);
  }

  uint counter_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_odom_pub_;
  ratslam::VisualOdometry *vo_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_visual_odometry = std::make_shared<RatSLAMVisualOdometry>();
  rclcpp::spin(ratslam_visual_odometry);
  rclcpp::shutdown();

  return 0;
}
