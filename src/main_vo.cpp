/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
using namespace std;

#include "ratslam/utils.h"
#include "ratslam/visual_odometry.h"

#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


ratslam::VisualOdometry *vo = NULL;

using namespace ratslam;

class RatSLAMVisualOdometry : public rclcpp::Node
{
public:
  RatSLAMVisualOdometry() : rclcpp::Node("ratslam_visual_odometry")
  {
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/irat_red/camera/image/compressed", 10, std::bind(&RatSLAMVisualOdometry::image_callback, this, std::placeholders::_1));

    visual_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", 1);

      counter_ = 0;
  }

private:

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // msg (CompressedImage) does not have enconding neither width or height
    counter_++;
    RCLCPP_INFO(this->get_logger(), "VO:image_callback{%d}", counter_);

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

    vo->on_image(&msg->data[0], (image_msg->encoding == "bgr8" ? false : true), 
      image_msg->width, image_msg->height, &odom_output.twist.twist.linear.x, 
      &odom_output.twist.twist.angular.z);

    odom_output.header.stamp = this->now();

    visual_odom_pub_->publish(odom_output);
  }

  uint counter_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_odom_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_view_template = std::make_shared<RatSLAMVisualOdometry>();

  RCLCPP_INFO(ratslam_view_template->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  RCLCPP_INFO(ratslam_view_template->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  RCLCPP_INFO(ratslam_view_template->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    RCLCPP_FATAL(ratslam_view_template->get_logger(), "Missing <config_file>.");
    exit(-1);
  }

  std::string topic_root = "";

  boost::property_tree::ptree settings, general_settings, vo_settings;
  read_ini(argv[1], settings);
  ratslam::get_setting_child(vo_settings, settings, "visual_odometry", true);
  ratslam::get_setting_child(general_settings, settings, "general", true);
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  vo = new ratslam::VisualOdometry(vo_settings);

  rclcpp::spin(ratslam_view_template);
  rclcpp::shutdown();

  return 0;
}
