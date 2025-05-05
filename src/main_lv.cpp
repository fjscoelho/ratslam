/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
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
#include <chrono>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <sensor_msgs/image_encodings.h>

#include "ratslam/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <topological_msgs/msg/view_template.hpp>

#include <image_transport/image_transport.hpp>

#include "ratslam/local_view_match.h"

// #if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
// #endif

std::ofstream outfile("tempo_execucao_ros2_image_callback_lv.txt");
using namespace ratslam;
ratslam::LocalViewMatch * lv = NULL;


class RatSLAMViewTemplate : public rclcpp::Node
{
public:
  RatSLAMViewTemplate() : rclcpp::Node("ratslam_view_template")
  {
    RCLCPP_INFO(get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    RCLCPP_INFO(get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    RCLCPP_INFO(get_logger(), "Distributed under the GNU GPL v3, see the included license file.");
  
    std::string config_file;
    declare_parameter<std::string>("config_file", "");
    get_parameter("config_file", config_file);

    boost::property_tree::ptree settings, ratslam_settings, general_settings;
    read_ini(config_file, settings);
  
    std::string topic_root;
    get_setting_child(general_settings, settings, "general", true);
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
    get_setting_child(ratslam_settings, settings, "ratslam", true);

    // #ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  
    setLV(ratslam_settings);
    if (use_graphics) {
      lvs = new ratslam::LocalViewScene(draw_settings, 
        getLV());
    }
    // #endif

    // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //       "/camera/image", 10, std::bind(&RatSLAMViewTemplate::image_callback, this, std::placeholders::_1));

    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10, std::bind(&RatSLAMViewTemplate::image_callback, this, std::placeholders::_1));

    view_template_pub_ = this->create_publisher<topological_msgs::msg::ViewTemplate>(
      topic_root + "/LocalView/Template", 1);

    counter_ = 0;
  }

  void setLV(boost::property_tree::ptree settings)
  {
    lv_ = new ratslam::LocalViewMatch(settings);
  }

  ratslam::LocalViewMatch * getLV() 
  {
    return lv_;
  }

private:

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Captura o tempo inicial
    auto start = std::chrono::high_resolution_clock::now();
    // msg (CompressedImage) does not have enconding neither width or height
    counter_++;
    // RCLCPP_INFO(this->get_logger(), "LV:image_callback{%d}", counter_);

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

    lv_->on_image(&image_msg->data[0], (image_msg->encoding == "bgr8" ? false : true), image_msg->width, image_msg->height);

    vt_output_.header.stamp = this->get_clock()->now();
    // vt_output_.header.seq++;
    vt_output_.current_id = lv_->get_current_vt();
    vt_output_.relative_rad = lv_->get_relative_rad();
    vt_output_.feature = lv_->get_data();
    vt_output_.elapsed_time = lv_->get_elapsed_time();

    view_template_pub_->publish(vt_output_);

    //#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      lvs->draw_all();
    }

    // Captura o tempo final
    auto end = std::chrono::high_resolution_clock::now();
    // Calcula a duração e exibe em milissegundos
    std::chrono::duration<double, std::milli> duration = end - start;
    // Abre um arquivo para escrita
    if (outfile.is_open()) {
        outfile << counter_ << ". Tempo de execução (image_callback) em lv: " << duration.count() << " ms" << std::endl;
    }
  }

  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<topological_msgs::msg::ViewTemplate>::SharedPtr view_template_pub_;
  topological_msgs::msg::ViewTemplate vt_output_;
  uint counter_;
  ratslam::LocalViewMatch *lv_;
  // image_transport::ImageTransport it(node);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_view_template = std::make_shared<RatSLAMViewTemplate>();
  rclcpp::spin(ratslam_view_template);
  rclcpp::shutdown();

  return 0;
}