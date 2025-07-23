#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef RATSLAM_ROS2_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include "ratslam/utils.h"
#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <topological_msgs/msg/view_template.hpp>

#include "ratslam/local_view_match.h"

//#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene *lvs = nullptr;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMViewTemplate : public rclcpp::Node {
public:
    RatSLAMViewTemplate() : Node("ratslam_view_template") {
        RCLCPP_INFO(this->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
        RCLCPP_INFO(this->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
        RCLCPP_INFO(this->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

        declare_parameter<std::string>("config_file", "");
        std::string config_file;
        get_parameter("config_file", config_file);

        std::string topic_root;
        boost::property_tree::ptree settings, general_settings, ratslam_settings;
        read_ini(config_file, settings);
        get_setting_child(general_settings, settings, "general", true);
        get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
        get_setting_child(ratslam_settings, settings, "ratslam", true);

        lv_ = std::make_unique<ratslam::LocalViewMatch>(ratslam_settings);

        pub_vt_ = this->create_publisher<topological_msgs::msg::ViewTemplate>(
            topic_root + "/view_template", 10);

        sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic_root + "/camera/image/compressed", 10,
            std::bind(&RatSLAMViewTemplate::image_callback, this, std::placeholders::_1));

        //#ifdef HAVE_IRRLICHT
        // this->declare_parameter<bool>("draw/enable", true);
        // this->get_parameter("draw/enable", use_graphics);
        boost::property_tree::ptree draw_settings;
        get_setting_child(draw_settings, settings, "draw", true);
        get_setting_from_ptree(use_graphics, draw_settings, "enable", true);

        if (use_graphics) {
            lvs = new ratslam::LocalViewScene(draw_settings, lv_.get());
        }
        //#endif
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
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
        topological_msgs::msg::ViewTemplate vt_output;
        lv_->on_image(&image_msg->data[0], (image_msg->encoding == "bgr8" ? false : true), 
          image_msg->width, image_msg->height);

        vt_output.header.stamp = this->get_clock()->now();
        vt_output.current_id = lv_->get_current_vt();
        vt_output.relative_rad = lv_->get_relative_rad();

        pub_vt_->publish(vt_output);

        //#ifdef HAVE_IRRLICHT
        if (use_graphics) {
            lvs->draw_all();
        }
        //#endif
    }

    std::unique_ptr<ratslam::LocalViewMatch> lv_;
    rclcpp::Publisher<topological_msgs::msg::ViewTemplate>::SharedPtr pub_vt_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RatSLAMViewTemplate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
