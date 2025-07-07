#include <iostream>
#include <chrono>
#include <fstream>
#include <memory>

#include "ratslam/utils.h"
#include <boost/property_tree/ini_parser.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <topological_msgs/msg/topological_action.hpp>
#include <topological_msgs/msg/view_template.hpp>

#include "ratslam/posecell_network.h"

//#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
ratslam::PosecellScene *pcs;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMPoseCells : public rclcpp::Node {
public:
    RatSLAMPoseCells() : Node("ratslam_pose_cells") {
        RCLCPP_INFO(this->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
        RCLCPP_INFO(this->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
        RCLCPP_INFO(this->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");
        
        std::string config_file;
        declare_parameter<std::string>("config_file", "");
        get_parameter("config_file", config_file);
      
        boost::property_tree::ptree settings, ratslam_settings, general_settings;
        read_ini(config_file, settings);
      
        std::string topic_root;
        get_setting_child(ratslam_settings, settings, "ratslam", true);
        get_setting_child(general_settings, settings, "general", true);
        get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

        pub_pc_ = this->create_publisher<topological_msgs::msg::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 10);

        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_root + "/odom", 10, std::bind(&RatSLAMPoseCells::odo_callback, this, std::placeholders::_1));

        sub_template_ = this->create_subscription<topological_msgs::msg::ViewTemplate>(
            topic_root + "/view_template", 10, std::bind(&RatSLAMPoseCells::template_callback, this, std::placeholders::_1));

        pc_ = std::make_unique<ratslam::PosecellNetwork>(ratslam_settings);

        boost::property_tree::ptree draw_settings;
        std::string media_path;
        this->declare_parameter<std::string>("media_path", "");
        this->get_parameter("media_path", media_path);
        std::string image_file;
        this->declare_parameter<std::string>("image_file", "");
        this->get_parameter("image_file", image_file);
        // this->declare_parameter<bool>("draw/enable", true);
        // this->get_parameter("draw/enable", use_graphics);
        get_setting_child(draw_settings, settings, "draw", true);
        get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
        if (use_graphics)
        {
            pcs = new ratslam::PosecellScene(draw_settings, pc_.get(), media_path, image_file);
        }

    }

private:
    void odo_callback(const nav_msgs::msg::Odometry::SharedPtr odo) {
        auto start = std::chrono::high_resolution_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "PC:odo_callback seq=%d v=%f r=%f",
                     odo->header.frame_id, odo->twist.twist.linear.x, odo->twist.twist.angular.z);

        static rclcpp::Time prev_time(0, 0, RCL_ROS_TIME);
        
        if (prev_time.nanoseconds() > 0) {
            rclcpp::Time odo_time(odo->header.stamp.sec, odo->header.stamp.nanosec, RCL_ROS_TIME);
            double time_diff = (odo_time - prev_time).seconds();
  
            pc_output_.src_id = pc_->get_current_exp_id();
            pc_->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
            pc_output_.action = pc_->get_action();

            if (pc_output_.action != ratslam::PosecellNetwork::NO_ACTION) {
                pc_output_.header.stamp = this->get_clock()->now();
                pc_output_.dest_id = pc_->get_current_exp_id();
                pc_output_.relative_rad = pc_->get_relative_rad();
                pub_pc_->publish(pc_output_);
            }

            //#ifdef HAVE_IRRLICHT
            if (use_graphics) {
                pcs->update_scene();
                pcs->draw_all();
            }
            //#endif
        }

        prev_time = odo->header.stamp;

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        std::ofstream outfile("tempo_execucao_odo_callback_pc.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << odo->header.frame_id << ". Tempo de execução (odo_callback) em pc: " << duration.count() << " ms" << std::endl;
        }
    }

    void template_callback(const topological_msgs::msg::ViewTemplate::SharedPtr vt) {
        auto start = std::chrono::high_resolution_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "PC:vt_callback seq=%d id=%d rad=%f",
                     vt->header.frame_id, vt->current_id, vt->relative_rad);

        pc_->on_view_template(vt->current_id, vt->relative_rad);

        //#ifdef HAVE_IRRLICHT
        if (use_graphics) {
            pcs->update_scene();
            pcs->draw_all();
        }
        //#endif

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        std::ofstream outfile("tempo_execucao_template_callback_pc.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << vt->header.frame_id << ". Tempo de execução (template_callback) em pc: " << duration.count() << " ms" << std::endl;
        }
    }

    std::unique_ptr<ratslam::PosecellNetwork> pc_;
    rclcpp::Publisher<topological_msgs::msg::TopologicalAction>::SharedPtr pub_pc_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<topological_msgs::msg::ViewTemplate>::SharedPtr sub_template_;
    topological_msgs::msg::TopologicalAction pc_output_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RatSLAMPoseCells>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
