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
#include <chrono>
#include <fstream>
using namespace std;

#include "ratslam/utils.h"

#include <boost/property_tree/ini_parser.hpp>


#include <rclcpp/rclcpp.hpp>
#include <topological_msgs/msg/topological_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <topological_msgs/msg/view_template.hpp>
#include "ratslam/posecell_network.h"


//#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
ratslam::PosecellScene *pcs;
bool use_graphics;
//#endif

std::ofstream outfile_vt("tempo_execucao_ros2_template_callback_pc.txt");
std::ofstream outfile_odo("tempo_execucao_ros2_odo_callback_pc.txt");
using namespace ratslam;

class RatSLAMPoseCells : public rclcpp::Node
{
public:
  RatSLAMPoseCells() 
    : Node("ratslam_pose_cells"), first_callback_(true)
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
    get_setting_child(ratslam_settings, settings, "ratslam", true);
    get_setting_child(general_settings, settings, "general", true);
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
  
    //#ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    
    setPC(ratslam_settings);
    if (use_graphics)
    {
      pcs = new ratslam::PosecellScene(draw_settings, 
        getPC());
    }
    //#endif

    pc_pub_ = this->create_publisher<topological_msgs::msg::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 1);
  
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_root + "/odom", 10, std::bind(&RatSLAMPoseCells::odo_callback, this, std::placeholders::_1));

    template_sub_ = this->create_subscription<topological_msgs::msg::ViewTemplate>(
      topic_root + "/LocalView/Template", 10, std::bind(&RatSLAMPoseCells::template_callback, this, std::placeholders::_1));

    counter_odo_ = 0;
    counter_vt_ = 0;
  }

  void setPC(boost::property_tree::ptree settings)
  {
    pc_ = new ratslam::PosecellNetwork(settings);
  }

  ratslam::PosecellNetwork * getPC() 
  {
    return pc_;
  }

private:
  void odo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Captura o tempo inicial
    auto start = std::chrono::high_resolution_clock::now();

    // Calculate the sampling time of odo_callback function
    auto current_time = this->now();
    double time_diff;
    if (!first_callback_) {
      auto current_time = this->now();
      time_diff = (double) (current_time.nanoseconds() - previous_time_.nanoseconds());
      time_diff = time_diff / 1000000000.0; // seconds
      // RCLCPP_INFO(this->get_logger(), 
      //   "Duration since last callback: %lf s", time_diff);
      previous_time_  = current_time;

      double linear_velocity = msg->twist.twist.linear.x;
      double angular_velocity = msg->twist.twist.angular.z;
      // RCLCPP_INFO(this->get_logger(), 
      //   "PC:odo_callback{linear velocity: %f, angular velocity: %f}", linear_velocity, 
      //   angular_velocity);

      // Execute odometry (motion model)
      pc_->on_odo(linear_velocity, angular_velocity, time_diff);

      topological_msgs::msg::TopologicalAction pc_output;
      pc_output.src_id = pc_->get_current_exp_id();
      
      pc_output.action = pc_->get_action();
      if (pc_output.action != ratslam::PosecellNetwork::NO_ACTION)
      {
        pc_output.header.stamp = this->now();
        pc_output.dest_id = pc_->get_current_exp_id();
        pc_output.relative_rad = pc_->get_relative_rad();
        
        // RCLCPP_INFO(this->get_logger(), 
        //   "PC:action = %d, src = %u, dest = %u", pc_output.action, 
        //   pc_output.src_id, pc_output.dest_id);

        pc_pub_->publish(pc_output);
      }


      //#ifdef HAVE_IRRLICHT
      if (use_graphics)
      {
        pcs->update_scene();
        pcs->draw_all();
      }
    } else {
      first_callback_ = false;
      previous_time_ = this->now();
    }

    auto end = std::chrono::high_resolution_clock::now();
    // Calcula a duração e exibe em milissegundos
    std::chrono::duration<double, std::milli> duration = end - start;
    counter_odo_++;
    // Abre um arquivo para escrita
    if (outfile_odo.is_open()) {
      outfile_odo << counter_odo_ << ". Tempo de execução (odo_callback) em pc: " << duration.count() << " ms" << std::endl;
    }
  }

  void template_callback(const topological_msgs::msg::ViewTemplate::SharedPtr msg)
  {
    // Captura o tempo inicial
    auto start = std::chrono::high_resolution_clock::now();

    // RCLCPP_INFO(this->get_logger(), 
    //   "PC:vt_callback{ id = %d, rad = %f", msg->current_id, msg->relative_rad);

    pc_->on_view_template(msg->current_id, msg->relative_rad);

  //#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      pcs->update_scene();
      pcs->draw_all();
    }
  //#endif

    // Captura o tempo final
    auto end = std::chrono::high_resolution_clock::now();
    // Calcula a duração e exibe em milissegundos
    std::chrono::duration<double, std::milli> duration = end - start;
    counter_vt_++;
    // Abre um arquivo para escrita
    if (outfile_vt.is_open()) {
      outfile_vt << counter_vt_ << ". Tempo de execução (template_callback) em pc: " << duration.count() << " ms" << std::endl;
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Subscription<topological_msgs::msg::ViewTemplate>::SharedPtr template_sub_;
  rclcpp::Publisher<topological_msgs::msg::TopologicalAction>::SharedPtr pc_pub_;
  rclcpp::Time previous_time_;
  bool first_callback_;
  ratslam::PosecellNetwork * pc_;
  uint counter_odo_;
  uint counter_vt_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_pose_cells = std::make_shared<RatSLAMPoseCells>();
  rclcpp::spin(ratslam_pose_cells);
  rclcpp::shutdown();

  return 0;
}
