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

#include "ratslam/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include "ratslam/experience_map.h"

#include <rclcpp/rclcpp.hpp>
#include <topological_msgs/msg/topological_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "graphics/experience_map_scene.h"
#include <topological_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker.hpp>

//#ifdef HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
ratslam::ExperienceMapScene *ems;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMExperienceMap : public rclcpp::Node
{
public:
  RatSLAMExperienceMap()
    : Node("ratslam_experience_map"), first_callback_(true)
  {
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/irat_red/odom", 10, std::bind(
        &RatSLAMExperienceMap::odo_callback, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<topological_msgs::msg::TopologicalAction>(
      "/PoseCell/TopologicalAction", 1, std::bind(
      &RatSLAMExperienceMap::action_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ExperienceMap/SetGoalPose", 1, std::bind(
      &RatSLAMExperienceMap::set_goal_pose_callback, this, std::placeholders::_1));
                                                                

    em_pub_ = this->create_publisher<topological_msgs::msg::TopologicalMap>(
      "/PoseCell/TopologicalMap", 1);

    goal_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/ExperienceMap/PathToGoal", 1);

    em_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/ExperienceMap/MapMarker", 1);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ExperienceMap/RobotPose", 1);

  }

  void setEM(boost::property_tree::ptree settings)
  {
    em_ = new ratslam::ExperienceMap(settings);
  }

  ratslam::ExperienceMap * getEM() 
  {
    return em_;
  }
private:
  void odo_callback(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), 
      "EM:odo_callback{ linear velocity = %f, angular velocity = %f", 
      msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    // Calculate the sampling time of odo_callback function
    auto current_time = this->now();
    double time_diff;
      
    if (!first_callback_) {
      auto current_time = this->now();
      time_diff = (double) (current_time.nanoseconds() - previous_time_.nanoseconds());
      time_diff = time_diff / 1000000000.0; // seconds
      RCLCPP_INFO(this->get_logger(), 
        "Duration since last callback: %lf s", time_diff);
      previous_time_  = current_time;

      double linear_velocity = msg->twist.twist.linear.x;
      double angular_velocity = msg->twist.twist.angular.z;
  
      // Execute odometry (motion model)
      em_->on_odo(linear_velocity, angular_velocity, time_diff);

      if (em_->get_current_goal_id() >= 0)
      {

        em_->calculate_path_to_goal(msg->header.stamp.sec);

        nav_msgs::msg::Path path;
        if (em_->get_current_goal_id() >= 0)
        {
          em_->get_goal_waypoint();

          geometry_msgs::msg::PoseStamped pose;
          path.header.stamp = this->now();
          path.header.frame_id = "1";

          pose.header.frame_id = "1";
          path.poses.clear();
          unsigned int trace_exp_id = em_->get_goals()[0];
          while (trace_exp_id != em_->get_goal_path_final_exp())
          {
            pose.pose.position.x = em_->get_experience(trace_exp_id)->x_m;
            pose.pose.position.y = em_->get_experience(trace_exp_id)->y_m;
            path.poses.push_back(pose);

            trace_exp_id = em_->get_experience(trace_exp_id)->goal_to_current;
          }

          goal_path_pub_->publish(path);

        }
        else
        {
          path.header.stamp = this->now();
          path.header.frame_id = "1";
          path.poses.clear();
          goal_path_pub_->publish(path);
        }
      }
    } else {
      first_callback_ = false;
      previous_time_ = this->now();
    }
  }

  void action_callback(topological_msgs::msg::TopologicalAction::SharedPtr action)
  {
    RCLCPP_INFO(this->get_logger(), 
      "EM:action_callback{action = %d, src = %d, dst = %d", 
      action->action, action->src_id, action->dest_id);

    switch (action->action)
    {
      case topological_msgs::msg::TopologicalAction::CREATE_NODE:
        em_->on_create_experience(action->dest_id);
        em_->on_set_experience(action->dest_id, 0);
        break;

      case topological_msgs::msg::TopologicalAction::CREATE_EDGE:
        em_->on_create_link(action->src_id, action->dest_id, action->relative_rad);
        em_->on_set_experience(action->dest_id, action->relative_rad);
        break;

      case topological_msgs::msg::TopologicalAction::SET_NODE:
        em_->on_set_experience(action->dest_id, action->relative_rad);
        break;

    }

    em_->iterate();

    geometry_msgs::msg::PoseStamped pose_output;
    pose_output.header.stamp = this->now();
    pose_output.header.frame_id = "1";
    pose_output.pose.position.x = em_->get_experience(em_->get_current_id())->x_m;
    pose_output.pose.position.y = em_->get_experience(em_->get_current_id())->y_m;
    pose_output.pose.position.z = 0;
    pose_output.pose.orientation.x = 0;
    pose_output.pose.orientation.y = 0;
    pose_output.pose.orientation.z = sin(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
    pose_output.pose.orientation.w = cos(em_->get_experience(em_->get_current_id())->th_rad / 2.0);
    pose_pub_->publish(pose_output);

    double duration = 30.0;

    // Extract timestamp from the message header
    rclcpp::Time msg_time(action->header.stamp);
    // Get current node ROS time
    static rclcpp::Time current_time = this->now();
    // Calculate the time difference
    rclcpp::Duration time_diff = msg_time - current_time;

    RCLCPP_INFO(this->get_logger(), 
          "Timer counter = %f", time_diff.seconds());

    if (time_diff.seconds() > duration)
    {
      current_time = msg_time;

      topological_msgs::msg::TopologicalMap em_map;
      em_map.header.stamp = this->now();
      em_map.node_count = em_->get_num_experiences();
      em_map.node.resize(em_->get_num_experiences());
      for (int i = 0; i < em_->get_num_experiences(); i++)
      {
        em_map.node[i].id = em_->get_experience(i)->id;
        em_map.node[i].pose.position.x = em_->get_experience(i)->x_m;
        em_map.node[i].pose.position.y = em_->get_experience(i)->y_m;
        em_map.node[i].pose.orientation.x = 0;
        em_map.node[i].pose.orientation.y = 0;
        em_map.node[i].pose.orientation.z = sin(em_->get_experience(i)->th_rad / 2.0);
        em_map.node[i].pose.orientation.w = cos(em_->get_experience(i)->th_rad / 2.0);
      }

      em_map.edge_count = em_->get_num_links();
      em_map.edge.resize(em_->get_num_links());
      for (int i = 0; i < em_->get_num_links(); i++)
      {
        em_map.edge[i].source_id = em_->get_link(i)->exp_from_id;
        em_map.edge[i].destination_id = em_->get_link(i)->exp_to_id;
        em_map.edge[i].duration = rclcpp::Duration::from_seconds(em_->get_link(i)->delta_time_s);
        em_map.edge[i].transform.translation.x = em_->get_link(i)->d * cos(em_->get_link(i)->heading_rad);
        em_map.edge[i].transform.translation.y = em_->get_link(i)->d * sin(em_->get_link(i)->heading_rad);
        em_map.edge[i].transform.rotation.x = 0;
        em_map.edge[i].transform.rotation.y = 0;
        em_map.edge[i].transform.rotation.z = sin(em_->get_link(i)->facing_rad / 2.0);
        em_map.edge[i].transform.rotation.w = cos(em_->get_link(i)->facing_rad / 2.0);
      }
      em_pub_->publish(em_map);
    }

    visualization_msgs::msg::Marker em_marker;
    em_marker.header.stamp = this->now();
    em_marker.header.frame_id = "1";
    em_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    em_marker.points.resize(em_->get_num_links() * 2);
    em_marker.action = visualization_msgs::msg::Marker::ADD;
    em_marker.scale.x = 0.01;
    //em_marker.scale.y = 1;
    //em_marker.scale.z = 1;
    em_marker.color.a = 1;
    em_marker.ns = "em";
    em_marker.id = 0;
    em_marker.pose.orientation.x = 0;
    em_marker.pose.orientation.y = 0;
    em_marker.pose.orientation.z = 0;
    em_marker.pose.orientation.w = 1;
    for (int i = 0; i < em_->get_num_links(); i++)

    {
      em_marker.points[i * 2].x = em_->get_experience(em_->get_link(i)->exp_from_id)->x_m;
      em_marker.points[i * 2].y = em_->get_experience(em_->get_link(i)->exp_from_id)->y_m;
      em_marker.points[i * 2].z = 0;
      em_marker.points[i * 2 + 1].x = em_->get_experience(em_->get_link(i)->exp_to_id)->x_m;
      em_marker.points[i * 2 + 1].y = em_->get_experience(em_->get_link(i)->exp_to_id)->y_m;
      em_marker.points[i * 2 + 1].z = 0;
    }

    em_markers_pub_->publish(em_marker);

  //#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      ems->update_scene();
      ems->draw_all();
    }
  //#endif
  }

  void set_goal_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    em_->add_goal(pose->pose.position.x, pose->pose.position.y);

  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Subscription<topological_msgs::msg::TopologicalAction>::SharedPtr action_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<topological_msgs::msg::TopologicalMap>::SharedPtr em_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr goal_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr em_markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Time previous_time_;
  bool first_callback_;
  ratslam::ExperienceMap *em_;
  double timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_experience_map = std::make_shared<RatSLAMExperienceMap>();

  RCLCPP_INFO(ratslam_experience_map->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  RCLCPP_INFO(ratslam_experience_map->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  RCLCPP_INFO(ratslam_experience_map->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

  std::string config_file;
  ratslam_experience_map->declare_parameter<std::string>("config_file", "");
  ratslam_experience_map->get_parameter("config_file", config_file);

  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(config_file, settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");

//#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);

  ratslam_experience_map->setEM(ratslam_settings);

  if (use_graphics)
  {
    ems = new ratslam::ExperienceMapScene(draw_settings, 
      ratslam_experience_map->getEM());
  }
//#endif

  rclcpp::spin(ratslam_experience_map);
  rclcpp::shutdown();

  return 0;
}

