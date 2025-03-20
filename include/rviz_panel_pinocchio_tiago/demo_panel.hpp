/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#ifndef RVIZ_PANEL_PINOCCHIO_TIAGO__DEMO_PANEL_HPP_
#define RVIZ_PANEL_PINOCCHIO_TIAGO__DEMO_PANEL_HPP_

#include <filesystem>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/collision/collision.hpp" 
#include "pinocchio/collision/distance.hpp" 
#include "ament_index_cpp/get_package_share_directory.hpp"
#define PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>


namespace rviz_panel_pinocchio_tiago
{


class PinocchioManager {
  public:
    PinocchioManager() = default;

    PinocchioManager(const std::string& urdf_xml, const std::string& package_share_directory);
  
    void setConfiguration(const std::vector<double>);
    std::vector<std::string>  getConfiguration();
    void performForwardKinematics();
    void performJacobian(const std::string& arm_link);
    std::vector<std::string> performCollisionCheck();
    std::string getFrameTransform(const std::string& arm_link);
  
  private:
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::GeometryModel visual_model;
    pinocchio::GeometryModel collision_model;
    pinocchio::GeometryData collision_data;
    Eigen::VectorXd q;
};



class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit DemoPanel(QWidget * parent = 0);
  ~DemoPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // subscriber to joint_state topic
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state_;

  void topicCallback(const std_msgs::msg::String& msg); // callback for topic robot_description

  void topicJointStateCallback(const sensor_msgs::msg::JointState& msg);

  QLabel * label_titleframe_transform_; // label to show possible collisions
  QLabel * label_frame_transform_; // label to show possible collisions
  QLabel * label_title_collision_; // label to show possible collisions
  QLabel * label_collision_; // label to show possible collisions
  QPushButton * button_; // button to calculate collision and other stuff
  QComboBox * dropdown_; // dropdown menu to select options
  

  std::string jacobian_; // string to store jacobian
  std::string collision_pairs_; // string to store collision pairs

  // obj for PinocchioManager
  PinocchioManager pinocchio_manager_obj_;


private Q_SLOTS:
  void buttonActivated();
  
};







}  // namespace rviz_panel_pinocchio_tiago

#endif  // RVIZ_PANEL_PINOCCHIO_TIAGO__DEMO_PANEL_HPP_
