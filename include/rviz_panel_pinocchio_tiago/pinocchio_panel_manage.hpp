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
#include "pinocchio/algorithm/rnea.hpp"
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
#include <QLineEdit>
#include <QHBoxLayout>
#include <QDoubleValidator>
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
  
    void setConfiguration(const std::vector<double>&); // set the initial configuration
    std::vector<std::string>  getConfiguration(); // get the configuration stored
    std::vector<std::string> getActiveJointsName(); // get the active joints names (e.g. not fixed)
    void performForwardKinematics(); // perform forward kinematics
    void performJacobian(const std::string& arm_link); // perform jacobian
    std::vector<std::string> performCollisionCheck(); // perform collision check
    std::string getFrameTransform(const std::string& arm_link); // get the frame transform ok a selected link
    std::vector<double> performTorqueEstimation(); // perform torque estimation
    void addPayloadMass(const double& mass); // add payload mass to the model
  
  private:
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::GeometryModel visual_model;
    pinocchio::GeometryModel collision_model;
    pinocchio::GeometryData collision_data;

    pinocchio::Model model_backup; // use to restore the model when a mass is added
    pinocchio::Data data_backup; // use to restore the model when a mass is added
    pinocchio::GeometryModel visual_model_backup; // use to restore the model when a mass is added
    pinocchio::GeometryModel collision_model_backup; // use to restore the model when a mass is added
    pinocchio::GeometryData collision_data_backup; // use to restore the model when a mass is added

    bool massAdded = false; // flag to check if mass is added

    Eigen::VectorXd q;
};



class PinocchioPanelManage : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PinocchioPanelManage(QWidget * parent = 0);
  ~PinocchioPanelManage() override;

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
  QPushButton * buttonTorque_; // button to calculate collision and other stuff
  QComboBox * dropdown_; // dropdown menu to select options
  QLabel * label_payload_; // label for payload mass input
  QLineEdit * input_payload_; // input field for payload mass
  QHBoxLayout * payload_layout; 
  
  double payload_mass_; // to store the payload mass value
  std::string jacobian_; // string to store jacobian
  std::string collision_pairs_; // string to store collision pairs
  std::vector<double> position_joints_;
  // obj for PinocchioManager
  PinocchioManager pinocchio_manager_obj_;


private Q_SLOTS:
  void buttonActivated();
  void buttonGetTorque();
};







}  // namespace rviz_panel_pinocchio_tiago

#endif  // RVIZ_PANEL_PINOCCHIO_TIAGO__DEMO_PANEL_HPP_
