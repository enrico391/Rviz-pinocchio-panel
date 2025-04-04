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

/* Modified by: Enrico Moro*/

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_panel_pinocchio_tiago/pinocchio_panel_manage.hpp>


namespace rviz_panel_pinocchio_tiago
{
PinocchioPanelManage::PinocchioPanelManage(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);

  label_frame_transform_ = new QLabel("");
  label_collision_ = new QLabel("");

  label_titleframe_transform_ = new QLabel("Frame Transform");
  label_title_collision_ = new QLabel("Collision Pairs");

  button_ = new QPushButton("Calculate collision pairs");
  buttonTorque_ = new QPushButton("Calculate Torque");
  dropdown_ = new QComboBox();

  // horizontal layout for the payload section
  payload_layout = new QHBoxLayout();
  label_payload_ = new QLabel("Payload mass (kg):");
  // Create and set up the input field
  input_payload_ = new QLineEdit("0.0");
  input_payload_->setFixedWidth(80); // Set a reasonable width
  input_payload_->setAlignment(Qt::AlignRight); // Right-align the text


  layout->addWidget(label_titleframe_transform_);
  layout->addWidget(label_frame_transform_);
  layout->addWidget(label_title_collision_);
  layout->addWidget(label_collision_);
  layout->addWidget(button_);
  layout->addWidget(buttonTorque_);
  layout->addWidget(dropdown_);

  // Add the widgets to the horizontal layout
  payload_layout->addWidget(label_payload_);
  payload_layout->addWidget(input_payload_);
  payload_layout->addStretch(); // This pushes the widgets to the left
  // add to the main layout
  layout->addLayout(payload_layout);

  // disable buttons 
  button_->setEnabled(false);
  buttonTorque_->setEnabled(false);

  // Create a PinocchioManager object
  PinocchioManager pinocchio_manager_obj_;

  

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &PinocchioPanelManage::buttonActivated);
  QObject::connect(buttonTorque_, &QPushButton::released, this, &PinocchioPanelManage::buttonGetTorque);
  // Connect event payload changes 
  QObject::connect(input_payload_, &QLineEdit::textChanged, this, [this](const QString& text) {
    payload_mass_ = text.toDouble(); // This will convert to 0.0 if text is invalid
  });

}

PinocchioPanelManage::~PinocchioPanelManage() = default;

void PinocchioPanelManage::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  
  //define robot description subscriber
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/robot_description", 10, std::bind(&PinocchioPanelManage::topicCallback, this, std::placeholders::_1));
  
  //define joint state subscriber
  subscription_joint_state_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&PinocchioPanelManage::topicJointStateCallback, this, std::placeholders::_1));
}

void PinocchioPanelManage::topicJointStateCallback(const sensor_msgs::msg::JointState& msg)
{
  // std::cout << "Joint names: ";
  // for (const auto& name : msg.name) {
  //   std::cout << name << " ";
  // }
  // std::cout << std::endl;
  position_joints_ = msg.position;

  //pinocchio_manager_obj_.setConfiguration(msg.position);
}

// When the subscriber gets a message, this callback is triggered,
// load the model from robot_description topic
void PinocchioPanelManage::topicCallback(const std_msgs::msg::String& msg)
{
  // enable buttons
  button_->setEnabled(true);
  buttonTorque_->setEnabled(true);

  //create model with pinocchio
  pinocchio_manager_obj_ = PinocchioManager(msg.data.c_str(), ament_index_cpp::get_package_share_directory("rviz_panel_pinocchio_tiago"));

  dropdown_->clear();
  
  // Get a joint configuration and add to doropdown menu
  std::vector<std::string> joints_names = pinocchio_manager_obj_.getConfiguration();
  for (int i = 0; i < (int)joints_names.size(); i++) {
    dropdown_->addItem(QString::fromStdString(joints_names[i]));
  }

}

// Perform collision check .
void PinocchioPanelManage::buttonActivated()
{
  // set the configuration
  //pinocchio_manager_obj_.setConfiguration();
  // perform forward kinematics
  pinocchio_manager_obj_.performForwardKinematics();
  // get the frame transform
  std::string frame_transform_str = pinocchio_manager_obj_.getFrameTransform(dropdown_->currentText().toStdString());
  // display the frame transform
  label_frame_transform_->setText(QString::fromStdString(frame_transform_str));
  // perform collision check
  std::vector<std::string> collision_pairs = pinocchio_manager_obj_.performCollisionCheck();

  // display the collision pairs
  std::string collision_pairs_str;
  for(int i = 0; i < (int)collision_pairs.size(); i++) {
    collision_pairs_str += collision_pairs[i] + "\n";
    
  }
  label_collision_->clear();
  
  label_collision_->setText(QString::fromStdString(collision_pairs_str));
}


// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void PinocchioPanelManage::buttonGetTorque()
{ 
   // add payload mass to the model
   pinocchio_manager_obj_.addPayloadMass(payload_mass_);
  
  // set current configuration
  pinocchio_manager_obj_.setConfiguration(position_joints_);

  // perform torque estimation
  std::vector<double> torque_data = pinocchio_manager_obj_.performTorqueEstimation();
  // get joints with DOFs
  std::vector<std::string> joints_names = pinocchio_manager_obj_.getActiveJointsName();

  // create string to display torque values
  std::string torque_string;
  for(int i = 0; i < (int)joints_names.size(); i++) {
    torque_string += joints_names[i] + ": " + std::to_string(torque_data[i]) + "\n";
  }

  label_collision_->clear();
  label_collision_->setText(QString::fromStdString(torque_string));
}




}  // namespace rviz_panel_pinocchio_tiago

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_pinocchio_tiago::PinocchioPanelManage, rviz_common::Panel)
