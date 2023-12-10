/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop_twist_joy/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{

  TeleopTwistJoy* parent;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);
  // void toggleROSParam(std::string paramName);
  void toggleROSParam(char* paramName, bool paramState);
  void toggleJogDevice();
  void toggleEmergencyStop(bool paramState);

  // Variable to store the previous state of joystick buttons to register button state change instead in <joyCallback>.
  // #TODO: GEt first msg from topic and assign it to this vector object
  std::vector<int> joy_msg_buttons_prev = std::vector<int>(12, 0);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  int64_t activate_estop_button;
  int64_t deactivate_estop_button;
  bool estop_activated;

  bool jog_arm;
  int64_t jog_arm_button;

  int64_t joystick_button_A;
  int64_t joystick_button_B;
  int64_t enable_button;

  struct {
    std::map<std::string, int64_t> axis_linear_map;
    std::map<std::string, std::map<std::string, double>> scale_linear_map;

    std::map<std::string, int64_t> axis_angular_map;
    std::map<std::string, std::map<std::string, double>> scale_angular_map;
  }base;

  struct {
    std::map<std::string, int64_t> axis_linear_map;
    std::map<std::string, std::map<std::string, double>> scale_linear_map;

    std::map<std::string, int64_t> axis_angular_map;
    std::map<std::string, std::map<std::string, double>> scale_angular_map;

    std::map<std::string, int64_t> gripper_map;
    std::map<std::string, int64_t> joystick_button;

    std::map<std::string, int64_t> pose_preset_button_map;

    int64_t toggleCartAdmittance;
    int64_t toggleNullAdmittance;
  }arm;

  // std::map<std::string, int64_t> axis_linear_map;
  // std::map<std::string, std::map<std::string, double>> scale_linear_map;

  // std::map<std::string, int64_t> axis_angular_map;
  // std::map<std::string, std::map<std::string, double>> scale_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options) : Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->parent = this;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->estop_activated = this->declare_parameter("estop_activated", false);
  pimpl_->deactivate_estop_button = this->declare_parameter("deactivate_estop_button", 4);
  pimpl_->activate_estop_button = this->declare_parameter("activate_estop_button", 5);

  pimpl_->jog_arm = this->declare_parameter("jog_arm", true);
  pimpl_->jog_arm_button = this->declare_parameter("jog_arm_button", 10);

  pimpl_->joystick_button_A = this->declare_parameter("joystick_button_A", 8);
  pimpl_->joystick_button_B = this->declare_parameter("joystick_button_B", 9);

  pimpl_->enable_button = this->declare_parameter("enable_button", 5);

  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("arm_axis_linear", default_linear_map);
  this->get_parameters("arm_axis_linear", pimpl_->arm.axis_linear_map);
  this->declare_parameters("base_axis_linear", default_linear_map);
  this->get_parameters("base_axis_linear", pimpl_->base.axis_linear_map);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("arm_axis_angular", default_angular_map);
  this->get_parameters("arm_axis_angular", pimpl_->arm.axis_angular_map);
  this->declare_parameters("base_axis_angular", default_angular_map);
  this->get_parameters("base_axis_angular", pimpl_->base.axis_angular_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.5},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 1.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_angular_normal_map{
    {"yaw", 0.5},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  std::map<std::string, double> default_scale_angular_turbo_map{
    {"yaw", 1.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  ROS_INFO_COND_NAMED(pimpl_->activate_estop_button >=0, "TeleopTwistJoy",
      "Default button to Activate ESTOP [Right Trigger Button] %" PRId64 ".", pimpl_->activate_estop_button);
  ROS_INFO_COND_NAMED(pimpl_->deactivate_estop_button >=0, "TeleopTwistJoy",
    "Default button to Deactivate ESTOP [Left Trigger Button] %" PRId64 ".", pimpl_->deactivate_estop_button);

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
       it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
    //   "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
       it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
    //   "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                              "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                              "activate_estop_button", "deactivate_estop_button", "jog_arm_button", 
                                              "joystick_button_A", "joystick_button_B"};
    static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                 "scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z",
                                                 "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll",
                                                 "scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll"};
    static std::set<std::string> boolparams = {"estop_activated", "jog_arm"};
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (boolparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      if (parameter.get_name() == "estop_activated")
      {
        this->pimpl_->estop_activated = parameter.get_value<rclcpp::PARAMETER_BOOL>();
      }
      else if (parameter.get_name() == "activate_estop_button")
      {
        this->pimpl_->activate_estop_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "deactivate_estop_button")
      {
        this->pimpl_->deactivate_estop_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "jog_arm_button")
      {
        this->pimpl_->jog_arm_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "jog_arm")
      {
        this->pimpl_->jog_arm = parameter.get_value<rclcpp::PARAMETER_BOOL>();
      }


      if (this->pimpl_->jog_arm)
      {
        // switch(parameter.get_name())
        // {
        //   case "joystick_button.A":
        //     this->pimpl_->joystick_button_A = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        //     break;
        // }


        if (parameter.get_name() == "arm.joystick_button.A")
        {
          this->pimpl_->joystick_button_A = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.joystick_button.B")
        {
          this->pimpl_->joystick_button_B = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_linear.x")
        {
          this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_linear.y")
        {
          this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_linear.z")
        {
          this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_angular.yaw")
        {
          this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_angular.pitch")
        {
          this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm.axis_angular.roll")
        {
          this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.x")
        {
          this->pimpl_->scale_linear_map["turbo"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.y")
        {
          this->pimpl_->scale_linear_map["turbo"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.z")
        {
          this->pimpl_->scale_linear_map["turbo"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.x")
        {
          this->pimpl_->scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.y")
        {
          this->pimpl_->scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.z")
        {
          this->pimpl_->scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.yaw")
        {
          this->pimpl_->scale_angular_map["turbo"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.pitch")
        {
          this->pimpl_->scale_angular_map["turbo"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.roll")
        {
          this->pimpl_->scale_angular_map["turbo"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.yaw")
        {
          this->pimpl_->scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.pitch")
        {
          this->pimpl_->scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.roll")
        {
          this->pimpl_->scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
      }
      else
      {

      }
      
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopTwistJoy::~TeleopTwistJoy()
{
  delete pimpl_;
}

double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      axis_map.at(fieldname) == -1L ||
      scale_map.find(fieldname) == scale_map.end() ||
      static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

  if (joy_msg->buttons[joystick_button_A] > 0)
  {
    cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
    cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
  }
  else
  {
    cmd_vel_msg->linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
    cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  }
  
  if (joy_msg->buttons[joystick_button_B] > 0)
  {
    cmd_vel_msg->angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  }
  else
  {
    cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  }
  
  cmd_vel_pub->publish(std::move(cmd_vel_msg));
  sent_disable_msg = false;
}

//
void TeleopTwistJoy::Impl::toggleJogDevice()
{
  // auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(my_node, "teleop_twist_joy_node");
  // while (!parameters_client->wait_for_service(1s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //     rclcpp::shutdown();
  //   }
  //   RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  // }
  // std::vector<rclcpp::Parameter> new_parameters{rclcpp::Parameter("jog_arm", !(jog_arm))};
  // RCLCPP_INFO(this->get_logger(), "\njog_arm param set: %s!\n", jog_arm ? "true" : "false");
  // parameters_client->set_parameters(new_parameters);

  // auto parameter = parameters_client->get_parameter("jog_arm");
  // RCLCPP_INFO(this->get_logger(), "\nPreviously set jog_arm = %s!\n", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
  parent->set_parameter(rclcpp::Parameter("jog_arm", !(jog_arm)));
  RCLCPP_INFO(parent->get_logger(), "\njog_arm param set: %s!\n", jog_arm ? "true" : "false");

  auto parameter = parent->get_parameter("jog_arm");
  RCLCPP_INFO(parent->get_logger(), "\nPreviously set jog_arm = %s!\n", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");

}

void TeleopTwistJoy::Impl::toggleROSParam(char* paramName, bool param_state)
{
  parent->set_parameter(rclcpp::Parameter(paramName, param_state));

  auto parameter = parent->get_parameter(paramName);
  RCLCPP_INFO(parent->get_logger(), "The recently set \"%s\" value = %s!\n", paramName, parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
}

void TeleopTwistJoy::Impl::toggleEmergencyStop(bool estop_state)
{
  parent->set_parameter(rclcpp::Parameter("estop_activated", estop_state));

  auto parameter = parent->get_parameter("estop_activated");
  RCLCPP_INFO(parent->get_logger(), "The recently set \"estop_activated\" value = %s!", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (joy_msg_buttons_prev[jog_arm_button]==0 && joy_msg->buttons[jog_arm_button] ==1)
  {
    toggleJogDevice();
  }

  if (joy_msg_buttons_prev[deactivate_estop_button]==0 && joy_msg->buttons[deactivate_estop_button] ==1)
  {
    toggleEmergencyStop(false);
    sent_disable_msg = false;
  }
  else if (joy_msg_buttons_prev[activate_estop_button]==0 && joy_msg->buttons[activate_estop_button] ==1)
  {
    toggleEmergencyStop(true);
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
      sent_disable_msg = true;
    }
  }
  else
  {
    if (!sent_disable_msg)
    {
      sendCmdVelMsg(joy_msg, "normal");
    }
  }

  joy_msg_buttons_prev = joy_msg->buttons;
}

}  // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)
