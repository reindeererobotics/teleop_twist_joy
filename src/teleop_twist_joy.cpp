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

#include <future>
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

    TeleopTwistJoy *parent;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string &which_map);
    // void toggleROSParam(std::string paramName);
    void toggleROSParam(char *paramName, bool paramState);
    void toggleJogDevice();
    void toggleEmergencyStop(bool paramState);


    void feedback_callback(GoalHandleGripper::SharedPtr,
                      const std::shared_ptr<const Gripper::Feedback> feedback);
    void result_callback(const GoalHandleGripper::WrappedResult & result);
    void goal_response_callback(const GoalHandleGripper::SharedPtr & goal_handle);
    void send_goal(control_msgs::msg::GripperCommand::SharedPtr goal_msg);




    // Variable to store the previous state of joystick buttons to register button state change instead in <joyCallback>.
    // #TODO: GEt first msg from topic and assign it to this vector object
    std::vector<int> joy_msg_axes_prev = std::vector<int>(8, 0);
    std::vector<int> joy_msg_buttons_prev = std::vector<int>(12, 0);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_arm_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_base_pub;

    rclcpp_action::Client<Gripper>::SharedPtr gripper_client_ptr_;


    int64_t activate_estop_button;
    int64_t deactivate_estop_button;
    bool estop_activated;

    bool arm_jogged;
    int64_t jog_arm_button;

    int64_t XYTwist_toggle; // Left joystick button to switch between controlling translation and rotation in the X and Y axes
    int64_t ZTwist_toggle;  // Right joystick button to switch between controlling translation and rotation in the Z axes


    struct
    {
      std::map<std::string, int64_t> axis_linear_map;
      std::map<std::string, std::map<std::string, double>> scale_linear_map;

      std::map<std::string, int64_t> axis_angular_map;
      std::map<std::string, std::map<std::string, double>> scale_angular_map;
    } base;

    struct
    {
      std::map<std::string, int64_t> axis_linear_map;
      std::map<std::string, std::map<std::string, double>> scale_linear_map;

      std::map<std::string, int64_t> axis_angular_map;
      std::map<std::string, std::map<std::string, double>> scale_angular_map;

      std::map<std::string, int64_t> gripper_map;

      std::map<std::string, int64_t> pose_preset_button_map;
      std::map<std::string, std::map<std::string, double>> pose_presets_map;

      int64_t toggleCartAdmittance;
      int64_t toggleNullAdmittance;
    } arm;

    bool sent_disable_msg;
    bool running_gripper_cmd;
    double gripper_pos;
  };

  /**
   * Constructs TeleopTwistJoy.
   */
  TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions &options) : Node("teleop_twist_joy_node", options)
  {
    pimpl_ = new Impl;

    pimpl_->parent = this;

    pimpl_->cmd_vel_arm_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_arm", 10);
    pimpl_->cmd_vel_base_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_base", 10);
    pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
                                                                       std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

    pimpl_->gripper_client_ptr_ = rclcpp_action::create_client<Gripper>(this, "/robotiq_gripper_controller/gripper_cmd");

    pimpl_->estop_activated = this->declare_parameter("estop_activated", false);
    pimpl_->deactivate_estop_button = this->declare_parameter("deactivate_estop_button", 4);
    pimpl_->activate_estop_button = this->declare_parameter("activate_estop_button", 5);

    pimpl_->arm_jogged = this->declare_parameter("arm_jogged", true);
    pimpl_->jog_arm_button = this->declare_parameter("jog_arm_button", 10);

    pimpl_->XYTwist_toggle = this->declare_parameter("XYTwist_toggle", 8);
    pimpl_->ZTwist_toggle = this->declare_parameter("ZTwist_toggle", 9);

    pimpl_->arm.toggleCartAdmittance = this->declare_parameter("arm_toggleCartAdmittance", 2);
    pimpl_->arm.toggleNullAdmittance = this->declare_parameter("arm_toggleNullAdmittance", 3);

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
    this->declare_parameters("arm_scale_linear", default_scale_linear_normal_map);
    this->get_parameters("arm_scale_linear", pimpl_->arm.scale_linear_map["normal"]);
    this->declare_parameters("base_scale_linear", default_scale_linear_normal_map);
    this->get_parameters("base_scale_linear", pimpl_->base.scale_linear_map["normal"]);

    std::map<std::string, double> default_scale_angular_normal_map{
        {"yaw", 0.5},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("arm_scale_angular", default_scale_angular_normal_map);
    this->get_parameters("arm_scale_angular", pimpl_->arm.scale_angular_map["normal"]);
    this->declare_parameters("base_scale_angular", default_scale_angular_normal_map);
    this->get_parameters("base_scale_angular", pimpl_->base.scale_angular_map["normal"]);

    std::map<std::string, int64_t> default_gripper_map{
        {"close", 2},
        {"open", 5},
    };
    this->declare_parameters("arm_gripper", default_gripper_map);
    this->get_parameters("arm_gripper", pimpl_->arm.gripper_map);

    std::map<std::string, int64_t> default_preset_button_map{
        {"home", 1},
        {"retract", 0},
    };
    this->declare_parameters("arm_pose_preset_button", default_preset_button_map);
    this->get_parameters("arm_pose_preset_button", pimpl_->arm.pose_preset_button_map);

    std::map<std::string, double> default_pose_preset_map{
        {"x", 0.0},
        {"y", 0.0},
        {"z", 0.0},
        {"yaw", 0.0},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("arm_pose_presets.home", default_pose_preset_map);
    this->get_parameters("arm_pose_presets.home", pimpl_->arm.pose_presets_map["home"]);
    this->declare_parameters("arm_pose_presets.retract", default_pose_preset_map);
    this->get_parameters("arm_pose_presets.retract", pimpl_->arm.pose_presets_map["retract"]);

    ROS_INFO_COND_NAMED(pimpl_->activate_estop_button >= 0, "TeleopTwistJoy",
                        "Default button to Activate ESTOP [Right Bumper] %" PRId64 ".", pimpl_->activate_estop_button);
    ROS_INFO_COND_NAMED(pimpl_->deactivate_estop_button >= 0, "TeleopTwistJoy",
                        "Default button to Deactivate ESTOP [Left Bumper] %" PRId64 ".", pimpl_->deactivate_estop_button);

    for (std::map<std::string, int64_t>::iterator it = pimpl_->arm.axis_linear_map.begin();
         it != pimpl_->arm.axis_linear_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
                          it->first.c_str(), it->second, pimpl_->arm.scale_linear_map["normal"][it->first]);
      // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      //   "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
    }

    for (std::map<std::string, int64_t>::iterator it = pimpl_->arm.axis_angular_map.begin();
         it != pimpl_->arm.axis_angular_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
                          it->first.c_str(), it->second, pimpl_->arm.scale_angular_map["normal"][it->first]);
      // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      //   "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
    }
        
    pimpl_->sent_disable_msg = false;
    pimpl_->running_gripper_cmd = false;

    auto param_callback =
        [this](std::vector<rclcpp::Parameter> parameters)
    {
      // RCLCPP_INFO(this->get_logger(), "Beginning of param callback!");
      static std::set<std::string> intparams = {"arm_axis_linear.x", "arm_axis_linear.y", "arm_axis_linear.z",
                                                "arm_axis_angular.yaw", "arm_axis_angular.pitch", "arm_axis_angular.roll",
                                                "arm_pose_preset_button.home", "arm_pose_preset_button.retract",
                                                "arm_gripper.close", "arm_gripper.open",
                                                "base_axis_linear.x", "base_axis_linear.y", "base_axis_linear.z",
                                                "base_axis_angular.yaw", "base_axis_angular.pitch", "base_axis_angular.roll",
                                                "activate_estop_button", "deactivate_estop_button", "jog_arm_button",
                                                "XYTwist_toggle", "ZTwist_toggle"};
      static std::set<std::string> doubleparams = {"arm_scale_linear.x", "arm_scale_linear.y", "arm_scale_linear.z",
                                                   "arm_scale_angular.yaw", "arm_scale_angular.pitch", "arm_scale_angular.roll",
                                                   "arm_pose_presets.home.x", "arm_pose_presets.home.y", "arm_pose_presets.home.z",
                                                   "arm_pose_presets.home.yaw", "arm_pose_presets.home.pitch", "arm_pose_presets.home.roll",
                                                   "arm_pose_presets.retract.x", "arm_pose_presets.retract.y", "arm_pose_presets.retract.z",
                                                   "arm_pose_presets.retract.yaw", "arm_pose_presets.retract.pitch", "arm_pose_presets.retract.roll",
                                                   "base_scale_linear.x", "base_scale_linear.y", "base_scale_linear.z",
                                                   "base_scale_angular.yaw", "base_scale_angular.pitch", "base_scale_angular.roll"};
      static std::set<std::string> boolparams = {"estop_activated", "arm_jogged"};
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      // Loop to check if changed parameters are of expected data type
      for (const auto &parameter : parameters)
      {
        RCLCPP_INFO(this->get_logger(), "%s value = %s!", parameter.get_name().c_str(), parameter.value_to_string().c_str());
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
      for (const auto &parameter : parameters)
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
        else if (parameter.get_name() == "arm_jogged")
        {
          this->pimpl_->arm_jogged = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "XYTwist_toggle")
        {
          this->pimpl_->XYTwist_toggle = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "ZTwist_toggle")
        {
          this->pimpl_->ZTwist_toggle = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "arm_axis_linear.x") // Get Parameters related to the robotic base
        {
          this->pimpl_->arm.axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_axis_linear.y")
        {
          this->pimpl_->arm.axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_axis_linear.z")
        {
          this->pimpl_->arm.axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_axis_linear.yaw")
        {
          this->pimpl_->arm.axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_axis_linear.pitch")
        {
          this->pimpl_->arm.axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_axis_linear.roll")
        {
          this->pimpl_->arm.axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "arm_scale_linear.x")
        {
          this->pimpl_->arm.scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_scale_linear.y")
        {
          this->pimpl_->arm.scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_scale_linear.z")
        {
          this->pimpl_->arm.scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_scale_angular.yaw")
        {
          this->pimpl_->arm.scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_scale_angular.pitch")
        {
          this->pimpl_->arm.scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_scale_angular.roll")
        {
          this->pimpl_->arm.scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.x")
        {
          this->pimpl_->arm.pose_presets_map["home"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.y")
        {
          this->pimpl_->arm.pose_presets_map["home"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.z")
        {
          this->pimpl_->arm.pose_presets_map["home"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.yaw")
        {
          this->pimpl_->arm.pose_presets_map["home"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.pitch")
        {
          this->pimpl_->arm.pose_presets_map["home"]["[pitch]"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.home.roll")
        {
          this->pimpl_->arm.pose_presets_map["home"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.x")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.y")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.z")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.yaw")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.pitch")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["[pitch]"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "arm_pose_presets.retract.roll")
        {
          this->pimpl_->arm.pose_presets_map["retract"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_axis_linear.x") // Get Parameters related to the robotic base
        {
          this->pimpl_->base.axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_axis_linear.y")
        {
          this->pimpl_->base.axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_axis_linear.z")
        {
          this->pimpl_->base.axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_axis_linear.yaw")
        {
          this->pimpl_->base.axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_axis_linear.pitch")
        {
          this->pimpl_->base.axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_axis_linear.roll")
        {
          this->pimpl_->base.axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "base_scale_linear.x")
        {
          this->pimpl_->base.scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_scale_linear.y")
        {
          this->pimpl_->base.scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_scale_linear.z")
        {
          this->pimpl_->base.scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_scale_angular.yaw")
        {
          this->pimpl_->base.scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_scale_angular.pitch")
        {
          this->pimpl_->base.scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "base_scale_angular.roll")
        {
          this->pimpl_->base.scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
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

  double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> &axis_map,
                const std::map<std::string, double> &scale_map, const std::string &fieldname)
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


  void TeleopTwistJoy::Impl::send_goal(control_msgs::msg::GripperCommand::SharedPtr goal_msg)
  {
    if (!gripper_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(parent->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_action = Gripper::Goal();
    goal_action.command = *goal_msg;

    // RCLCPP_INFO(parent->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Gripper>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TeleopTwistJoy::Impl::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&TeleopTwistJoy::Impl::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&TeleopTwistJoy::Impl::result_callback, this, std::placeholders::_1);

    gripper_client_ptr_->async_send_goal(goal_action, send_goal_options);
    
    running_gripper_cmd = true;
    // RCLCPP_INFO(parent->get_logger(), "Running Gripper cmd = %s", running_gripper_cmd ? "True" : "False");
  }

  void TeleopTwistJoy::Impl::goal_response_callback(const GoalHandleGripper::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(parent->get_logger(), "Goal was rejected by server");
    } else {
      // RCLCPP_INFO(parent->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void TeleopTwistJoy::Impl::feedback_callback(
    GoalHandleGripper::SharedPtr,
    const std::shared_ptr<const Gripper::Feedback> feedback)
  {
    RCLCPP_INFO(parent->get_logger(), "Current Feedback = %f", feedback->position);
    gripper_pos = feedback->position;
  }

  void TeleopTwistJoy::Impl::result_callback(const GoalHandleGripper::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(parent->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(parent->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(parent->get_logger(), "Unknown result code");
        return;
    }
    auto goal_result = result.result->reached_goal;

    // RCLCPP_INFO(parent->get_logger(), "Goal Result = %s", goal_result ? "True" : "False");

    running_gripper_cmd = false;
    
    #if RCLCPP_LOG_MIN_SEVERITY_DEBUG == 1
      RCLCPP_DEBUG(parent->get_logger(), "Running Gripper cmd = %s", running_gripper_cmd ? "True" : "False");
    #endif
    gripper_pos = result.result->position;
    // rclcpp::shutdown();
  }

/// Maps a value from an input range to an output range
double map_to_range(double inputVal, std::tuple<double, double> input_range, std::tuple<double, double> output_range)
{
  auto slope = 1.0 * (std::get<1>(output_range) - std::get<0>(output_range)) / (std::get<1>(input_range) - std::get<0>(input_range));
  return std::get<0>(output_range) + (slope * (inputVal - std::get<0>(input_range)));
}


  void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                           const std::string &which_map)
  {
    // Initializes with zeros by default.
    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    // auto zero_cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  if (arm_jogged)
  {
    if (joy_msg->buttons[XYTwist_toggle] > 0)
    {
      cmd_vel_msg->angular.y = getVal(joy_msg, arm.axis_angular_map, arm.scale_angular_map[which_map], "pitch");
      cmd_vel_msg->angular.x = getVal(joy_msg, arm.axis_angular_map, arm.scale_angular_map[which_map], "roll");
    }
    else
    {
      cmd_vel_msg->linear.x = getVal(joy_msg, arm.axis_linear_map, arm.scale_linear_map[which_map], "x");
      cmd_vel_msg->linear.y = getVal(joy_msg, arm.axis_linear_map, arm.scale_linear_map[which_map], "y");
    }

    if (joy_msg->buttons[ZTwist_toggle] > 0)
    {
      cmd_vel_msg->angular.z = getVal(joy_msg, arm.axis_angular_map, arm.scale_angular_map[which_map], "yaw");
    }
    else
    {
      cmd_vel_msg->linear.z = getVal(joy_msg, arm.axis_linear_map, arm.scale_linear_map[which_map], "z");
    }

    cmd_vel_arm_pub->publish(std::move(cmd_vel_msg));
    // cmd_vel_base_pub->publish(std::move(zero_cmd_vel_msg));

  }
  else
  {
    cmd_vel_msg->angular.z = getVal(joy_msg, base.axis_angular_map, base.scale_angular_map[which_map], "yaw");
    cmd_vel_msg->angular.y = getVal(joy_msg, base.axis_angular_map, base.scale_angular_map[which_map], "pitch");
    cmd_vel_msg->angular.x = getVal(joy_msg, base.axis_angular_map, base.scale_angular_map[which_map], "roll");
  
    cmd_vel_msg->linear.x = getVal(joy_msg, base.axis_linear_map, base.scale_linear_map[which_map], "x");
    cmd_vel_msg->linear.y = getVal(joy_msg, base.axis_linear_map, base.scale_linear_map[which_map], "y");
    cmd_vel_msg->linear.z = getVal(joy_msg, base.axis_linear_map, base.scale_linear_map[which_map], "z");
    
    cmd_vel_base_pub->publish(std::move(cmd_vel_msg));
    // cmd_vel_arm_pub->publish(std::move(zero_cmd_vel_msg));

  }
    
    sent_disable_msg = false;
  }

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
    // std::vector<rclcpp::Parameter> new_parameters{rclcpp::Parameter("arm_jogged", !(arm_jogged))};
    // RCLCPP_INFO(this->get_logger(), "\njog_arm param set: %s!\n", arm_jogged ? "true" : "false");
    // parameters_client->set_parameters(new_parameters);
    // auto parameter = parameters_client->get_parameter("arm_jogged");
    // RCLCPP_INFO(this->get_logger(), "\nPreviously set arm_jogged = %s!\n", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
    parent->set_parameter(rclcpp::Parameter("arm_jogged", !(arm_jogged)));

    #if RCLCPP_LOG_MIN_SEVERITY_DEBUG == 1
      auto parameter = parent->get_parameter("arm_jogged");
      RCLCPP_DEBUG(parent->get_logger(), "The recently set arm_jogged = %s!", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
      #endif
  }

  void TeleopTwistJoy::Impl::toggleROSParam(char *paramName, bool param_state)
  {
    parent->set_parameter(rclcpp::Parameter(paramName, param_state));

    #if RCLCPP_LOG_MIN_SEVERITY_DEBUG == 1
      auto parameter = parent->get_parameter(paramName);
      RCLCPP_DEBUG(parent->get_logger(), "The recently set \"%s\" value = %s!\n", paramName, parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
    #endif
  }

  void TeleopTwistJoy::Impl::toggleEmergencyStop(bool estop_state)
  {
    parent->set_parameter(rclcpp::Parameter("estop_activated", estop_state));
    #if RCLCPP_LOG_MIN_SEVERITY_DEBUG == 1
      auto parameter = parent->get_parameter("estop_activated");
      RCLCPP_DEBUG(parent->get_logger(), "The recently set \"estop_activated\" value = %s!", parameter.get_value<rclcpp::PARAMETER_BOOL>() ? "true" : "false");
    #endif
  }

  void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // SWitches between controlling the Arm or the Base based on the jog_arm_button presses
    if (joy_msg_buttons_prev[jog_arm_button] == 1 && joy_msg->buttons[jog_arm_button] == 0)
    {
      toggleJogDevice();

      // Initializes with zeros by default to stop the arm and base.
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      auto cmd_vel_msg2 = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_arm_pub->publish(std::move(cmd_vel_msg));
      cmd_vel_base_pub->publish(std::move(cmd_vel_msg2));
    }


    // CMD_VEL mapping based on E-Stop State
    if (joy_msg_buttons_prev[deactivate_estop_button] == 0 && joy_msg->buttons[deactivate_estop_button] == 1)
    {
      sent_disable_msg = false;
      toggleEmergencyStop(sent_disable_msg);
    }
    else if (joy_msg_buttons_prev[activate_estop_button] == 0 && joy_msg->buttons[activate_estop_button] == 1)
    {
      if (!sent_disable_msg)
      {
        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        auto cmd_vel_msg2 = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel_arm_pub->publish(std::move(cmd_vel_msg));
        cmd_vel_base_pub->publish(std::move(cmd_vel_msg2));
        sent_disable_msg = true;
      }
      toggleEmergencyStop(sent_disable_msg);
    }
    else
    {
      if (!sent_disable_msg)
      {
        sendCmdVelMsg(joy_msg, "normal");
      }
    }


    auto close_trigger_val = joy_msg->axes[arm.gripper_map.at("close")];
    auto open_trigger_val = joy_msg->axes[arm.gripper_map.at("open")];

    // Handles logic for opening and closing the gripper with the triggers
    if (close_trigger_val <= 0.99 && (gripper_pos <= map_to_range(close_trigger_val, {1, -1}, {0, 0.8})))
    {
      if (!running_gripper_cmd)
      {
        auto goal_msg = std::make_shared<control_msgs::msg::GripperCommand>();
        goal_msg->position = map_to_range(close_trigger_val, {1, -1}, {gripper_pos, 0.8});
        goal_msg->max_effort = 100.0;
        // RCLCPP_INFO(parent->get_logger(), "Trigger Value [Close] = %f", close_trigger_val);
        RCLCPP_INFO(parent->get_logger(), "Gripper Position [Close] = %f", map_to_range(close_trigger_val, {1, -1}, {gripper_pos, 0.8}));
        send_goal(goal_msg);
      }
    }
    else if (open_trigger_val <= 0.99 && (gripper_pos >= map_to_range(open_trigger_val, {1, -1}, {0.8, 0})))
    {
      if (!running_gripper_cmd)
      {
        // RCLCPP_INFO(parent->get_logger(), "Running Gripper cmd = %s", running_gripper_cmd ? "True" : "False");
        auto goal_msg = std::make_shared<control_msgs::msg::GripperCommand>();
        goal_msg->position = map_to_range(open_trigger_val, {1, -1}, {gripper_pos, 0});
        goal_msg->max_effort = 100.0;
        // RCLCPP_INFO(parent->get_logger(), "Trigger Value [Open] = %f", open_trigger_val);
        RCLCPP_INFO(parent->get_logger(), "Gripper Position [Open] = %f", map_to_range(open_trigger_val, {1, -1}, {gripper_pos, 0}));
        send_goal(goal_msg);
      }
    }

    // REcord the current joy msg state as the previous state in the next iteration
    joy_msg_buttons_prev = joy_msg->buttons;
  }

} // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)
