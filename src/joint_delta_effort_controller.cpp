/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Tin Lai
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/*
 Author: Tin Lai
 Desc: Effort(force)-based delta effort controller using basic PID loop
*/

#include <angles/angles.h>
#include <effort_controllers/joint_delta_effort_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {

JointDeltaEffortController::JointDeltaEffortController() : loop_count_(0) {}

JointDeltaEffortController::~JointDeltaEffortController() {
  sub_position_command_.shutdown();
  sub_delta_effort_command_.shutdown();
  sub_delta_effort_duration_command_.shutdown();
}

bool JointDeltaEffortController::init(
    hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
          n, "state", 1));

  // Start command subscriber
  sub_position_command_ = n.subscribe<std_msgs::Float64>(
      "position_command", 1, &JointDeltaEffortController::setPositionCB, this);
  sub_delta_effort_command_ = n.subscribe<std_msgs::Float64>(
      "delta_effort_command", 1, &JointDeltaEffortController::setDeltaEffortCB,
      this);
  sub_delta_effort_duration_command_ = n.subscribe<std_msgs::Duration>(
      "delta_effort_duration", 1,
      [this](const std_msgs::DurationConstPtr &msg) {
        delta_effort_command_struct_.duration = msg.get()->data.toSec();
        delta_effort_command_.writeFromNonRT(delta_effort_command_struct_);
      });
  applying_delta_effort.store(false);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n)) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_) {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void JointDeltaEffortController::setGains(const double &p, const double &i,
                                          const double &d, const double &i_max,
                                          const double &i_min,
                                          const bool &antiwindup) {
  pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void JointDeltaEffortController::getGains(double &p, double &i, double &d,
                                          double &i_max, double &i_min,
                                          bool &antiwindup) {
  pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void JointDeltaEffortController::getGains(double &p, double &i, double &d,
                                          double &i_max, double &i_min) {
  bool dummy;
  pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

void JointDeltaEffortController::printDebug() { pid_controller_.printValues(); }

std::string JointDeltaEffortController::getJointName() {
  return joint_.getName();
}

double JointDeltaEffortController::getPosition() {
  return joint_.getPosition();
}

// Set the joint position command
void JointDeltaEffortController::setCommand(double pos_command) {
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ =
      false; // Flag to ignore the velocity command since our setCommand method
             // did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to
  //  ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void JointDeltaEffortController::setCommand(double pos_command,
                                            double vel_command) {
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

void JointDeltaEffortController::starting(const ros::Time &time) {
  double pos_command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  command_.initRT(command_struct_);

  delta_effort_command_struct_.duration = 0.1; // default 100 ms
  delta_effort_command_struct_.effort = 0;
  delta_effort_command_struct_.start_time = ros::Time::now();
  delta_effort_command_.initRT(delta_effort_command_struct_);

  pid_controller_.reset();
}

void JointDeltaEffortController::update(const ros::Time &time,
                                        const ros::Duration &period) {
  double commanded_effort;

  delta_effort_command_struct_ = *(delta_effort_command_.readFromRT());
  //    applying_delta_effort = true;
  if (applying_delta_effort) {
    /*
     * High priority delta effort controller
     * */
    // delta_effort_duration <= 0 are special values that implies applying
    // the same effort effort forever.
    if (delta_effort_command_struct_.duration <= 0 ||
        (time - delta_effort_command_struct_.start_time).toSec() <
            delta_effort_command_struct_.duration) {

      commanded_effort =
          last_commanded_effort + delta_effort_command_struct_.effort;
      ////////////////////////////////////
      joint_.setCommand(commanded_effort);
      ////////////////////////////////////
    } else {
      // delta effort just expired
      applying_delta_effort.store(false);
      // assign current joint pos as target, to facilitate holding position
      command_struct_.position_ = joint_.getPosition();
      command_struct_.has_velocity_ = false;
      command_.writeFromNonRT(command_struct_);
    }
  }
  if (!applying_delta_effort) {
    /*
     * Original position controller
     * */
    command_struct_ = *(command_.readFromRT());
    double command_position = command_struct_.position_;
    double command_velocity = command_struct_.velocity_;
    bool has_velocity_ = command_struct_.has_velocity_;

    double error, vel_error;

    double current_position = joint_.getPosition();

    // Make sure joint is within limits if applicable
    enforceJointLimits(command_position);

    // Compute position error
    if (joint_urdf_->type == urdf::Joint::REVOLUTE) {
      angles::shortest_angular_distance_with_large_limits(
          current_position, command_position, joint_urdf_->limits->lower,
          joint_urdf_->limits->upper, error);
    } else if (joint_urdf_->type == urdf::Joint::CONTINUOUS) {
      error =
          angles::shortest_angular_distance(current_position, command_position);
    } else // prismatic
    {
      error = command_position - current_position;
    }

    // Decide which of the two PID computeCommand() methods to call
    if (has_velocity_) {
      // Compute velocity error if a non-zero velocity command was given
      vel_error = command_velocity - joint_.getVelocity();

      // Set the PID error and compute the PID command with nonuniform
      // time step size. This also allows the user to pass in a precomputed
      // derivative error.
      commanded_effort =
          pid_controller_.computeCommand(error, vel_error, period);
    } else {
      // Set the PID error and compute the PID command with nonuniform
      // time step size.
      commanded_effort = pid_controller_.computeCommand(error, period);
    }
    ////////////////////////////////////
    joint_.setCommand(commanded_effort);
    ////////////////////////////////////

    // publish state
    if (loop_count_ % 10 == 0) {
      if (controller_state_publisher_ &&
          controller_state_publisher_->trylock()) {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_position;
        controller_state_publisher_->msg_.process_value = current_position;
        controller_state_publisher_->msg_.process_value_dot =
            joint_.getVelocity();
        controller_state_publisher_->msg_.error = error;
        controller_state_publisher_->msg_.time_step = period.toSec();
        controller_state_publisher_->msg_.command = commanded_effort;

        double dummy;
        bool antiwindup;
        getGains(controller_state_publisher_->msg_.p,
                 controller_state_publisher_->msg_.i,
                 controller_state_publisher_->msg_.d,
                 controller_state_publisher_->msg_.i_clamp, dummy, antiwindup);
        controller_state_publisher_->msg_.antiwindup =
            static_cast<char>(antiwindup);
        controller_state_publisher_->unlockAndPublish();
      }
    }

    last_commanded_effort = commanded_effort;
  }

  loop_count_++;
}

void JointDeltaEffortController::setPositionCB(
    const std_msgs::Float64ConstPtr &msg) {
  applying_delta_effort.store(false);
  setCommand(msg->data);
}

void JointDeltaEffortController::setDeltaEffortCB(
    const std_msgs::Float64ConstPtr &msg) {

  delta_effort_command_struct_.effort = msg->data;
  delta_effort_command_struct_.start_time = ros::Time::now();

  delta_effort_command_.writeFromNonRT(delta_effort_command_struct_);
  applying_delta_effort.store(true);
}

// Note: we may want to remove this function once issue
// https://github.com/ros/angles/issues/2 is resolved
void JointDeltaEffortController::enforceJointLimits(double &command) {
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE ||
      joint_urdf_->type == urdf::Joint::PRISMATIC) {
    if (command > joint_urdf_->limits->upper) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    } else if (command < joint_urdf_->limits->lower) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace effort_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::JointDeltaEffortController,
                       controller_interface::ControllerBase)
