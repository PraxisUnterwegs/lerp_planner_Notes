
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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
 *  POSSIBILITY OF SUCH DAMAGE
 *********************************************************************/

/* Author: Omid Heidari */

#include "lerp_interface.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace lerp_interface
{

LERPInterface::LERPInterface(const rclcpp::Node::SharedPtr& node) 
  : node_(node), name_("LERPInterface"), num_steps_(10), dof_(0)
{
  node_->get_parameter_or("num_steps", num_steps_, 10);
}

bool LERPInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::msg::MotionPlanDetailedResponse& res)
{
  // Check input parameters
  if (!planning_scene || !node_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid planning scene or node handle.");
    return false;
  }

  if (req.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No goal constraints provided.");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // Load the planner-specific parameters
  auto start_time = node_->now();
  moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model));
  *start_state = planning_scene->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  dof_ = joint_names.size();
  std::vector<double> start_joint_values;
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

  // This planner only supports one goal constraint in the request
  std::vector<moveit_msgs::msg::Constraints> goal_constraints = req.goal_constraints;
  std::vector<moveit_msgs::msg::JointConstraint> goal_joint_constraint = goal_constraints[0].joint_constraints;

  std::vector<double> goal_joint_values;
  goal_joint_values.reserve(goal_joint_constraint.size());

  for (const auto& constraint : goal_joint_constraint)
  {
    goal_joint_values.push_back(constraint.position);
  }

  // ==================== Interpolation
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

  // ==================== feed the response
  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header.stamp = node_->now();
  res.trajectory[0].joint_trajectory = joint_trajectory;

  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((node_->now() - start_time).seconds());

  res.group_name = req.group_name;
  res.trajectory_start.joint_state.name = joint_names;
  res.trajectory_start.joint_state.position = start_joint_values;
  res.trajectory_start.joint_state.header.stamp = node_->now();

  return true;
}

void LERPInterface::interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
                                const moveit::core::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  joint_trajectory.points.resize(num_steps_ + 1);

  // Calculate the step size for each joint
  std::vector<double> dt_vector;
  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
    double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
    dt_vector.push_back(dt);
  }

  // Perform linear interpolation
  for (int step = 0; step <= num_steps_; ++step)
  {
    std::vector<double> joint_values;
    for (int k = 0; k < dof_; ++k)
    {
      double joint_value = start_joint_vals[k] + step * dt_vector[k];
      joint_values.push_back(joint_value);
    }
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    rob_state->update();

    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points[step].positions = joint_values;
    
    // Set timestamp for each trajectory point (required in ROS2)
    joint_trajectory.points[step].time_from_start = rclcpp::Duration::from_seconds(step * 0.1);  // 0.1s per step
  }
}

}  // namespace lerp_interface
