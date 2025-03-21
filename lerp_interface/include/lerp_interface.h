
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Omid Heidari
   Desc: LERP planner which is a linear interpolation algorithm in joint space.
 */
 
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <vector>
#include <string>

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPInterface);  // Defines LERPInterfacePtr, ConstPtr, WeakPtr... etc.

class LERPInterface
{
public:
  LERPInterface(const rclcpp::Node::SharedPtr& node);
  
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req, 
             moveit_msgs::msg::MotionPlanDetailedResponse& res);

  // Getter and setter for num_steps_
  int getNumSteps() const { return num_steps_; }
  void setNumSteps(int num_steps) { num_steps_ = num_steps; }

  // Getter for dof_
  int getDOF() const { return dof_; }
              
protected:
  rclcpp::Node::SharedPtr node_;
  const std::string name_;
  
private:
  int num_steps_;
  int dof_;
  
  void interpolate(const std::vector<std::string>& joint_names, 
                   moveit::core::RobotStatePtr& robot_state,
                   const moveit::core::JointModelGroup* joint_model_group, 
                   const std::vector<double>& start_joint_vals,
                   const std::vector<double>& goal_joint_vals, 
                   trajectory_msgs::msg::JointTrajectory& joint_trajectory);
};

}  // namespace lerp_interface
