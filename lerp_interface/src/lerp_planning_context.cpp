
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Omid Heidari */

#include "lerp_planning_context.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

namespace lerp_interface
{

LERPPlanningContext::LERPPlanningContext(const std::string& context_name, const std::string& group_name,
                                         const moveit::core::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  // Create a ROS 2 node for the LERP interface
  auto node = std::make_shared<rclcpp::Node>("lerp_planning_context");
  lerp_interface_ = std::make_shared<LERPInterface>(node);
}

bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  moveit_msgs::msg::MotionPlanDetailedResponse res_msg;
  bool lerp_solved = lerp_interface_->solve(planning_scene_, request_, res_msg);

  if (lerp_solved)
  {
    res.trajectory_.resize(1);
    res.trajectory_[0] =
        robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

    moveit::core::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);

    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
    res.description_.push_back("plan");
    res.processing_time_ = res_msg.processing_time;
    res.error_code_ = res_msg.error_code;

    return true;
  }

  res.error_code_ = res_msg.error_code;
  return false;
}

bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool planning_success = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;

  if (planning_success)
  {
    res.trajectory_ = res_detailed.trajectory_[0];
    res.planning_time_ = res_detailed.processing_time_[0];
  }

  return planning_success;
}

bool LERPPlanningContext::terminate()
{
  return true;
}

void LERPPlanningContext::clear()
{
  // This planner has no state, so has nothing to clear
}

}  // namespace lerp_interface

