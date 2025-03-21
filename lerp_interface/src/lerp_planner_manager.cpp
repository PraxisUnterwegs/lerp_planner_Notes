
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

/* Author: Omid Heidari
   Desc:   LERP planning plugin
*/

#include "lerp_planning_context.h"

#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/planning_interface/planning_interface.h>

#include <pluginlib/class_list_macros.hpp>

namespace lerp_interface
{

class LERPPlannerManager : public planning_interface::PlannerManager
{
public:
  LERPPlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, [[maybe_unused]] const rclcpp::Node::SharedPtr& node, const std::string& /*ns*/) override
  {
    for (const std::string& gpName : model->getJointModelGroupNames())
    {
      RCLCPP_INFO(rclcpp::get_logger("LERPPlannerManager"), "group name %s", gpName.c_str());
      RCLCPP_INFO(rclcpp::get_logger("LERPPlannerManager"), "robot model %s", model->getName().c_str());

      planning_contexts_[gpName] =
          std::make_shared<LERPPlanningContext>("lerp_planning_context", gpName, model);
    }
    return true;
  }
  
  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "LERP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("lerp");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger("LERPPlannerManager"), "No group specified to plan for");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LERPPlannerManager"), "No planning scene supplied as input");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // create PlanningScene using hybrid collision detector
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
    ps->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());

    // retrieve and configure existing context
    const LERPPlanningContextPtr& context = planning_contexts_.at(req.group_name);
    RCLCPP_INFO(rclcpp::get_logger("LERPPlannerManager"), "===>>> context is made");

    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    return context;
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
      for (const auto& config : pcs)
      {
          RCLCPP_INFO(rclcpp::get_logger("LERPPlannerManager"), "Setting planner configuration for group %s", config.second.group.c_str());
      }
  }

protected:
  std::map<std::string, LERPPlanningContextPtr> planning_contexts_;
};

}  // namespace lerp_interface

// register the LERPPlannerManager class as a plugin
PLUGINLIB_EXPORT_CLASS(lerp_interface::LERPPlannerManager, planning_interface::PlannerManager)
