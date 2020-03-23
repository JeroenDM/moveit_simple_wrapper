#ifndef _MSRW_ROBOT_H
#define _MSRW_ROBOT_H

#include "types.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace moveit_simple_wrapper
{

class Robot
{
protected:
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  const robot_state::JointModelGroup *joint_model_group_;

  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  bool check_collisions_ = true;

  std::string tcp_frame_;

public:
  Robot(const std::string &tcp_frame = "tool0");
  ~Robot() = default;

  const Transform &fk(const std::vector<double> &q) const;
  const Transform &fk(const std::vector<double> &q, const std::string &frame) const;
  virtual IKSolution ik(const Transform &tf);
  Eigen::MatrixXd jacobian(const std::vector<double> &q);

  const Transform &getLinkFixedRelativeTransform(const std::string &name) const;
  void updatePlanningScene();
  bool isInCollision(const std::vector<double> &joint_pose) const;
  bool isPathColliding(const JointValues &q_from, const JointValues &q_to, int steps) const;
  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double> &joint_pose,
            const rviz_visual_tools::colors &color = rviz_visual_tools::DEFAULT);
};

} // namespace moveit_simple_wrapper
#endif