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
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;
    const robot_state::JointModelGroup* joint_model_group_;

    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    bool check_collisions_ = true;

    std::string planning_group_;
    std::string tcp_name_;

    /** \brief default timeout for inverse kinematic calculations.
     * This is used in the default solver by MoveIt.
     * I do not except from analytical solvers to use this.
     * */
    const double DEFAULT_IK_TIMEOUT_ = 0.1;

  public:
    /** Main constructor.
     *
     * Can be used after the "demo.launch" file is launched
     * from an existing MoveIt config for a robot.
     * */
    Robot(const std::string& planning_group = "manipulator", const std::string& tcp_name = "tool0");

    /** Constructor that takes an existing robot model.
     *
     * Mainly useful for testing.
     * */
    Robot(robot_model::RobotModelPtr robot_model, const std::string& planning_group = "manipulator",
          const std::string& tcp_name = "tool0");

    ~Robot() = default;

    /** \brief Calculate forward kinematics for default tcp_name (tool center point). */
    const Transform& fk(const JointPositions& q) const;

    /** \brief Calculate forward kinematics for a specific link of the robot. */
    const Transform& fk(const JointPositions& q, const std::string& frame) const;

    /** \Brief Calculate a single inverse kinematic solutions.
     *
     * This method has a default implementation and expects the robot has some
     * kind of inverse kinematics plugin active.
     * It is assumed this is true for most robot arms.
     *
     * \return Returns an emtpy vector if no solution is found.
     * */
    virtual IKSolution ik(const Transform& tf);

    /** \brief Calculate geometric jacobian. */
    Eigen::MatrixXd jacobian(const JointPositions& q);

    const Transform& getLinkFixedRelativeTransform(const std::string& name) const;
    void updatePlanningScene();
    bool isColliding(const JointPositions& joint_pose) const;
    bool isPathColliding(const JointPositions& q_from, const JointPositions& q_to, int steps) const;
    void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, JointPositions& joint_pose,
              const rviz_visual_tools::colors& color = rviz_visual_tools::DEFAULT);
};

}  // namespace moveit_simple_wrapper
#endif