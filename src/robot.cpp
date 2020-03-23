#include "moveit_simple_wrapper/robot.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>

//#include <moveit/planning_scene/planning_scene.h>
#include "moveit_simple_wrapper/util.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace moveit_simple_wrapper
{
Robot::Robot(const std::string &tcp_frame) : tcp_frame_(tcp_frame)
{
    // load robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    // load robot state
    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup("manipulator");

    // create planning scene to for collision checking
    planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    updatePlanningScene();
}

const Transform &Robot::getLinkFixedRelativeTransform(const std::string &frame) const
{
    return kinematic_model_->getLinkModel(frame)->getJointOriginTransform();
}

const Transform &Robot::fk(const std::vector<double> &q) const
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    return kinematic_state_->getGlobalLinkTransform(tcp_frame_);
}

const Transform &Robot::fk(const std::vector<double> &q, const std::string &frame) const
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    return kinematic_state_->getGlobalLinkTransform(frame);
}

IKSolution Robot::ik(const Transform &tf)
{
    double timeout = 0.1;
    bool found_ik = kinematic_state_->setFromIK(joint_model_group_, tf, timeout);
    IKSolution sol;
    if (found_ik)
    {
        std::vector<double> joint_values;
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        sol.push_back(joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Failed to find ik solution.");
    }
    return sol;
}

Eigen::MatrixXd Robot::jacobian(const std::vector<double> &q)
{
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    return kinematic_state_->getJacobian(joint_model_group_);
}

void Robot::updatePlanningScene()
{
    // I'm not sure yet how this works
    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    planning_scene_ = ps->diff();
    planning_scene_->decoupleParent();
}

bool Robot::isInCollision(const std::vector<double> &joint_pose) const
{
    bool in_collision = false;

    // ROS_INFO("Checking for collision.");
    // planning_scene_->printKnownObjects(std::cout);

    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_pose);
    in_collision = planning_scene_->isStateColliding(*kinematic_state_);

    // ros::V_string links;
    // planning_scene_->getCollidingLinks(links, *kinematic_state_);
    // for (auto l : links)
    // {
    //     std::cout << l << "\n";
    // }
    return in_collision;
}

bool Robot::isPathColliding(const JointValues &q_from, const JointValues &q_to, int steps) const
{
    for (int step = 0; step < steps; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        if (isInCollision(q_step))
        {
            return true;
        }
    }
    return false;
}

void Robot::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double> &joint_pose,
                 const rviz_visual_tools::colors &color)
{
    namespace rvt = rviz_visual_tools;
    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_pose);
    mvt->publishRobotState(kinematic_state_, color);
    mvt->trigger();
}

} // namespace moveit_simple_wrapper