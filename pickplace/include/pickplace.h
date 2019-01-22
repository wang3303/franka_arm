//
// Created by michael on 1/17/19.
//

#pragma once

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class PickPlace {
public:
    PickPlace();
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void pick();
    void place();
    void addCollisionObjects();

private:
    void setGripperPosition(trajectory_msgs::JointTrajectory& posture, double position);
    const std::string kFinger1 = "panda_finger_joint1";
    const std::string kFinger2 = "panda_finger_joint2";
    const std::string kPlanFrame = "panda_link0";
    const std::string kGroupName = "panda_arm";
    double openPosition_ = 0.04;
    double openDuration_ = 0.5;
    double closePosition_ = 0.0;
    double planningTime_ = 45.0;
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;
    moveit::planning_interface::MoveGroupInterface moveGroup_;
};