//
// Created by michael on 1/17/19.
//
#include "pickplace.h"

PickPlace::PickPlace(): moveGroup_(kGroupName) {
    moveGroup_.setPlanningTime(planningTime_);
}

void PickPlace::setGripperPosition(trajectory_msgs::JointTrajectory& posture, double position) {
    posture.joint_names.resize(2);
    posture.joint_names[0] = kFinger1;
    posture.joint_names[1] = kFinger2;

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = position;
    posture.points[0].positions[1] = position;
    posture.points[0].time_from_start = ros::Duration(openDuration_);
}

void PickPlace::openGripper(trajectory_msgs::JointTrajectory& posture) {
    setGripperPosition(posture, openPosition_);
}

void PickPlace::closedGripper(trajectory_msgs::JointTrajectory& posture) {
    setGripperPosition(posture, closePosition_);
}

void PickPlace::pick() {
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);
    grasps[0].grasp_pose.header.frame_id = kPlanFrame;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    // Setting grasp pose
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0.0;
    grasps[0].grasp_pose.pose.position.z = 0.5;
    // setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = kPlanFrame;
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;
    // setting post-grasp approach
    grasps[0].post_grasp_retreat.direction.header.frame_id = kPlanFrame;
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
    // setting pre-grasp and grasp pose
    openGripper(grasps[0].pre_grasp_posture);
    closedGripper(grasps[0].grasp_posture);

    moveGroup_.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    moveGroup_.pick("object", grasps);
}

void PickPlace::place() {
    std::vector<moveit_msgs::PlaceLocation> placeLocation;
    placeLocation.resize(1);

    placeLocation[0].place_pose.header.frame_id = kPlanFrame;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    placeLocation[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    placeLocation[0].place_pose.pose.position.x = 0;
    placeLocation[0].place_pose.pose.position.y = 0.5;
    placeLocation[0].place_pose.pose.position.z = 0.5;

    placeLocation[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    placeLocation[0].pre_place_approach.direction.vector.z = -1.0;
    placeLocation[0].pre_place_approach.min_distance = 0.095;
    placeLocation[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    placeLocation[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    placeLocation[0].post_place_retreat.direction.vector.y = -1.0;
    placeLocation[0].post_place_retreat.min_distance = 0.1;
    placeLocation[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(placeLocation[0].post_place_posture);

    // Set support surface as table2.
    moveGroup_.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    moveGroup_.place("object", placeLocation);
}

void PickPlace::addCollisionObjects() {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);
    //define table1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = kPlanFrame;

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;

    collision_objects[0].operation = collision_objects[0].ADD;

    // define table2
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = kPlanFrame;

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;

    collision_objects[1].operation = collision_objects[1].ADD;

    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = kPlanFrame;
    collision_objects[2].id = "object";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;

    collision_objects[2].operation = collision_objects[2].ADD;
    planningSceneInterface_.applyCollisionObjects(collision_objects);
}
