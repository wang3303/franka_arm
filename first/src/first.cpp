//
// Created by michael on 1/16/19.
//

#include "first.h"

int main(int argc, char** argv) {

    // init node
    ros::init(argc, argv, "first_trial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string kPlanningGroup = "panda_arm";

    // planning interface init
    moveit::planning_interface::MoveGroupInterface move_group(kPlanningGroup);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(kPlanningGroup);

    // visualization init
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools("panda_link0");
    visualTools.deleteAllMarkers();
    visualTools.loadRemoteControl();

    Eigen::Affine3d textPose = Eigen::Affine3d::Identity();
    textPose.translation().z() = 1.75;
    visualTools.publishText(textPose, "First Trial", rvt::WHITE, rvt::XLARGE);

    visualTools.trigger();

    // Logging
    ROS_INFO_NAMED("first", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("first", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("first", "Move to start");

    move_group.setNamedTarget("ready");
    move_group.move();

    visualTools.prompt("Press 'next'");

    // planning to a pose goal
    geometry_msgs::Pose targetPose;
    targetPose.orientation.w = 1.0;
    targetPose.position.x = 0.28;
    targetPose.position.y = -0.2;
    targetPose.position.z = 0.5;
    move_group.setPoseTarget(targetPose);

    // compute plan
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("first", "Visualizing plan 1 %S", success ? "" : "FAILED!");

    // visualizing plans
    ROS_INFO_NAMED("first", "Visualizing plan");
    visualTools.publishAxisLabeled(targetPose, "pose1");
    visualTools.publishText(textPose, "pose Goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
    visualTools.trigger();
    visualTools.prompt("press Next");

    // move the robot
    move_group.execute(myPlan);

    // move to joint space goal
    moveit::core::RobotStatePtr currentState = move_group.getCurrentState();
    std::vector<double> jointGroupPosition;
    currentState->copyJointGroupPositions(joint_model_group, jointGroupPosition);

    jointGroupPosition[0] = -1.0;
    move_group.setJointValueTarget(jointGroupPosition);
    success = move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    ROS_INFO_NAMED("first", "Visualizing plan");
    visualTools.publishAxisLabeled(targetPose, "pose2");
    visualTools.publishText(textPose, "pose Goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
    visualTools.trigger();
    visualTools.prompt("press Next");

    move_group.execute(myPlan);

    // plan with path constraint
    robot_state::RobotState startState(*move_group.getCurrentState());
    geometry_msgs::Pose startPose2;
    startPose2.orientation.w = 1.0;
    startPose2.position.x = 0.55;
    startPose2.position.y = -0.05;
    startPose2.position.z = 0.8;
    startState.setFromIK(joint_model_group, startPose2);
    move_group.setStartState(startState);


    visualTools.prompt("press Next");
    move_group.setPoseTarget(startPose2);
    move_group.move();

    move_group.setStartStateToCurrentState();
    // set constrain
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.emplace_back(ocm);
    move_group.setPathConstraints(test_constraints);
    move_group.setPoseTarget(targetPose);

    move_group.setPlanningTime(10.0);
    success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    visualTools.deleteAllMarkers();
    visualTools.publishAxisLabeled(move_group.getCurrentPose().pose, "start");
    visualTools.publishAxisLabeled(targetPose, "goal");
    visualTools.publishText(textPose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
    visualTools.trigger();
    visualTools.prompt("next step");

    move_group.execute(myPlan);
    move_group.clearPathConstraints();
    move_group.setStartStateToCurrentState();

    // Cartesian Paths
    geometry_msgs::Pose targetPose3 = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(targetPose3);
    targetPose3.position.z -= 0.2;
    waypoints.push_back(targetPose3);
    targetPose3.position.y -= 0.2;
    waypoints.push_back(targetPose3);
    targetPose3.position.z += 0.2;
    targetPose3.position.y += 0.2;
    targetPose3.position.x -= 0.2;
    waypoints.push_back(targetPose3);  // up and left

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("first", "Visualizing plan");

    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//    visualTools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//    for (std::size_t i = 0; i < waypoints.size(); ++i)
//        visualTools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visualTools.publishTrajectoryLine(trajectory, joint_model_group);
    visualTools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visualTools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan myPlan2;

//    move_group.setPoseTarget(move_group.getCurrentPose().pose);
//    move_group.plan(myPlan);
    myPlan2.trajectory_ = trajectory;
    move_group.execute(myPlan2);

    // Adding/Removing Objects and Attaching/Detaching Objects
    moveit_msgs::CollisionObject collisionObject;
    collisionObject.header.frame_id = move_group.getPlanningFrame();

    collisionObject.id = "box";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.2;
    box_pose.position.z = 1.0;
    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(box_pose);
    collisionObject.operation = collisionObject.ADD;

    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.push_back(collisionObject);
    ROS_INFO_NAMED("first", "add a collision object");
    planning_scene_interface.addCollisionObjects(collisionObjects);
    visualTools.prompt("Objects are added. Press next.");

    move_group.setStartStateToCurrentState();
    geometry_msgs::Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = 0.4;
    another_pose.position.y = -0.4;
    another_pose.position.z = 0.9;
    move_group.setPoseTarget(another_pose);

    success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("first", "visualizing plan with obstacles " + success? "": "FAILED");
    visualTools.deleteAllMarkers();
    visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
    visualTools.trigger();
    visualTools.prompt("Press next");

    move_group.execute(myPlan);

    std::vector<std::string> object_ids;
    object_ids.push_back(collisionObject.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    ros::shutdown();

    return 0;
}
