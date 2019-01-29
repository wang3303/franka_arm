//
// Created by michael on 1/16/19.
//

#include "first.h"

void executeTrajectory(moveit::planning_interface::MoveGroupInterface &move_group,
            moveit_visual_tools::MoveItVisualTools &visualTools,
            const robot_state::JointModelGroup* joint_model_group,
            moveit_msgs::RobotTrajectory &trajectoryMsg,
            ros::Publisher &pub,
            double planning_time = 0.0) {
    pub.publish(trajectoryMsg);
    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    robot_trajectory::RobotTrajectory trajectory(move_group.getCurrentState()->getRobotModel(), "panda_arm");

    visualTools.prompt("Going to set the trajectory!");

    // set planning time
    myPlan.planning_time_ = planning_time;

    // set robot start state
    moveit_msgs::RobotState robotStartStateMsg;
    const moveit::core::RobotState currentState = *move_group.getCurrentState();
    robot_state::robotStateToRobotStateMsg(currentState, robotStartStateMsg);
    myPlan.start_state_ = robotStartStateMsg;

    // set robot robot trajectory
    trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectoryMsg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(trajectory);
    ROS_INFO_NAMED("first", "compute time stamps %s", success? "succeeds" : "fails");

    trajectory.getRobotTrajectoryMsg(trajectoryMsg);
    myPlan.trajectory_ = trajectoryMsg;
    pub.publish(trajectoryMsg);

    visualTools.prompt("Going to execute");

    // execute the trajectory
    move_group.execute(myPlan);


}


void testExecuteTraj(moveit::planning_interface::MoveGroupInterface &move_group,
                     moveit_visual_tools::MoveItVisualTools &visualTools,
                     const robot_state::JointModelGroup* joint_model_group,
                     ros::Publisher &pub) {
    // Cartesian Paths
    geometry_msgs::Pose targetPose3 = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(targetPose3);
/*    targetPose3.position.z -= 0.1;
    waypoints.push_back(targetPose3);
    targetPose3.position.y -= 0.2;
    waypoints.push_back(targetPose3);
    targetPose3.position.z += 0.2;
    targetPose3.position.y += 0.2;
    targetPose3.position.x -= 0.2;
    waypoints.push_back(targetPose3);  // up and left
*/
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    trajectory = moveit_msgs::RobotTrajectory();
    pub.publish(trajectory);
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    std::vector<double> init_joint = move_group.getCurrentJointValues();
    for (int i = 0; i < 10; i++) {
        trajectory_msgs::JointTrajectoryPoint temp_point;
        temp_point.positions = init_joint;
        temp_point.velocities = std::vector<double>(7, 0);
        temp_point.accelerations = std::vector<double>(7, 0);
        init_joint[1] += 0.01;
        trajectory.joint_trajectory.points.push_back(temp_point);

    }
    pub.publish(trajectory);
    executeTrajectory(move_group, visualTools, joint_model_group, trajectory, pub);
}

void trajCallback(const moveit_msgs::RobotTrajectory& msg)
{
    ROS_INFO("I heard: []");
}

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

    visualTools.prompt("Ready to start");

    ros::Publisher pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("traj", 1);
    //ros::Subscriber sub = node_handle.subscribe("traj_listener", 1000, trajCallback);

    //ros::spin();
    testExecuteTraj(move_group, visualTools, joint_model_group, pub);
    ros::shutdown();

    return 0;
}
