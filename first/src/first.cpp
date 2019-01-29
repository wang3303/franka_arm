//
// Created by michael on 1/16/19.
//

#include "first.h"

class Listener  {
    public:
      bool message_received;
      void trajCallback(const moveit_msgs::RobotTrajectory& msg);
      moveit_msgs::RobotTrajectory traj_reference; 
};

void Listener::trajCallback(const moveit_msgs::RobotTrajectory& msg) {
  traj_reference = msg; 
  message_received = true;
  ROS_INFO("Message from Python node received!");
}

void executeTrajectory(moveit::planning_interface::MoveGroupInterface &move_group,
            moveit_visual_tools::MoveItVisualTools &visualTools,
            const robot_state::JointModelGroup* joint_model_group,
            moveit_msgs::RobotTrajectory &trajectoryMsg,
            ros::Publisher &pub,
            double planning_time = 0.0) {
    geometry_msgs::Pose targetPose3 = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(targetPose3);
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory temp_trajectory;
    temp_trajectory = moveit_msgs::RobotTrajectory();
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, temp_trajectory);
   
    std::vector<double> joint_target = {};
    for (int i = 0; i < 7; i++) {
      joint_target.push_back(trajectoryMsg.joint_trajectory.points[0].positions[i]);
    }
    move_group.setJointValueTarget(joint_target);
    move_group.move();

    ROS_INFO("Beginning trajectory execution!");
    // trajectoryMsg.joint_trajectory.header.stamp = ros::Time::now();
    trajectoryMsg.joint_trajectory.header = temp_trajectory.joint_trajectory.header; 

    moveit::planning_interface::MoveGroupInterface::Plan myPlan;
    robot_trajectory::RobotTrajectory trajectory(move_group.getCurrentState()->getRobotModel(), "panda_arm");

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

    // execute the trajectory
    move_group.execute(myPlan);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "first_trial");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(30);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Listener listener;
    listener.message_received = false;

    ros::Publisher pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("traj", 1);
    ros::Subscriber sub = node_handle.subscribe("joint_traj", 1, &Listener::trajCallback, &listener);

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

    move_group.setNamedTarget("ready");
    move_group.move();

    while (ros::ok()) {
      ros::spinOnce();
      ROS_INFO("Waiting for message");

      if (listener.message_received) {
        executeTrajectory(move_group, visualTools, joint_model_group, listener.traj_reference, pub);
        ros::shutdown();
        return 0;
      }
      loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
