#include "first.h"

std::vector<moveit_msgs::CollisionObject> add_collision_objects(moveit::planning_interface::MoveGroupInterface &move_group) {
  int n_obs = 9; 

  std::vector<std::vector<double> > corners(n_obs, std::vector<double>(3,1));
  corners[0] = {0.45,0.1,0.0};
  corners[1] = {0.45,0.1,0.7};
  corners[2] = {-0.2,-0.3,0.3};
  corners[3] = {-0.5,0.3,0.1};
  corners[4] = {0.0,0.65,0.40};
  corners[5] = {-0.45,0.0,0.4};
  corners[6] = {0.50,0.50,0.2};
  corners[7] = {-0.4,0.0,0.8};
  corners[8] = {0.4,-0.3,0.6};

  std::vector<std::vector<double> > widths(n_obs, std::vector<double>(3,1));
  widths[0] = {0.10,0.10,0.10};
  widths[1] = {0.10,0.10,0.10};
  widths[2] = {0.10,0.10,0.10};
  widths[3] = {0.10,0.10,0.10};
  widths[4] = {0.10,0.10,0.10};
  widths[5] = {0.10,0.10,0.10};
  widths[6] = {0.10,0.10,0.10};
  widths[7] = {0.10,0.10,0.10};
  widths[8] = {0.10,0.10,0.10};

  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  for (int idx = 0; idx < n_obs; idx++) {
    moveit_msgs::CollisionObject collisionObject;

    std::string box_name = "box_id_" + std::to_string(idx);
    collisionObject.id = box_name; 
    collisionObject.header.frame_id = move_group.getPlanningFrame();

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = widths[idx][0]
    primitive.dimensions[1] = widths[idx][1];
    primitive.dimensions[2] = widths[idx][2];

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = corners[idx][0]; 
    box_pose.position.y = corners[idx][1]; 
    box_pose.position.z = corners[idx][2]; 

    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(box_pose);
    collisionObject.operation = collisionObject.ADD;

    collisionObjects.push_back(collisionObject);
  }
  return collisionObjects;
}

int main(int argc, char** argv) {
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

  // Add collision objects
  std::vector<moveit_msgs::CollisionObject> collisionObjects = add_collision_objects(move_group); 
  planning_scene_interface.addCollisionObjects(collisionObjects);

  visualTools.prompt("Objects are added. Press next.");

  std::vector<double> th_init = {0., 0., 0., -0.07, 0., 0., 0.}; 
  moveit_msgs::RobotState state_init;
  state_init.joint_state.position = th_init;
  move_group.setStartState(state_init);
  
  std::vector<double> th_goal = {-0.105115, 1.58432, -1.53984, -0.183412, 0.668877, 1.25792, -2.89474}, 
  move_group.setJointValueTarget(th_goal);

  success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("first", "visualizing plan with obstacles " + success? "": "FAILED");
  visualTools.deleteAllMarkers();
  visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visualTools.trigger();
  visualTools.prompt("Press next");

  move_group.execute(myPlan);

  // std::vector<std::string> object_ids;
  // object_ids.push_back(collisionObject.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  ros::shutdown();

  return 0;
}
