#include <fstream>

#include "first.h"

# define M_PI           3.14159265358979323846

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
    primitive.dimensions[0] = widths[idx][0];
    primitive.dimensions[1] = widths[idx][1];
    primitive.dimensions[2] = widths[idx][2];

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = corners[idx][0] + 0.5*widths[idx][0]; 
    box_pose.position.y = corners[idx][1] + 0.5*widths[idx][1]; 
    box_pose.position.z = corners[idx][2] + 0.5*widths[idx][2]; 

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
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visualTools("panda_link0");
  // visualTools.deleteAllMarkers();
  // visualTools.loadRemoteControl();

  // Eigen::Affine3d textPose = Eigen::Affine3d::Identity();
  // textPose.translation().z() = 1.75;
  // visualTools.publishText(textPose, "First Trial", rvt::WHITE, rvt::XLARGE);

  // visualTools.trigger();

  // // Logging
  // ROS_INFO_NAMED("first", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  // ROS_INFO_NAMED("first", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("first", "Move to start");

  // Add collision objects
  std::vector<moveit_msgs::CollisionObject> collisionObjects = add_collision_objects(move_group); 
  planning_scene_interface.addCollisionObjects(collisionObjects);

  // visualTools.prompt("Objects are added. Press next.");

  // moveit::planning_interface::MoveGroupInterface::Plan myPlan;
  // 
  // std::vector<double> th_goal = {-0.105115, 1.58432, -1.53984, -0.183412, 0.668877, 1.25792, -2.89474}; 
  // std::vector<double> th_goal2 = {-0.287769, -0.828924, 0.00223236, -2.71563, 0.627072, 0.966626, -1.24478};

  // move_group.setJointValueTarget(th_goal);
  // bool success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // 
  // visualTools.deleteAllMarkers();
  // visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  // visualTools.trigger();
  // visualTools.prompt("Press next");

  // move_group.execute(myPlan);

  // move_group.setJointValueTarget(th_goal2);
  // success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("first", "visualizing plan with obstacles " + success? "": "FAILED");
  // visualTools.deleteAllMarkers();
  // visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  // visualTools.trigger();
  // visualTools.prompt("Press next");
  // 
  // move_group.execute(myPlan);
  // 
  // std::vector<double> th_goal3 = {0., 0., 0., -0.07, 0., 0., 0.};
  // move_group.setJointValueTarget(th_goal2);
  // success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // 
  // ROS_INFO_NAMED("first", "visualizing plan with obstacles " + success? "": "FAILED");
  // visualTools.deleteAllMarkers();
  // visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  // visualTools.trigger();
  // visualTools.prompt("Press next");

  // move_group.execute(myPlan);

  // std::vector<std::string> object_ids;
  // object_ids.push_back(collisionObject.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);
  int n_th = 10;
  std::vector<std::vector<double> > th_inits(n_th, std::vector<double>(7,1));
  // th_inits[0] = {0,0,0,-0.07,0,0,0};
  th_inits[0] = {0.597223, 0.37125, -0.0100986, -2.37541, 0.180311, 1.03483, 0.0706328};
  th_inits[1] = {0,  -0.6, 0., -0.20*M_PI, 0.25, 1.10*M_PI, 0.25*M_PI};
  th_inits[2] = {M_PI/4,  0.50, 0, -0.5*M_PI, 0.25,  0.5*M_PI, 0.25*M_PI};
  th_inits[3] = {-M_PI/5, -0.65, 1., -0.5*M_PI, 0.25,  0.5*M_PI, 0.25*M_PI};
  th_inits[4] = {M_PI/2,M_PI/4,0,-0.07,0,0.5,0};
  th_inits[5] = {-1.62393, -1.22264, -0.406409, -0.4119, -1.76969, 2.34654, -2.43675};
  th_inits[6] = {1.85565, -1.51793, -1.16217, -2.2597, -0.53239, 2.14503, -2.77667};
  th_inits[7] = {0.844936, -1.01263, -.7381, -2.31455, -0.247229, 2.31731, 0.970091};
  th_inits[8] = {1.5964, 0.0857214, -2.28602, -0.676237, 1.77197, 0.840032, 2.88113};
  th_inits[9] = {0.235829, -0.337666, 1.1378, -2.22957, 2.37467, 1.40022, -1.20456};
  
  std::vector<std::vector<double> > th_goals(n_th, std::vector<double>(7,1));
  th_goals[0] = {0,  +0.1, 0., -0.1*M_PI, 0.25, 1.72, 0.25*M_PI};
  th_goals[1] = {-M_PI/5, 0.65, 0, -0.5*M_PI, 0.25,  0.75*M_PI, 0.25*M_PI};
  th_goals[2] = {-0.105115, 1.58432, -1.53984, -0.183412, 0.668877, 1.25792, -2.89474};
  th_goals[3] = {0.130862, -0.70124, 2.0121, -1.69346, 0.674449,2.58168, 0.317337};
  th_goals[4] = {-0.0718304, -0.761838, 1.858, -1.78156, -2.20838, 0.123241, -1.39996};
  th_goals[4] = {-1.98575, 0.592403, -0.491794, -0.824348, -1.86245, 1.57712, -0.905226};
  th_goals[5] = {1.93431, 0.256879, 1.17659, -0.0998391, -0.455313, 0.33823, 1.43323};
  th_goals[6] = {-0.156785, -1.06, 0.992719, -1.21834, 2.38384, 0.340514, -1.74002};
  th_goals[7] = {-1.49469, 0.377673, -0.986205, -1.50891, -0.476247, 1.55533, -0.311577};
  th_goals[8] = {-0.560172, -0.941266, 0.0038547, -1.89165, -0.37443, 1.39094, -2.10081};
  th_goals[9] = {-0.287769,-0.828924,0.00223236,-2.71563,0.627072, 0.966626, -1.24478};

  bool success;
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;
  for (int th_init_idx = 0; th_init_idx < th_inits.size(); th_init_idx++) {
    move_group.setJointValueTarget(th_inits[th_init_idx]);
    success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.execute(myPlan);
    // if (!success) {
    //   ROS_WARN("Didn't reach th_init %d", th_init_idx);
    // } else {
    //   ROS_INFO("Reached th_init %d", th_init_idx);
    // };

    for (int th_goal_idx = 0; th_goal_idx < th_goals.size(); th_goal_idx++) {
      move_group.setJointValueTarget(th_goals[th_goal_idx]);
      success = (move_group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        std::vector<double> joint_trajectory;

        std::cout << th_init_idx+1 << "\n";
        std::cout << th_goal_idx+1 << "\n";
        for (int point_idx=0; point_idx < myPlan.trajectory_.joint_trajectory.points.size(); point_idx++) {
          trajectory_msgs::JointTrajectoryPoint p = myPlan.trajectory_.joint_trajectory.points[point_idx];
          for (int q_idx = 0; q_idx < p.positions.size(); q_idx++) {
            joint_trajectory.push_back(p.positions[q_idx]);
            std::cout << p.positions[q_idx] << " ";
          }
          std::cout << "\n";
        }
        // ROS_INFO("Number of points: %d", joint_trajectory.size()); 
        std::cout << "\n\n\n";
        // ROS_INFO("th_init_idx: %d", th_init_idx);
        // ROS_INFO("th_goal_idx: %d", th_goal_idx);

      } else {
       //  ROS_WARN("th_init_idx: %d", th_init_idx);
        ROS_WARN("th_goal_idx: %d", th_goal_idx);
      }
    }
  }

  ros::shutdown();

  return 0;
}
