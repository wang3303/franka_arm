//
// Created by michael on 1/17/19.
//

#include "pickplace.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    PickPlace pickplace;
    pickplace.addCollisionObjects();
    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();
    pickplace.pick();
    ros::WallDuration(1.0).sleep();
    pickplace.place();
    ros::waitForShutdown();
    return 0;
}