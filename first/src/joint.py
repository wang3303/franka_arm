import sys
import copy
import rospy
import moveit_msgs.msg
import trajectory_msgs.msg
import h5py
import numpy as np
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
  
# using HDF5
# fn = "/home/acauligi/franka_test/src/joint_reference/src/test.h5"
# h5write(fn, "joint_traj", joint_traj)

def main():
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  
  n_joints = 7

  fn = 'test.h5'
  f = h5py.File(fn, 'r')
  data = list(f['joint_traj'])

  robot_trajectory_publisher = rospy.Publisher('joint_traj',
                                                 moveit_msgs.msg.RobotTrajectory,
                                                 queue_size=20)

  robot_trajectory = moveit_msgs.msg.RobotTrajectory()
  robot_trajectory.joint_trajectory.header.frame_id = "/panda_link0"
  joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
  for joint_name in joint_names:
    robot_trajectory.joint_trajectory.joint_names.append(joint_name)

  for datum in data:
    new_pt = trajectory_msgs.msg.JointTrajectoryPoint()
    for i in range(n_joints):
      new_pt.positions.append(datum[i])
      new_pt.velocities.append(0.)
      new_pt.accelerations.append(0.)
    robot_trajectory.joint_trajectory.points.append(new_pt)

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    rospy.loginfo('Hello world')
    robot_trajectory_publisher.publish(robot_trajectory)
    rate.sleep()

if __name__ == '__main__':
  main()
