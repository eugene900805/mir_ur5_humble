#!/usr/bin/env python3

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

from scipy.spatial.transform import Rotation as R

def get_joint_pose():
    global moveit2
    # Get joint position
    joint_state = None
    while not joint_state:
        joint_state = moveit2.joint_state
    # print("Joint Names:", joint_state.name)
    # print("Joint Positions:", list(joint_state.position))
        
    # Get EEF pose
    end_effector_pose = moveit2.compute_fk(
        joint_state=joint_state,
    )[0]
    # print(end_effector_pose)
    eef_pose = [0,0,0,0,0,0,0]
    eef_pose[0] = end_effector_pose.pose.position.x
    eef_pose[1] = end_effector_pose.pose.position.y
    eef_pose[2] = end_effector_pose.pose.position.z
    eef_pose[3] = end_effector_pose.pose.orientation.x
    eef_pose[4] = end_effector_pose.pose.orientation.y
    eef_pose[5] = end_effector_pose.pose.orientation.z
    eef_pose[6] = end_effector_pose.pose.orientation.w
    
    return list(joint_state.position), eef_pose

def move_joint(joint_positions):
    global moveit2
    print(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

def move_to_pose(position, quat_xyzw, cartesian=False):
    global moveit2
    print(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

def move_to_pose2(position, r_xyz, cartesian=False):
    global moveit2
    r = R.from_euler('xyz', r_xyz)
    qx, qy, qz, qw = r.as_quat()
    quat_xyzw = [qx, qy, qz, qw]

    print(f"Moving to {{position: {list(position)}, r_xyzw: {list(r_xyz)}}}")
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    

def main():
    global moveit2
    rclpy.init()

    # Create node for this example
    node = Node("pymoveit2_test")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    joint_pos, eef_pos = get_joint_pose()
    print(joint_pos)

    # move_to_pose(position=[0.35, 0.10, 0.68], quat_xyzw=[1.0, 0.0, 0.0, 0.0])
    move_to_pose2(position=[0.35, 0.10, 0.68], r_xyz=[3.1415926, 0.0, 0.0])

    joint_pos, eef_pos = get_joint_pose()
    print(eef_pos)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
