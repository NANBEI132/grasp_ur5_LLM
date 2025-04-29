#!/usr/bin/env python3

import rospy
import json
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys  # 添加这一行，导入 sys 模块

def load_json(file_path):
    """加载 JSON 文件"""
    with open(file_path, 'r') as f:
        return json.load(f)

def control_gripper(gripper_action, force=20):
    """控制夹爪"""
    pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=10)
    rospy.sleep(1)  # 等待发布者准备好
    if gripper_action == "close":
        rospy.loginfo("Closing gripper with force: {}".format(force))
        pub.publish(Float64(-force))  # 关闭夹爪
    elif gripper_action == "open":
        rospy.loginfo("Opening gripper with force: {}".format(force))
        pub.publish(Float64(force))   # 打开夹爪

def move_to_target(move_group, target_position, speed=0.1, acceleration=0.05):
    """移动到目标位置"""
    pose_target = Pose()
    pose_target.position.x = target_position[0]
    pose_target.position.y = target_position[1]
    pose_target.position.z = target_position[2]

    # 设置速度和加速度
    move_group.set_max_velocity_scaling_factor(speed)
    move_group.set_max_acceleration_scaling_factor(acceleration)

    # 设置目标位置并规划运动
    move_group.set_pose_target(pose_target)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def execute_trajectory(joint_trajectory):
    """执行轨迹"""
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rospy.sleep(1)  # 等待发布者准备好

    goal = FollowJointTrajectoryActionGoal()
    goal.goal.trajectory = joint_trajectory

    rospy.loginfo("Publishing trajectory to /scaled_pos_joint_traj_controller/follow_joint_trajectory/goal")
    pub.publish(goal)

def main():
    # 初始化 ROS 节点
    rospy.init_node('ur5_moveit_json_controller', anonymous=True)

    # 初始化 MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)  # 这里需要 sys 模块
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # UR5 的规划组名称
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 加载 JSON 动作序列
    json_file = "/home/zzq/UR5-Pick-and-Place-Simulation/catkin_ws/src/motion_planning/json/test.json"  # 替换为您的 JSON 文件路径
    data = load_json(json_file)

    # 执行动作序列
    for action in data.get("actions", []):
        action_type = action.get("type")
        if action_type == "move":
            target_position = action.get("target_position")
            speed = action.get("speed", 0.1)
            acceleration = action.get("acceleration", 0.05)
            rospy.loginfo("Moving to target position: {}".format(target_position))
            move_to_target(move_group, target_position, speed, acceleration)

        elif action_type == "gripper":
            gripper_action = action.get("action")
            force = action.get("force", 20)
            control_gripper(gripper_action, force)

        else:
            rospy.logwarn("Unsupported action type: {}".format(action_type))

    # 关闭 MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
