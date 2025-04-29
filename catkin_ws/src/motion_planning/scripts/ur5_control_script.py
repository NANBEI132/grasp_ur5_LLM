#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi


def main():
    # 初始化MoveIt!和ROS节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_loop_move_group_python_interface', anonymous=True)

    # 创建MoveGroupCommander对象
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 获取规划参考坐标系
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # 获取末端执行器的名称
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # 获取所有可移动的关节
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # 打印机器人的当前状态
    print("Robot state:", robot.get_current_state())

    # 调整初始关节值
    init_joint_goal = move_group.get_current_joint_values()
    init_joint_goal[0] = 0
    init_joint_goal[1] = -pi / 4
    init_joint_goal[2] = pi / 4
    init_joint_goal[3] = 0
    init_joint_goal[4] = 0
    init_joint_goal[5] = 0

    move_group.go(init_joint_goal, wait=True)
    move_group.stop()

    # 定义抬手和放手的关节值
    up_joint_goal = move_group.get_current_joint_values()
    up_joint_goal[0] = 0
    up_joint_goal[1] = -pi / 2
    up_joint_goal[2] = pi / 2
    up_joint_goal[3] = 0
    up_joint_goal[4] = 0
    up_joint_goal[5] = 0

    down_joint_goal = move_group.get_current_joint_values()
    down_joint_goal[0] = 0
    down_joint_goal[1] = 0
    down_joint_goal[2] = 0
    down_joint_goal[3] = 0
    down_joint_goal[4] = 0
    down_joint_goal[5] = 0

    # 设定循环次数
    loop_count = 5

    for i in range(loop_count):
        print(f"Loop {i + 1}: Moving arm up...")
        # 规划并执行抬手动作
        move_group.go(up_joint_goal, wait=True)
        move_group.stop()
        rospy.sleep(1)  # 等待1秒

        print(f"Loop {i + 1}: Moving arm down...")
        # 规划并执行放手动作
        move_group.go(down_joint_goal, wait=True)
        move_group.stop()
        rospy.sleep(1)  # 等待1秒

    # 关闭MoveIt!和ROS节点
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
