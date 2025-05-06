#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class GripperController:
    def __init__(self):
        # 移除 rospy.init_node 调用，因为节点已经在 motion_plan2ros4.py 中初始化
        self.client = actionlib.SimpleActionClient(
            "/gripper_controller/gripper_cmd",
            GripperCommandAction
        )
        self.client.wait_for_server()
        self.gripper_position = 0.0  # 初始状态为打开

    def set_gripper(self, position):
        """控制夹爪开合位置 (0.0: 全开, 0.8: 全闭)"""
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = -1  # 最大力度
        self.client.send_goal(goal)
        self.client.wait_for_result()
        self.gripper_position = position
        return self.client.get_result()

    def open(self):
        """完全打开夹爪"""
        return self.set_gripper(0.0)

    def close(self):
        """部分闭合夹爪（无需实际抓取）"""
        return self.set_gripper(0.7)  # 调整为部分闭合

