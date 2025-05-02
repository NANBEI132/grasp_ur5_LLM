#!/usr/bin/python3

import os
import math
import rospy
from controller import ArmController
from pyquaternion import Quaternion as PyQuaternion
from geometry_msgs.msg import Point
from gripper_control import GripperController  # 新的夹爪控制模块

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

# 机械臂默认参数
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
DEFAULT_POS = (-0.1, -0.2, 1.2)  # 初始位置
PLACE_POS = (0.264589, -0.293903, 0.777)  # 固定放置位置

class LegoManipulator:
    def __init__(self):
        self.controller = ArmController()
        self.gripper = GripperController()  # 使用新的夹爪控制器
        self.place_coords_pub = rospy.Publisher('/place_coords', Point, queue_size=10)

        # 初始化机械臂和夹爪
        self.controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
        self.gripper.open()

        # 订阅话题
        rospy.Subscriber("/pickup_coords", Point, self.pickup_callback)
        rospy.Subscriber("/place_coords", Point, self.place_callback)

        self.is_busy = False

    def move_to_position(self, x, y, z, quaternion=DEFAULT_QUAT):
        """移动到指定位置"""
        rospy.loginfo("Moving to position: x=%.2f, y=%.2f, z=%.2f", x, y, z)
        self.controller.move_to(x, y, z, target_quat=quaternion)
        rospy.sleep(2)  # 等待运动完成

    def pickup_callback(self, msg):
        """处理夹取指令"""
        rospy.loginfo("Received pickup coordinates: x=%.2f, y=%.2f, z=%.2f", msg.x, msg.y, msg.z)
        if not self.is_busy:
            self.is_busy = True
            try:
                # 移动到夹取位置（上方）
                self.move_to_position(msg.x, msg.y, msg.z + 0.1, DEFAULT_QUAT)

                # 下降到目标高度
                self.move_to_position(msg.x, msg.y, msg.z, DEFAULT_QUAT)

                # 完全闭合夹爪
                rospy.loginfo("Closing gripper completely...")
                self.gripper.close()  # 使用新的夹爪闭合方法
                rospy.loginfo("Gripper is fully closed.")

                # 自动发布放置坐标
                place_point = Point()
                place_point.x, place_point.y, place_point.z = PLACE_POS
                self.place_coords_pub.publish(place_point)

            except Exception as e:
                rospy.logerr(f"Pickup failed: {str(e)}")
            finally:
                self.is_busy = False

    def place_callback(self, msg):
        """处理放置指令"""
        rospy.loginfo("Received place coordinates: x=%.2f, y=%.2f, z=%.2f", msg.x, msg.y, msg.z)
        if not self.is_busy and self.gripper.gripper_position > 0.0:
            self.is_busy = True
            try:
                # 移动到放置位置（上方）
                self.move_to_position(msg.x, msg.y, msg.z + 0.1, DEFAULT_QUAT)

                # 下降到目标高度
                self.move_to_position(msg.x, msg.y, msg.z, DEFAULT_QUAT)

                # 打开夹爪释放物体
                self.gripper.open()
                rospy.loginfo("Object released.")

                # 返回初始位置
                self.move_to_position(*DEFAULT_POS, DEFAULT_QUAT)
                rospy.loginfo("Arm returned to initial position.")

            except Exception as e:
                rospy.logerr(f"Placement failed: {str(e)}")
            finally:
                self.is_busy = False

if __name__ == "__main__":
    rospy.init_node("lego_manipulator")
    manipulator = LegoManipulator()
    rospy.spin()

