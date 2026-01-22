#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from rm_msgs.msg import MoveJ_P, Plan_State
from tf.transformations import quaternion_from_euler

class RM_Robot_Controller:
    def __init__(self):
        rospy.init_node('api_moveJ_P_flexible_demo', anonymous=True)
        # 发布位姿指令的话题
        self.pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
        # 订阅执行结果的话题
        self.sub = rospy.Subscriber("/rm_driver/Plan_State", Plan_State, self.plan_state_callback)
        self.is_moving = False
        rospy.sleep(1.0) # 等待话题连接建立

    def plan_state_callback(self, msg):
        if msg.state:
            rospy.loginfo("******* 运动目标已达成")
        else:
            rospy.logwarn("******* 运动目标执行失败")
        self.is_moving = False

    def move_arm(self, x, y, z, orientation, input_type="euler", speed=0.2):
        """
        x, y, z: 单位米
        orientation: 如果是euler, 格式为 [roll, pitch, yaw] (单位：度)
                     如果是quaternion, 格式为 [x, y, z, w]
        input_type: "euler" 或 "quaternion"
        """
        msg = MoveJ_P()
        msg.speed = speed
        msg.Pose.position.x = x
        msg.Pose.position.y = y
        msg.Pose.position.z = z

        if input_type == "euler":
            # 将角度转换为弧度
            r = math.radians(orientation[0])
            p = math.radians(orientation[1])
            y_rad = math.radians(orientation[2])
            # 欧拉角转四元数
            q = quaternion_from_euler(r, p, y_rad)
            msg.Pose.orientation.x = q[0]
            msg.Pose.orientation.y = q[1]
            msg.Pose.orientation.z = q[2]
            msg.Pose.orientation.w = q[3]
            rospy.loginfo("使用欧拉角输入控制...")
        
        elif input_type == "quaternion":
            msg.Pose.orientation.x = orientation[0]
            msg.Pose.orientation.y = orientation[1]
            msg.Pose.orientation.z = orientation[2]
            msg.Pose.orientation.w = orientation[3]
            rospy.loginfo("使用四元数输入控制...")

        self.is_moving = True
        self.pub.publish(msg)
        
        # 等待运动完成（或者你可以根据需要去掉 spin，让程序执行完直接结束）
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = RM_Robot_Controller()

        # --- 方式一：使用欧拉角（最直观，单位：度） ---
        # 比如让末端垂直向下，通常 roll 可能接近 180 度
        # controller.move_arm(0.3, 0.1, 0.3, [180.0, 0.0, 0.0], input_type="euler")

        # --- 方式二：使用四元数（精确，直接透传） ---
        q_data = [0.0, -0.707, 0.0, 0.707]
        # q_data = [0.0, 0.0, 1.0, 0.0]
        controller.move_arm(0.15, 0, 0.45, q_data, input_type="quaternion")

    except rospy.ROSInterruptException:
        pass