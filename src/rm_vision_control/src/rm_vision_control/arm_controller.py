#!/usr/bin/env python

# src/rm_vision_control/scripts/arm_controller.py
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from rm_msgs.msg import MoveJ, MoveJ_P, MoveL, Plan_State, Arm_Current_State, GetArmState_Command
from std_msgs.msg import Int32, Float32

class ArmController:
    def __init__(self):
        # ==== 需要根据实际情况修改的配置 ====
        # 机械臂初始位置（零点）
        self.home_pose = Pose()
        self.home_pose.position.x = 0.0     # 请根据实际情况修改
        self.home_pose.position.y = 0.0      # 请根据实际情况修改  
        self.home_pose.position.z = 0.85      # 请根据实际情况修改
        self.home_pose.orientation.x = 0
        self.home_pose.orientation.y = 0
        self.home_pose.orientation.z = 1.0
        self.home_pose.orientation.w = 0

        horizontal_orientation = self.euler_to_quaternion(roll=np.pi/2, pitch=0, yaw=0)
        
        # 观测姿态pose1和pose2 
        self.pose1 = Pose()
        self.pose1.position.x = - 0.3          # 请根据实际情况修改
        self.pose1.position.y = 0.0          # 请根据实际情况修改
        self.pose1.position.z = 0.65          # 请根据实际情况修改
        self.pose1.orientation.x = 0.0
        self.pose1.orientation.y = -0.707
        self.pose1.orientation.z = 0.0
        self.pose1.orientation.w = 0.707
        
        self.pose2 = Pose()
        self.pose2.position.x = self.pose1.position.x
        self.pose2.position.y = self.pose1.position.y
        self.pose2.position.z = self.pose1.position.z - 0.3 # !!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.pose2.orientation = self.pose1.orientation

        self.detect_pose = Pose()
        self.detect_pose.position.x = 0.15          # 请根据实际情况修改
        self.detect_pose.position.y = 0.0          # 请根据实际情况修改
        self.detect_pose.position.z = 0.55          # 请根据实际情况修改
        self.detect_pose.orientation.x = 0.0
        self.detect_pose.orientation.y = -0.707
        self.detect_pose.orientation.z = 0.0
        self.detect_pose.orientation.w = 0.707
        # =================================
        
        # 发布者
        self.movej_pub = rospy.Publisher('/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.movej_p_pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
        self.movel_pub = rospy.Publisher('/rm_driver/MoveL_Cmd', MoveL, queue_size=10)
        self.car_speed_pub = rospy.Publisher('/car_speed', Float32, queue_size=10)
        self.arm_state_pub = rospy.Publisher('/arm_state', Int32, queue_size=10)
        self.arm_state_timer = None  # 用于定时发布
        self.get_state_pub = rospy.Publisher('/rm_driver/GetArmState_Cmd', 
                                           GetArmState_Command, queue_size=10)

        # 订阅者
        rospy.Subscriber('/rm_driver/Plan_State', Plan_State, self.plan_state_callback)
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, 
                        self.arm_state_callback)
        
        # 状态变量
        self.plan_completed = False
        self.last_plan_result = False
        self.arm_state = 0  # 0-未工作, 1-工作中
        self.current_pose = None  # [x, y, z, rx, ry, rz]

        # 启动定时发布
        self.start_arm_state_publishing()
        
        rospy.loginfo("ArmController initialized")

    def arm_state_callback(self, msg):
        """机械臂状态回调 - 存储位置和姿态信息"""
        # msg.Pose 是 [x, y, z, rx, ry, rz] 格式
        self.current_pose = msg.Pose
    
    def get_current_position(self):
        """获取当前位置 [x, y, z]"""
        if self.current_pose is not None:
            return self.current_pose[0:3]  # 只返回位置
        return None
    
    def get_current_orientation(self):
        """获取当前姿态 [rx, ry, rz] (欧拉角)"""
        if self.current_pose is not None:
            return self.current_pose[3:6]  # 只返回姿态
        return None
    
    def is_at_home(self, position_tolerance=0.05, orientation_tolerance=0.1):
        """检查是否在home位置（检查位置和姿态）"""
        if self.current_pose is None:
            return False
        
        # current_pose 格式: [x, y, z, rx, ry, rz]
        current_pos = self.current_pose[0:3]  # 位置
        current_ori = self.current_pose[3:6]  # 姿态（欧拉角 rx, ry, rz）
        
        # 检查位置
        dx = abs(current_pos[0] - self.home_pose.position.x)
        dy = abs(current_pos[1] - self.home_pose.position.y)
        dz = abs(current_pos[2] - self.home_pose.position.z)
        
        position_match = (dx < position_tolerance and 
                         dy < position_tolerance and 
                         dz < position_tolerance)
        
        if not position_match:
            rospy.logdebug(f"Position not at home: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")
            return False
        
        # 将home_pose的四元数转换为欧拉角进行比较
        home_euler = self.quaternion_to_euler(
            self.home_pose.orientation.x,
            self.home_pose.orientation.y,
            self.home_pose.orientation.z,
            self.home_pose.orientation.w
        )
        
        # 检查姿态（欧拉角）
        drx = abs(current_ori[0] - home_euler[0])
        dry = abs(current_ori[1] - home_euler[1])
        drz = abs(current_ori[2] - home_euler[2])
        
        # 处理角度的周期性（例如 2π 和 0 是同一个角度）
        drx = min(drx, 2*np.pi - drx)
        dry = min(dry, 2*np.pi - dry)
        drz = min(drz, 2*np.pi - drz)
        
        orientation_match = (drx < orientation_tolerance and 
                            dry < orientation_tolerance and 
                            drz < orientation_tolerance)
        
        if not orientation_match:
            rospy.logdebug(f"Orientation not at home: drx={drx:.4f}, dry={dry:.4f}, drz={drz:.4f}")
        
        return position_match and orientation_match
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """四元数转欧拉角 (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def request_arm_state(self):
        """请求获取机械臂当前状态"""
        if self.get_state_pub.get_num_connections() > 0:
            command = GetArmState_Command()
            command.command = "get_current_arm_state"
            self.get_state_pub.publish(command)
    
    def wait_until_home(self, timeout=60.0, check_interval=0.5):
        """阻塞直到机械臂回到home位置"""
        rospy.loginfo("Waiting for arm to reach home position (checking position and orientation)...")
        
        start_time = rospy.Time.now()
        attempts = 0
        
        # 先确保我们有当前状态
        self.request_arm_state()
        rospy.sleep(0.2)  # 等待第一次回调
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            # 每5次检查请求一次状态（避免过于频繁）
            if attempts % 10 == 0:  # 每5秒请求一次（0.5*10=5秒）
                self.request_arm_state()
                rospy.sleep(0.1)  # 给回调一点时间
            
            # 检查是否到家
            if self.is_at_home():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                rospy.loginfo(f"✅ Arm confirmed at home position (after {elapsed:.1f}s, {attempts} checks)")
                return True
            
            attempts += 1
            rospy.sleep(check_interval)
        
        # 超时时的详细日志
        current_pos = self.get_current_position()
        if current_pos:
            rospy.logwarn(f"⚠️ Timeout! Current: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}], "
                        f"Home: [{self.home_pose.position.x:.3f}, {self.home_pose.position.y:.3f}, {self.home_pose.position.z:.3f}]")
        else:
            rospy.logwarn("⚠️ Timeout! No position data received")
        
        return False
    
    def start_arm_state_publishing(self):
        """以1Hz频率发布arm_state"""
        if self.arm_state_timer is not None:
            self.arm_state_timer.shutdown()
        
        def publish_state(event):
            msg = Int32()
            msg.data = self.arm_state
            self.arm_state_pub.publish(msg)
        
        self.arm_state_timer = rospy.Timer(rospy.Duration(1.0), publish_state)

    def set_arm_state(self, state):
        """设置机械臂状态"""
        self.arm_state = state
        rospy.loginfo(f"Arm state changed to: {state}")

    def plan_state_callback(self, msg):
        """规划状态回调"""
        self.plan_completed = True
        self.last_plan_result = msg.state
    
    def wait_for_completion(self, timeout=10.0):
        """等待规划完成"""
        start_time = rospy.Time.now()
        while not self.plan_completed and (rospy.Time.now() - start_time).to_sec() < timeout:
            rospy.sleep(0.1)
        
        result = self.last_plan_result
        self.plan_completed = False
        return result
    
    def go_home(self):
        """回到初始位置"""
        rospy.loginfo("Moving to home position...")
        success = self.move_to_pose_jp(self.home_pose, speed=0.3)
        return success

    def move_to_pose_jp(self, pose, speed=0.2):
        """通过MoveJ_P移动到位姿（关节空间规划）"""
        movej_p_cmd = MoveJ_P()
        movej_p_cmd.Pose = pose
        movej_p_cmd.speed = speed
        movej_p_cmd.trajectory_connect = 0
        
        self.plan_completed = False
        self.movej_p_pub.publish(movej_p_cmd)
        
        return self.wait_for_completion()
    
    def move_to_pose_line(self, pose, speed=0.2):
        """通过MoveL移动到位姿（直线运动）"""
        movel_cmd = MoveL()
        movel_cmd.Pose = pose
        movel_cmd.speed = speed
        movel_cmd.trajectory_connect = 0
        
        self.plan_completed = False
        self.movel_pub.publish(movel_cmd)
        
        return self.wait_for_completion()
    
    def move_joints(self, joints, speed=0.2):
        """通过MoveJ移动到关节角度"""
        movej_cmd = MoveJ()
        movej_cmd.joint = joints
        movej_cmd.speed = speed
        movej_cmd.trajectory_connect = 0
        
        self.plan_completed = False
        self.movej_pub.publish(movej_cmd)
        
        return self.wait_for_completion()
    
    def set_car_speed(self, speed, publish_times=1):
        """控制小车速度，可指定发布次数"""
        for i in range(publish_times):
            speed_msg = Float32()
            speed_msg.data = speed
            self.car_speed_pub.publish(speed_msg)
            if i < publish_times - 1:  # 如果不是最后一次发布，等待短暂时间
                rospy.sleep(0.01)  # 10ms间隔，避免消息堆积
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat
    
    def create_pose(self, x, y, z, orientation=None):
        """创建位姿"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        if orientation is None:
            pose.orientation = self.home_pose.orientation
        else:
            pose.orientation = orientation
        return pose