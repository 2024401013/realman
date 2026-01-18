#!/usr/bin/env python3
# set_arm_pose.py - 使用多种方式设定位姿
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from rm_msgs.msg import MoveJ, MoveJ_P, MoveL, Plan_State
from rm_msgs.msg import Arm_Current_State, JointPos

class ArmPoseSetter:
    def __init__(self):
        rospy.init_node('arm_pose_setter')
        
        # 发布者
        self.movej_pub = rospy.Publisher('/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.movej_p_pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
        self.movel_pub = rospy.Publisher('/rm_driver/MoveL_Cmd', MoveL, queue_size=10)
        
        # 状态订阅者 - 用于查询当前状态
        self.current_pose = None
        self.current_joints = None
        self.current_state = None
        
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.state_callback)
        rospy.Subscriber('/rm_driver/JointPos', JointPos, self.joint_callback)
        
        # 等待状态更新
        rospy.sleep(1.0)
        
        rospy.loginfo("Arm Pose Setter initialized")
    
    def state_callback(self, msg):
        """机械臂状态回调"""
        self.current_state = msg
        self.current_joints = list(msg.joint)  # 转换为列表
    
    def joint_callback(self, msg):
        """关节角度回调"""
        self.current_joints = list(msg.joint)
    
    def get_current_state(self):
        """获取当前状态"""
        attempts = 0
        while (self.current_joints is None or self.current_state is None) and attempts < 10:
            rospy.loginfo("Waiting for arm state...")
            rospy.sleep(0.5)
            attempts += 1
        
        if self.current_joints is not None:
            rospy.loginfo(f"Current joints: {[math.degrees(j) for j in self.current_joints]}°")
        
        if self.current_state is not None and hasattr(self.current_state, 'Pose'):
            pose = self.current_state.Pose
            rospy.loginfo(f"Current pose - Position: ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
            rospy.loginfo(f"Current pose - Orientation(RPY): ({pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f})°")
    
    def create_pose_from_euler(self, x, y, z, roll, pitch, yaw):
        """
        使用欧拉角创建位姿
        roll, pitch, yaw: 弧度制
        """
        # 将欧拉角转换为四元数
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        return pose
    
    def create_pose_from_degrees(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        """
        使用角度制欧拉角创建位姿
        """
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        
        return self.create_pose_from_euler(x, y, z, roll, pitch, yaw)
    
    def move_to_pose_jp(self, pose, speed=0.3):
        """使用MoveJ_P移动到位姿"""
        rospy.loginfo(f"Moving to pose: Position({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        # 获取欧拉角用于显示
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = euler_from_quaternion(quat)
        rospy.loginfo(f"Orientation (RPY): ({math.degrees(euler[0]):.1f}, {math.degrees(euler[1]):.1f}, {math.degrees(euler[2]):.1f})°")
        
        movej_p_cmd = MoveJ_P()
        movej_p_cmd.Pose = pose
        movej_p_cmd.speed = speed
        movej_p_cmd.trajectory_connect = 0
        
        self.movej_p_pub.publish(movej_p_cmd)
        rospy.sleep(2.0)  # 等待执行
        
        return True
    
    def move_to_pose_line(self, pose, speed=0.3):
        """使用MoveL直线移动到位姿"""
        rospy.loginfo(f"Moving linearly to pose: Position({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        movel_cmd = MoveL()
        movel_cmd.Pose = pose
        movel_cmd.speed = speed
        movel_cmd.trajectory_connect = 0
        
        self.movel_pub.publish(movel_cmd)
        rospy.sleep(2.0)
        
        return True
    
    def move_joints(self, joints_deg, speed=0.3):
        """使用MoveJ移动到关节角度（输入角度制）"""
        # 转换为弧度
        joints_rad = [math.radians(angle) for angle in joints_deg]
        
        rospy.loginfo(f"Moving joints to: {joints_deg}°")
        
        movej_cmd = MoveJ()
        movej_cmd.joint = joints_rad
        movej_cmd.speed = speed
        movej_cmd.trajectory_connect = 0
        
        self.movej_pub.publish(movej_cmd)
        rospy.sleep(2.0)
        
        return True
    
    def go_home(self):
        """回到初始位置"""
        rospy.loginfo("Going to home position...")
        
        # 定义一个安全的初始位置（角度制）
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零点位置
        
        return self.move_joints(home_joints, speed=0.2)
    
    def interactive_set_pose(self):
        """交互式设定位姿"""
        while not rospy.is_shutdown():
            print("\n" + "="*60)
            print("ARM POSE SETTER - Interactive Mode")
            print("="*60)
            print("1. Set pose using Euler angles (degrees)")
            print("2. Set pose using position only (keep current orientation)")
            print("3. Set joint angles directly")
            print("4. Move to predefined test poses")
            print("5. Show current state")
            print("6. Go home")
            print("0. Exit")
            print("-"*60)
            
            choice = input("Select option (0-6): ")
            
            if choice == '1':
                self.set_pose_euler_interactive()
            elif choice == '2':
                self.set_pose_position_interactive()
            elif choice == '3':
                self.set_joints_interactive()
            elif choice == '4':
                self.predefined_poses()
            elif choice == '5':
                self.get_current_state()
            elif choice == '6':
                self.go_home()
            elif choice == '0':
                rospy.loginfo("Exiting...")
                break
            else:
                print("Invalid choice!")
    
    def set_pose_euler_interactive(self):
        """交互式使用欧拉角设定位姿"""
        print("\n--- Set Pose using Euler Angles (degrees) ---")
        
        try:
            x = float(input("Enter X position (meters): "))
            y = float(input("Enter Y position (meters): "))
            z = float(input("Enter Z position (meters): "))
            roll = float(input("Enter Roll (degrees): "))
            pitch = float(input("Enter Pitch (degrees): "))
            yaw = float(input("Enter Yaw (degrees): "))
            speed = float(input("Enter speed (0.1-1.0, default 0.3): ") or "0.3")
            
            # 创建位姿
            pose = self.create_pose_from_degrees(x, y, z, roll, pitch, yaw)
            
            # 移动方式选择
            print("\nMovement type:")
            print("1. MoveJ_P (关节空间规划)")
            print("2. MoveL (直线运动)")
            move_type = input("Select (1 or 2): ")
            
            if move_type == '1':
                self.move_to_pose_jp(pose, speed)
            elif move_type == '2':
                self.move_to_pose_line(pose, speed)
            else:
                print("Invalid choice, using MoveJ_P")
                self.move_to_pose_jp(pose, speed)
            
        except ValueError as e:
            print(f"Error: {e}. Please enter valid numbers.")
    
    def set_pose_position_interactive(self):
        """交互式仅设置位置（保持当前姿态）"""
        print("\n--- Set Position Only (Keep Current Orientation) ---")
        
        # 首先获取当前姿态
        if self.current_state is None or not hasattr(self.current_state, 'Pose'):
            print("Cannot get current pose. Please try option 5 first.")
            return
        
        try:
            current_pose_data = self.current_state.Pose
            current_orientation = Quaternion()
            # 注意：Arm_Current_State中的Pose是float32[6]，不是标准的Pose消息
            # 这里我们需要从其他话题获取姿态，或者使用预设姿态
            
            print("Note: Using predefined orientation (horizontal)")
            print("For precise orientation control, use option 1")
            
            x = float(input("Enter X position (meters): "))
            y = float(input("Enter Y position (meters): "))
            z = float(input("Enter Z position (meters): "))
            speed = float(input("Enter speed (0.1-1.0, default 0.3): ") or "0.3")
            
            # 使用水平姿态
            pose = self.create_pose_from_degrees(x, y, z, 180, 0, 0)  # 水平向下
            
            self.move_to_pose_jp(pose, speed)
            
        except ValueError as e:
            print(f"Error: {e}")
    
    def set_joints_interactive(self):
        """交互式设置关节角度"""
        print("\n--- Set Joint Angles (degrees) ---")
        
        try:
            joints = []
            for i in range(6):
                angle = float(input(f"Enter Joint {i+1} angle (degrees): "))
                joints.append(angle)
            
            speed = float(input("Enter speed (0.1-1.0, default 0.3): ") or "0.3")
            
            self.move_joints(joints, speed)
            
        except ValueError as e:
            print(f"Error: {e}")
    
    def predefined_poses(self):
        """预定义测试位姿"""
        print("\n--- Predefined Test Poses ---")
        print("1. Observation Pose 1 (水平姿态)")
        print("2. Observation Pose 2 (Z+0.5m)")
        print("3. Left side")
        print("4. Right side")
        print("5. Up high")
        print("6. Down low")
        
        choice = input("Select pose (1-6): ")
        speed = float(input("Enter speed (0.1-1.0, default 0.2): ") or "0.2")
        
        # 水平姿态（向下）
        orientation = self.create_pose_from_degrees(0, 0, 0, 180, 0, 0)
        
        poses = {
            '1': self.create_pose_from_degrees(0.5, 0.0, 0.3, 180, 0, 0),
            '2': self.create_pose_from_degrees(0.5, 0.0, 0.8, 180, 0, 0),
            '3': self.create_pose_from_degrees(0.3, 0.3, 0.4, 180, 0, 0),
            '4': self.create_pose_from_degrees(0.3, -0.3, 0.4, 180, 0, 0),
            '5': self.create_pose_from_degrees(0.3, 0.0, 0.6, 180, 0, 0),
            '6': self.create_pose_from_degrees(0.3, 0.0, 0.2, 180, 0, 0),
        }
        
        if choice in poses:
            pose = poses[choice]
            print(f"Moving to Pose {choice}...")
            self.move_to_pose_jp(pose, speed)
        else:
            print("Invalid choice!")

def main():
    """主函数"""
    try:
        # 初始化
        pose_setter = ArmPoseSetter()
        
        # 显示当前状态
        pose_setter.get_current_state()
        
        # 启动交互模式
        pose_setter.interactive_set_pose()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main()