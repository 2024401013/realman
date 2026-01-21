#!/usr/bin/env python3
import rospy
import numpy as np
import json
import yaml
import os
from datetime import datetime
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Utils:
    @staticmethod
    def load_config(config_path):
        """加载YAML配置文件"""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            rospy.loginfo(f"Loaded config from {config_path}")
            return config
        except Exception as e:
            rospy.logwarn(f"Failed to load config from {config_path}: {e}")
            return None
    
    @staticmethod
    def save_config(config, config_path):
        """保存配置到YAML文件"""
        try:
            with open(config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            rospy.loginfo(f"Saved config to {config_path}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to save config to {config_path}: {e}")
            return False
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """欧拉角转四元数"""
        q = quaternion_from_euler(roll, pitch, yaw)
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        return quat
    
    @staticmethod
    def quaternion_to_euler(quaternion):
        """四元数转欧拉角"""
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return euler_from_quaternion(q)
    
    @staticmethod
    def create_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """创建位姿"""
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = Utils.euler_to_quaternion(roll, pitch, yaw)
        return pose
    
    @staticmethod
    def pose_to_string(pose):
        """将位姿转换为字符串表示"""
        pos = pose.position
        orientation = pose.orientation
        return f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), " \
               f"Orientation: ({orientation.x:.3f}, {orientation.y:.3f}, " \
               f"{orientation.z:.3f}, {orientation.w:.3f})"
    
    @staticmethod
    def create_timestamp_filename(prefix, extension):
        """创建带时间戳的文件名"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{prefix}_{timestamp}.{extension}"
    
    @staticmethod
    def save_detection_result(data, save_dir="detection_results"):
        """保存检测结果"""
        # 创建目录
        os.makedirs(save_dir, exist_ok=True)
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        # 保存JSON数据
        json_path = os.path.join(save_dir, f"detection_{timestamp}.json")
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        
        rospy.loginfo(f"Detection result saved to {json_path}")
        return json_path
    
    @staticmethod
    def check_pose_reachable(pose, joint_limits):
        """
        检查位姿是否可达
        joint_limits: [(min1, max1), (min2, max2), ...]
        """
        # 这里可以添加逆运动学检查
        # 实际使用中需要调用逆运动学求解
        rospy.logwarn("IK check not implemented, assuming pose is reachable")
        return True
    
    @staticmethod
    def normalize_quaternion(quaternion):
        """归一化四元数"""
        norm = np.sqrt(
            quaternion.x**2 + 
            quaternion.y**2 + 
            quaternion.z**2 + 
            quaternion.w**2
        )
        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm
        quaternion.w /= norm
        return quaternion
    
    @staticmethod
    def interpolate_poses(pose1, pose2, alpha):
        """在两个位姿之间插值"""
        result = Pose()
        
        # 位置线性插值
        result.position.x = pose1.position.x * (1 - alpha) + pose2.position.x * alpha
        result.position.y = pose1.position.y * (1 - alpha) + pose2.position.y * alpha
        result.position.z = pose1.position.z * (1 - alpha) + pose2.position.z * alpha
        
        # 四元数球面线性插值
        q1 = np.array([pose1.orientation.x, pose1.orientation.y, 
                       pose1.orientation.z, pose1.orientation.w])
        q2 = np.array([pose2.orientation.x, pose2.orientation.y, 
                       pose2.orientation.z, pose2.orientation.w])
        
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            result_quat = q1 * (1 - alpha) + q2 * alpha
        else:
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * alpha
            sin_theta = np.sin(theta)
            s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0
            result_quat = s0 * q1 + s1 * q2
        
        result_quat = result_quat / np.linalg.norm(result_quat)
        result.orientation.x = result_quat[0]
        result.orientation.y = result_quat[1]
        result.orientation.z = result_quat[2]
        result.orientation.w = result_quat[3]
        
        return result
    
    @staticmethod
    def calculate_distance(pose1, pose2):
        """计算两个位姿之间的距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)