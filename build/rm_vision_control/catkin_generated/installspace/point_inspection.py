#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Quaternion

class PointInspectionTask:
    def __init__(self, arm_controller, vision_detector):
        self.arm = arm_controller
        self.vision = vision_detector
        
        # 任务状态
        self.is_active = False
        
        # 订阅小车指令
        rospy.Subscriber('/car/is_arrive', Int32, self.car_command_callback)
        
        # 配置参数
        self.grid_size = 0.05  # 5cm网格间距
        self.rows = 4
        self.cols = 4
        self.approach_distance = 0.1  # 前进10cm
        self.approach_speed = 0.1     # 缓慢速度
        
        rospy.loginfo("PointInspectionTask initialized")
    
    def car_command_callback(self, msg):
        """处理小车指令"""
        if msg.data == 3 and not self.is_active:
            # 开始16点检测任务
            rospy.loginfo("Starting 16-point inspection task")
            self.is_active = True
            
            # 在新线程中执行任务，避免阻塞回调
            import threading
            task_thread = threading.Thread(target=self.execute_task)
            task_thread.start()
    
    def execute_task(self):
        """执行16点检测任务"""
        try:
            # 停止小车
            self.arm.set_car_speed(0.0)
            rospy.loginfo("Car stopped")
            
            # 识别目标并计算16个点
            rospy.loginfo("Detecting target...")
            target_center = self.vision.detect_target()
            
            if target_center is None:
                rospy.logwarn("Target not detected, task failed")
                self.is_active = False
                return
            
            rospy.loginfo(f"Target center detected at: {target_center}")
            
            # 计算16个点的坐标
            points = self.vision.calculate_16_points(
                target_center, 
                grid_size=self.grid_size,
                rows=self.rows,
                cols=self.cols
            )
            
            rospy.loginfo(f"Calculated {len(points)} points")
            
            # 执行16点检测
            self.execute_point_inspection(points)
            
            # 回到初始位置
            rospy.loginfo("Returning to home position")
            self.arm.go_home()
            
            # 恢复小车速度
            self.arm.set_car_speed(0.8)
            rospy.loginfo("Car speed set to 0.8")
            
            rospy.loginfo("16-point inspection task completed successfully")
            
        except Exception as e:
            rospy.logerr(f"Error in 16-point inspection task: {e}")
        finally:
            self.is_active = False
    
    def execute_point_inspection(self, points):
        """执行点检测"""
        if len(points) != self.rows * self.cols:
            rospy.logerr(f"Expected {self.rows * self.cols} points, got {len(points)}")
            return
        
        # 移动到左上角第一个点（第0行第0列）
        first_point = points[0]
        rospy.loginfo(f"Moving to first point: {first_point}")
        
        first_pose = self.create_pose_from_point(first_point)
        if not self.arm.move_to_pose_jp(first_pose, speed=0.3):
            rospy.logerr("Failed to move to first point")
            return
        
        # 顺序访问所有点
        for i in range(len(points)):
            point_idx = i  # 从左到右，从上到下
            
            rospy.loginfo(f"Inspecting point {i+1}/{len(points)}: {points[point_idx]}")
            
            # 创建目标位姿
            target_pose = self.create_pose_from_point(points[point_idx])
            
            # 使用直线运动精确到达该点
            if not self.arm.move_to_pose_line(target_pose, speed=0.2):
                rospy.logwarn(f"Failed to move to point {i+1}, skipping")
                continue
            
            # 垂直向墙面前进10cm
            rospy.loginfo(f"Approaching point {i+1} (moving forward {self.approach_distance}m)")
            approach_pose = self.create_pose_from_point(points[point_idx])
            approach_pose.position.z -= self.approach_distance  # 假设z轴指向墙面
            
            if not self.arm.move_to_pose_line(approach_pose, speed=self.approach_speed):
                rospy.logwarn(f"Failed to approach point {i+1}")
            
            # 在这里可以添加实际的检测操作
            # self.perform_point_test()
            
            # 退回到原位
            rospy.loginfo(f"Retracting from point {i+1}")
            if not self.arm.move_to_pose_line(target_pose, speed=self.approach_speed):
                rospy.logwarn(f"Failed to retract from point {i+1}")
            
            # 检查是否继续
            if not self.is_active:
                rospy.loginfo("Task interrupted")
                break
            
            # 短暂停顿
            rospy.sleep(0.5)
    
    def create_pose_from_point(self, point):
        """从3D点创建位姿"""
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        
        # 保持水平姿态（与初始姿态相同）
        pose.orientation = self.arm.home_pose.orientation
        
        return pose