#!/usr/bin/env python

# src/rm_vision_control/scripts/point_inspection.py
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
        
        # 配置参数
        self.grid_size = 0.05  # 5cm网格间距
        self.rows = 4
        self.cols = 4
        self.safety_distance = 0.45  # 5cm安全距离
        self.approach_distance = 0.1  # 前进10cm
        self.approach_speed = 0.05     # 缓慢速度 
        self.current_count = 0
        
        rospy.loginfo("PointInspectionTask initialized")
    
    def execute_task(self):
        """执行16点检测任务"""
        try:
            # 停止小车
            self.arm.set_car_speed(0.0)
            rospy.loginfo("Car stopped")
           
            # 移动到观测位置
            if not self.arm.move_to_pose_jp(self.arm.detect_pose, speed=0.2):
                rospy.logerr("Failed to move to detect pose")
                return
           
            rospy.loginfo("Detecting target for up to 10 seconds...")
            start_time = rospy.Time.now()
            target_center = None
            while (rospy.Time.now() - start_time) < rospy.Duration(10.0) and target_center is None:
                target_center = self.vision.detect_target()
                if target_center is None:
                    rospy.loginfo("Target not detected yet, retrying...")
                    rospy.sleep(0.5)
           
            if target_center is None:
                rospy.logwarn("Target not detected after 10 seconds, task failed")
                rospy.loginfo("Returning to home position due to detection failure")
                self.arm.go_home()
                
                # 失败也要恢复状态
                if self.current_count == 2:
                    self.arm.set_arm_state(0)
                    rospy.loginfo("Failed second point → arm_state → 0")
                rospy.sleep(0.5)
                self.arm.set_car_speed(0.8)
                rospy.loginfo("Failed detection → car speed restored to 0.8")
                
                self.is_active = False
                return   
           
            rospy.loginfo(f"Target center detected at: {target_center}")
           
            # 计算16个点的坐标
            points = self.vision.calculate_16_points(
                target_center['center_3d_robot'],
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
           
            rospy.loginfo("16-point inspection task completed successfully")
           
        except Exception as e:
            rospy.logerr(f"Error in 16-point inspection task: {e}")
        
        finally:
            rospy.loginfo("Point task cleanup: ensuring return to home and restore car speed")
           
            self.arm.go_home()
           
            # 等待回家
            if self.arm.wait_until_home(timeout=60.0):
                rospy.loginfo("Arm reached home during cleanup")
            else:
                rospy.logwarn("Home timeout in cleanup, proceeding anyway")
           
            if self.current_count == 2:
                self.arm.set_arm_state(0)
                rospy.loginfo("Second point task cleanup: arm_state set to 0 (first)")
            else:
                rospy.loginfo("Point task cleanup: keeping arm_state=1 (first run)")
            rospy.sleep(0.5)
            self.arm.set_car_speed(0.8)
            rospy.loginfo("Point task cleanup: car speed restored to 0.8 (after arm_state)")
           
            self.is_active = False
            rospy.loginfo("Point inspection task thread completed - is_active=False")

    def execute_point_inspection(self, points):
        """执行点检测"""
        if len(points) != self.rows * self.cols:
            rospy.logerr(f"Expected {self.rows * self.cols} points, got {len(points)}")
            return
        
        # 顺序访问所有点
        for i in range(len(points)):
            point_idx = i  # 从左到右，从上到下
            
            rospy.loginfo(f"Inspecting point {i+1}/{len(points)}: {points[point_idx]}")
            
            # 创建目标位姿
            target_pose = self.create_pose_from_point(points[point_idx])
            
            if not self.arm.move_to_pose_jp(target_pose, speed=0.2):
                rospy.logwarn(f"Failed to move to point {i+1}, skipping")
                continue
            
            # 垂直向墙面前进10cm
            rospy.loginfo(f"Approaching point {i+1} (moving forward {self.approach_distance}m)")
            approach_pose = self.create_pose_from_point(points[point_idx])
            approach_pose.position.x -= self.approach_distance  # 假设z轴指向墙面
            
            if not self.arm.move_to_pose_jp(approach_pose, speed=self.approach_speed):
                rospy.logwarn(f"Failed to approach point {i+1}")
            
            # 在这里可以添加实际的检测操作
            # self.perform_point_test()
            
            # 退回到原位
            rospy.loginfo(f"Retracting from point {i+1}")
            if not self.arm.move_to_pose_jp(target_pose, speed=self.approach_speed):
                rospy.logwarn(f"Failed to retract from point {i+1}")
            
            # 短暂停顿
            rospy.sleep(0.5)
    
    def create_pose_from_point(self, point):
        """从3D点创建位姿"""
        pose = Pose()
        pose.position.y = point[1]
        pose.position.z = point[2]
        
        # 关键：减去安全距离
        pose.position.x = point[0] + self.safety_distance
        
        # 保持水平姿态
        pose.orientation = self.arm.pose1.orientation
        
        return pose