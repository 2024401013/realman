#!/usr/bin/env python

# src/rm_vision_control/scripts/crack_detection.py
import rospy
import time
from std_msgs.msg import Int32

class CrackDetectionTask:
    def __init__(self, arm_controller, vision_detector):
        self.arm = arm_controller
        self.vision = vision_detector
        
        # 任务状态
        self.is_active = False
        self.last_detection_time = 0
        self.detection_interval = 5.0  # 4秒间隔
        
        rospy.loginfo("CrackDetectionTask initialized")
    
    
    def execute_task(self):
        """执行裂缝检测任务"""
        if self.is_active:  # 防止重复执行
            return

        self.is_active = True
        try:
            # 移动到观测姿态pose1
            rospy.loginfo("Moving to pose1")
            if not self.arm.move_to_pose_jp(self.arm.pose1, speed=0.5):
                rospy.logerr("Failed to move to pose1")
                return
            
            # 发布小车速度
            self.arm.set_car_speed(0.3)
            rospy.loginfo("Car speed set to 0.3")
            
            # 开始往返运动
            self.execute_reciprocating_motion()
            
            # 任务完成，回到初始位置
            rospy.loginfo("Task completed, returning home")
            self.arm.go_home()
            
        except Exception as e:
            rospy.logerr(f"Error in crack detection task: {e}")
        finally:
            self.is_active = False
    
    def execute_reciprocating_motion(self):
        """在pose1和pose2之间直线往返运动"""
        move_to_pose1 = True
        
        while self.is_active and not rospy.is_shutdown():
            # 移动到目标位姿
            if move_to_pose1:
                rospy.loginfo("Moving to pose1")
                target_pose = self.arm.pose1
            else:
                rospy.loginfo("Moving to pose2")
                target_pose = self.arm.pose2
            
            if not self.arm.move_to_pose_jp(target_pose, speed=0.2):
                rospy.logerr("Failed to move to target pose")
                break
            
            # 执行检测
            self.perform_detection()
            
            # 切换目标位姿
            move_to_pose1 = not move_to_pose1
            
            # 检查是否继续
            if not self.is_active:
                break
            
            rospy.sleep(0.1)
    
    def perform_detection(self):
        """执行裂缝检测"""
        detection_result = self.vision.detect_cracks()
        
        if detection_result['detected']:
            current_time = time.time()
            
            # 检查时间间隔
            if current_time - self.last_detection_time > self.detection_interval:
                rospy.loginfo(f"Crack detected! Confidence: {detection_result['confidence']:.2f}")
                
                # 保存检测结果
                self.vision.save_detection_result(detection_result)
                
                # 更新上次检测时间
                self.last_detection_time = current_time
            else:
                rospy.loginfo(f"Crack detected but too frequent, waiting for interval")