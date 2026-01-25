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
        self.safety_distance = 0.4  # 5cm安全距离
        self.approach_distance = 0.085  # 前进10cm
        self.approach_speed = 0.05     # 缓慢速度 
        self.current_count = 0

        self.x_offset = 0
        self.y_offset = 0.165
        self.z_offset = -0.02

        self.stable_frames_required = 10      # 需要连续稳定多少帧
        self.stable_threshold = 0.05       # XYZ 最大变化阈值（米）
        self.center_history = []             # 存储连续帧的 center_3d_robot（list of [x,y,z]）
        
        rospy.loginfo("PointInspectionTask initialized")

    def execute_task(self):
        """執行16點檢測任務"""
        try:
            # 停止小車
            self.arm.set_car_speed(0.0)
            rospy.loginfo("Car stopped")
         
            # 移動到观测位置
            if not self.arm.move_to_pose_jp(self.arm.detect_pose, speed=0.2):
                rospy.logerr("Failed to move to detect pose")
                return
         
            rospy.loginfo("Detecting target for up to 10 seconds...")
            start_time = rospy.Time.now()
            target_center = None
            self.center_history = []  # 清空历史记录
            
            while (rospy.Time.now() - start_time) < rospy.Duration(10.0) and target_center is None:
                current_target = self.vision.detect_target()
                if current_target is None:
                    rospy.loginfo("Target not detected yet, retrying...")
                    rospy.sleep(0.2)  # 缩短间隔，提高检测频率
                    continue
                
                center_robot = current_target['center_3d_robot']
                if center_robot is None:
                    rospy.loginfo("No valid 3D robot center, retrying...")
                    rospy.sleep(0.2)
                    continue
                
                # 添加当前帧到历史
                self.center_history.append(center_robot)
                
                # 如果收集到足够帧数，开始检查稳定性
                if len(self.center_history) >= self.stable_frames_required:
                    is_stable = True
                    max_diff = 0.0
                    
                    # 检查最后 N 帧中相邻帧的最大变化
                    for i in range(1, self.stable_frames_required):
                        prev = np.array(self.center_history[-i-1])
                        curr = np.array(self.center_history[-i])
                        diff = np.linalg.norm(prev - curr)  # 欧氏距离
                        max_diff = max(max_diff, diff)
                        if diff > self.stable_threshold:
                            is_stable = False
                            break
                    
                    rospy.loginfo(f"当前稳定检查：最大变化 = {max_diff:.4f} m")
                    
                    if is_stable:
                        # 稳定！取最后 N 帧平均值作为最终中心
                        stable_center = np.mean(self.center_history[-self.stable_frames_required:], axis=0)
                        target_center = current_target.copy()
                        target_center['center_3d_robot'] = stable_center.tolist()
                        
                        rospy.loginfo(f"目标稳定！连续 {self.stable_frames_required} 帧，最大变化 {max_diff:.4f}m < {self.stable_threshold}m")
                        rospy.loginfo(f"稳定中心点: {stable_center}")
                        break
                    else:
                        rospy.loginfo(f"目标不稳定，继续检测...")
                
                rospy.sleep(0.2)  # 每帧间隔
            
            if target_center is None:
                rospy.logwarn("Target not detected or not stable after 10 seconds, task failed")
                rospy.loginfo("Returning to home position due to detection failure")
                self.arm.go_home()
               
                self.arm.set_car_speed(0.8)
                rospy.loginfo("Failed detection → car speed restored to 0.8")
               
                self.is_active = False
                self._publish_task_done("failed")
                return
            
            # 应用偏移（在稳定中心上）
            center_robot = target_center['center_3d_robot']
            center_robot[0] += self.x_offset  # x
            center_robot[1] += self.y_offset  # y
            center_robot[2] += self.z_offset  # z
            
            rospy.loginfo(f"Target center detected at (after offset): {target_center}")
         
            # 计算16个点的坐标
            points = self.vision.calculate_16_points(
                center_robot,
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
         
            # 成功结束也要通知
            self._publish_task_done("completed")
         
        except Exception as e:
            rospy.logerr(f"Error in 16-point inspection task: {e}")
            self._publish_task_done("error")
       
        finally:
            rospy.loginfo("Point task cleanup: ensuring return to home and restore car speed")
         
            self.arm.go_home()
         
            if self.arm.wait_until_home(timeout=60.0):
                rospy.loginfo("Arm reached home during cleanup")
            else:
                rospy.logwarn("Home timeout in cleanup, proceeding anyway")
         
            self.arm.set_car_speed(0.8)
            rospy.loginfo("Point task cleanup: car speed restored to 0.8")
         
            self.is_active = False
            rospy.loginfo("Point inspection task thread completed - is_active=False")
           
            self._publish_task_done("finally")
    def _publish_task_done(self, status="done"):
        from std_msgs.msg import String
        try:
            pub = rospy.Publisher('/point_task_done', String, queue_size=1, latch=True)
            msg = String()
            msg.data = f"point_task_{status}_{self.current_count}"
            pub.publish(msg)
            rospy.loginfo(f"Published point task done signal: {msg.data}")
        except Exception as e:
            rospy.logwarn(f"Failed to publish point task done: {e}")

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
            
            if not self.arm.move_to_pose_line(approach_pose, speed=self.approach_speed):
                rospy.logwarn(f"Failed to approach point {i+1}")

            
            # 退回到原位
            rospy.loginfo(f"Retracting from point {i+1}")
            if not self.arm.move_to_pose_line(target_pose, speed=self.approach_speed):
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