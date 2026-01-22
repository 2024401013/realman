#!/usr/bin/env python

# src/rm_vision_control/scripts/main_controller.py
import rospy
import signal
import sys
import threading  
from std_msgs.msg import Int32  
from rm_vision_control.arm_controller import ArmController
from rm_vision_control.vision_detector import VisionDetector
from rm_vision_control.crack_detection import CrackDetectionTask
from rm_vision_control.point_inspection import PointInspectionTask

class MainController:
    def __init__(self):
        rospy.init_node('main_controller', anonymous=True)
        
        # 初始化模块
        rospy.loginfo("Initializing modules...")
        self.task_lock = threading.Lock()
        
        self.arm_controller = ArmController()
        self.vision_detector = VisionDetector()
        
        # 添加小车指令订阅
        rospy.Subscriber('/is_arrive', Int32, self.car_command_callback)
        
        # 初始化任务
        self.crack_task = CrackDetectionTask(self.arm_controller, self.vision_detector)
        self.point_task = PointInspectionTask(self.arm_controller, self.vision_detector)
        
        # 修改状态变量
        self.active_task = None  # 'crack' 或 'point'
        self.point_task_count = 0  # 记录16点检测执行次数（0, 1, 2）
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("Main Controller initialized successfully")
        rospy.loginfo("Waiting for car commands...")
        rospy.loginfo("  is_arrive=1: Start crack detection")
        rospy.loginfo("  is_arrive=2: Stop crack detection OR start second point inspection")
        rospy.loginfo("  is_arrive=3: Starsst first 16-point inspection")

    def car_command_callback(self, msg):
        """处理小车指令"""
        with self.task_lock:
            # ========== 首先检查是否有已完成的任务需要清理 ==========
            if self.active_task == 'point' and not self.point_task.is_active:
                # Point任务已经完成（is_active=False表示任务线程已结束）
                rospy.loginfo("Point inspection task completed, cleaning up...")
                rospy.loginfo(f"DEBUG: point_task_count={self.point_task_count}")
                
                # 堵塞等待确认到达home
                if self.arm_controller.wait_until_home(timeout=15.0):
                    rospy.loginfo("✅ Point task: Arm confirmed at home")
                else:
                    rospy.logwarn("⚠️ Point task: Arm may not be fully at home")
                
                # 恢复小车速度
                self.arm_controller.set_car_speed(0.8)
                rospy.loginfo("Car speed restored to 0.8")
                
                # 根据执行次数决定arm_state
                if self.point_task_count == 1:
                    # 第一次16点检测完成，保持arm_state=1
                    self.arm_controller.set_arm_state(1)
                    rospy.loginfo("First point task completed, arm_state=1 (kept for second task)")
                    # ⚠️ point_task_count 保持为1，不需要操作
                elif self.point_task_count == 2:
                    # 第二次16点检测完成，设置arm_state=0
                    self.arm_controller.set_arm_state(0)
                    rospy.loginfo("Second point task completed, arm_state=0 (all done)")
                    self.point_task_count = 0  # 重置计数
                else:
                    rospy.logwarn(f"Unexpected point_task_count: {self.point_task_count}")
                
                # 清空活动任务
                self.active_task = None
                rospy.loginfo("Active task cleared")
            
            elif self.active_task == 'crack' and not self.crack_task.is_active:
                # Crack任务已经完成
                rospy.loginfo("Crack detection task completed, cleaning up...")
                
                # 堵塞等待确认到达home
                if self.arm_controller.wait_until_home(timeout=10.0):
                    rospy.loginfo("✅ Crack task: Arm confirmed at home")
                else:
                    rospy.logwarn("⚠️ Crack task: Arm may not be fully at home")
                
                # 恢复小车速度
                self.arm_controller.set_car_speed(0.8)
                rospy.loginfo("Car speed restored to 0.8")
                
                # 机械臂复位
                self.arm_controller.set_arm_state(0)
                rospy.loginfo("Arm state set to 0")
                
                # 清空活动任务
                self.active_task = None
                rospy.loginfo("Active task cleared")
            
            # ========== 处理新命令 ==========
            if msg.data == 1:
                # 命令1：启动裂缝检测任务
                if self.active_task is None:
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("Received command 1: Start crack detection")
                    self.active_task = 'crack'
                    
                    # 设置状态
                    self.arm_controller.set_arm_state(1)
                    rospy.loginfo("Arm state set to 1")
                    self.arm_controller.set_car_speed(0.3)
                    rospy.loginfo("Car speed set to 0.3")
                    
                    # 在新线程中执行任务
                    task_thread = threading.Thread(target=self.crack_task.execute_task)
                    task_thread.daemon = True
                    task_thread.start()
                    rospy.loginfo("Crack detection task thread started")
                else:
                    rospy.logwarn(f"Cannot start crack task: active_task={self.active_task}")
            
            elif msg.data == 2:
                # 命令2：停止裂缝检测 OR 启动第二次16点检测
                if self.active_task == 'crack':
                    # 停止裂缝检测任务
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("Received command 2: Stop crack detection")
                    
                    # 发送停止信号
                    self.crack_task.is_active = False
                    rospy.loginfo("Sent stop signal to crack task")
                    
                    # 等待任务线程开始回家流程
                    rospy.sleep(0.5)
                    
                    # 堵塞等待机械臂回到home
                    rospy.loginfo("Waiting for arm to physically return home...")
                    if self.arm_controller.wait_until_home(timeout=20.0):
                        rospy.loginfo("✅ Arm confirmed at home position")
                    else:
                        rospy.logwarn("⚠️ Arm may not be fully at home, but proceeding")
                    
                    # 发布状态
                    self.arm_controller.set_arm_state(0)
                    rospy.loginfo("Arm state set to 0")
                    self.arm_controller.set_car_speed(0.8)
                    rospy.loginfo("Car speed restored to 0.8")
                    
                    # 清空活动任务
                    self.active_task = None
                    rospy.loginfo("Crack task stopped and cleared")
                
                elif self.active_task is None and self.point_task_count == 1:
                    # 启动第二次16点检测
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("Received command 2: Start SECOND point inspection")
                    self.active_task = 'point'
                    self.point_task_count = 2
                    
                    # 设置状态
                    self.arm_controller.set_arm_state(1)
                    rospy.loginfo("Arm state set to 1")
                    self.arm_controller.set_car_speed(0.0)
                    rospy.loginfo("Car speed set to 0.0")
                    
                    # 重置并启动任务
                    self.point_task.is_active = True
                    task_thread = threading.Thread(target=self.point_task.execute_task)
                    task_thread.daemon = True
                    task_thread.start()
                    rospy.loginfo("Second point inspection task thread started")
                else:
                    rospy.logwarn(f"Cannot process command 2: active_task={self.active_task}, point_count={self.point_task_count}")
            
            elif msg.data == 3:
                # 命令3：启动第一次16点检测
                if self.active_task is None:
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("Received command 3: Start FIRST point inspection")
                    self.active_task = 'point'
                    self.point_task_count = 1
                    
                    # 设置状态
                    self.arm_controller.set_arm_state(1)
                    rospy.loginfo("Arm state set to 1")
                    self.arm_controller.set_car_speed(0.0)
                    rospy.loginfo("Car speed set to 0.0")
                    
                    # 启动任务
                    self.point_task.is_active = True
                    task_thread = threading.Thread(target=self.point_task.execute_task)
                    task_thread.daemon = True
                    task_thread.start()
                    rospy.loginfo("First point inspection task thread started")
                else:
                    rospy.logwarn(f"Cannot start first point task: active_task={self.active_task}")
            
            elif msg.data != 0:
                rospy.logwarn(f"Unknown command: {msg.data}")

    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("Shutdown signal received")
        sys.exit(0)
    
    def run(self):
        """运行主控制器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MainController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main controller: {e}")

