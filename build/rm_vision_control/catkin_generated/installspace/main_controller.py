#!/usr/bin/env python3
import rospy
import signal
import sys
from arm_controller import ArmController
from vision_detector import VisionDetector
from crack_detection import CrackDetectionTask
from point_inspection import PointInspectionTask

class MainController:
    def __init__(self):
        rospy.init_node('main_controller', anonymous=True)
        
        # 初始化模块
        rospy.loginfo("Initializing modules...")
        self.arm_controller = ArmController()
        self.vision_detector = VisionDetector()
        
        # 初始化任务
        self.crack_task = CrackDetectionTask(self.arm_controller, self.vision_detector)
        self.point_task = PointInspectionTask(self.arm_controller, self.vision_detector)
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("Main Controller initialized successfully")
        rospy.loginfo("Waiting for car commands...")
        rospy.loginfo("  is_arrive=1: Start crack detection")
        rospy.loginfo("  is_arrive=2: Stop crack detection")
        rospy.loginfo("  is_arrive=3: Start 16-point inspection")
    
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