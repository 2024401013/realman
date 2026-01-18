#!/usr/bin/env python3
# test_arm_camera_integration.py - 测试机械臂和相机的集成
import rospy
import cv2
import numpy as np
from arm_controller import ArmController
from realsense_processor import RealSenseProcessor
import threading

class ArmCameraIntegrationTest:
    def __init__(self):
        rospy.init_node('arm_camera_integration_test')
        
        # 初始化模块
        self.arm = ArmController()
        self.camera = RealSenseProcessor()
        
        # 测试状态
        self.test_results = {}
        
        # 确保模块就绪
        rospy.sleep(2.0)
        rospy.loginfo("Arm-Camera Integration Test initialized")
    
    def test_basic_movement_with_camera(self):
        """测试基本运动时相机工作正常"""
        rospy.loginfo("=== Testing Basic Movement with Camera ===")
        
        # 检查相机
        if not self.camera.is_ready():
            rospy.logwarn("✗ Camera not ready")
            return False
        
        # 移动到初始位置
        rospy.loginfo("Moving to home position...")
        if not self.arm.go_home():
            rospy.logwarn("✗ Failed to move home")
            return False
        
        rospy.sleep(1.0)
        
        # 检查相机图像是否正常
        color_img, depth_img = self.camera.get_image()
        if color_img is None or depth_img is None:
            rospy.logwarn("✗ Camera images not available after movement")
            return False
        
        rospy.loginfo(f"Camera working: Color shape={color_img.shape}, Depth shape={depth_img.shape}")
        
        # 移动到另一个位置
        rospy.loginfo("Moving to test position...")
        test_pose = self.arm.create_pose(0.4, 0.0, 0.3)
        if not self.arm.move_to_pose_jp(test_pose, speed=0.2):
            rospy.logwarn("✗ Failed to move to test position")
            return False
        
        rospy.sleep(1.0)
        
        # 再次检查相机
        color_img2, depth_img2 = self.camera.get_image()
        if color_img2 is None or depth_img2 is None:
            rospy.logwarn("✗ Camera images lost after movement")
            return False
        
        rospy.loginfo("✓ Camera works during arm movement")
        return True
    
    def test_coordinate_transform(self):
        """测试坐标变换"""
        rospy.loginfo("=== Testing Coordinate Transform ===")
        
        # 移动机械臂到已知位置
        known_pose = self.arm.create_pose(0.3, 0.0, 0.4)
        rospy.loginfo(f"Moving to known position: {known_pose.position}")
        
        if not self.arm.move_to_pose_jp(known_pose, speed=0.1):
            rospy.logwarn("✗ Failed to move to known position")
            return False
        
        rospy.sleep(2.0)  # 等待稳定
        
        # 获取当前图像
        color_img, depth_img = self.camera.get_image()
        if color_img is None or depth_img is None:
            rospy.logwarn("✗ No camera images")
            return False
        
        # 选择图像中心点进行测试
        center_u, center_v = color_img.shape[1] // 2, color_img.shape[0] // 2
        depth = self.camera.get_depth_at_point(center_u, center_v)
        
        if depth is None:
            rospy.logwarn("✗ No depth at center")
            return False
        
        rospy.loginfo(f"Center depth: {depth} mm")
        
        # 尝试坐标变换
        try:
            point_robot = self.camera.pixel_to_3d_robot(center_u, center_v, depth)
            if point_robot is not None:
                rospy.loginfo(f"Transformed to robot coordinates: {point_robot}")
                rospy.loginfo("✓ Coordinate transform works")
                return True
            else:
                rospy.logwarn("✗ Coordinate transform failed")
                return False
        except Exception as e:
            rospy.logerr(f"✗ Transform error: {e}")
            return False
    
    def test_concurrent_operation(self):
        """测试并发操作"""
        rospy.loginfo("=== Testing Concurrent Operation ===")
        
        # 启动相机监视线程
        camera_ok = [True]
        
        def monitor_camera():
            """监控相机图像流"""
            try:
                for i in range(20):  # 监控2秒
                    img, _ = self.camera.get_image()
                    if img is None:
                        camera_ok[0] = False
                        break
                    rospy.sleep(0.1)
            except Exception as e:
                rospy.logerr(f"Camera monitor error: {e}")
                camera_ok[0] = False
        
        # 启动监控线程
        monitor_thread = threading.Thread(target=monitor_camera)
        monitor_thread.start()
        
        # 同时进行机械臂运动
        rospy.loginfo("Starting arm movements...")
        
        poses = [
            self.arm.create_pose(0.3, 0.0, 0.3),
            self.arm.create_pose(0.4, 0.0, 0.3),
            self.arm.create_pose(0.3, 0.1, 0.3),
            self.arm.create_pose(0.3, -0.1, 0.3),
        ]
        
        for i, pose in enumerate(poses):
            rospy.loginfo(f"Movement {i+1}/{len(poses)}")
            self.arm.move_to_pose_jp(pose, speed=0.15)
            rospy.sleep(0.5)
        
        # 等待监控线程结束
        monitor_thread.join()
        
        if camera_ok[0]:
            rospy.loginfo("✓ Concurrent operation test passed")
            return True
        else:
            rospy.logwarn("✗ Camera failed during concurrent operation")
            return False
    
    def run_tests(self):
        """运行所有集成测试"""
        rospy.loginfo("Starting arm-camera integration tests...")
        
        tests = [
            ("Basic Movement with Camera", self.test_basic_movement_with_camera),
            ("Coordinate Transform", self.test_coordinate_transform),
            ("Concurrent Operation", self.test_concurrent_operation)
        ]
        
        all_passed = True
        for test_name, test_func in tests:
            rospy.loginfo(f"\n{'='*50}")
            try:
                result = test_func()
                self.test_results[test_name] = result
                rospy.loginfo(f"{test_name}: {'✓ PASS' if result else '✗ FAIL'}")
                all_passed = all_passed and result
                rospy.sleep(2.0)
            except Exception as e:
                rospy.logerr(f"{test_name} error: {e}")
                self.test_results[test_name] = False
                all_passed = False
        
        # 最终回到安全位置
        rospy.loginfo("\nReturning to home position...")
        self.arm.go_home()
        
        # 输出总结
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo("INTEGRATION TEST SUMMARY:")
        for test_name, passed in self.test_results.items():
            status = '✓ PASS' if passed else '✗ FAIL'
            rospy.loginfo(f"  {test_name}: {status}")
        
        return all_passed

if __name__ == '__main__':
    try:
        tester = ArmCameraIntegrationTest()
        success = tester.run_tests()
        exit(0 if success else 1)
    except rospy.ROSInterruptException:
        pass