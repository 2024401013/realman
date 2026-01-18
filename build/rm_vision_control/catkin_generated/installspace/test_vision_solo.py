#!/usr/bin/env python3
# test_vision_solo.py - 单独测试YOLOv8检测
import rospy
import cv2
import numpy as np
import os
from datetime import datetime
from ultralytics import YOLO

class VisionSoloTest:
    def __init__(self):
        rospy.init_node('vision_solo_test')
        
        # YOLOv8模型路径
        self.model_path = 'test_model.pt'  # 先用测试模型
        
        # 创建测试目录
        self.test_dir = 'vision_test_results'
        os.makedirs(self.test_dir, exist_ok=True)
        
        # 测试结果
        self.test_results = {}
        
        rospy.loginfo("Vision Solo Test initialized")
    
    def create_test_image(self):
        """创建测试图像"""
        rospy.loginfo("Creating test image...")
        
        # 创建一个640x480的彩色图像
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 添加一些测试图案
        # 1. 随机彩色方块
        for i in range(5):
            x = np.random.randint(100, 540)
            y = np.random.randint(100, 380)
            w = np.random.randint(50, 100)
            h = np.random.randint(50, 100)
            color = (np.random.randint(0, 255), 
                    np.random.randint(0, 255), 
                    np.random.randint(0, 255))
            cv2.rectangle(img, (x, y), (x+w, y+h), color, -1)
        
        # 2. 线条
        cv2.line(img, (100, 100), (300, 400), (0, 255, 0), 2)
        cv2.line(img, (500, 100), (300, 400), (255, 0, 0), 2)
        
        # 3. 文本
        cv2.putText(img, 'YOLOv8 Test Image', (150, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return img
    
    def test_model_loading(self):
        """测试模型加载"""
        rospy.loginfo("=== Testing Model Loading ===")
        
        try:
            # 尝试加载模型
            model = YOLO(self.model_path)
            rospy.loginfo(f"✓ Model loaded from {self.model_path}")
            
            # 或者使用预训练模型进行测试
            if not os.path.exists(self.model_path):
                rospy.loginfo("Using pretrained YOLOv8n model for testing...")
                model = YOLO('yolov8n.pt')
                rospy.loginfo("✓ Pretrained model loaded")
            
            self.model = model
            self.test_results['model_loading'] = True
            return True
            
        except Exception as e:
            rospy.logerr(f"✗ Model loading error: {e}")
            rospy.loginfo("Tip: Install YOLOv8 with: pip install ultralytics")
            self.test_results['model_loading'] = False
            return False
    
    def test_inference_speed(self):
        """测试推理速度"""
        rospy.loginfo("=== Testing Inference Speed ===")
        
        if not hasattr(self, 'model'):
            rospy.logwarn("✗ Model not loaded")
            self.test_results['inference_speed'] = False
            return False
        
        # 创建测试图像
        test_img = self.create_test_image()
        
        # 保存测试图像
        test_img_path = os.path.join(self.test_dir, 'test_image.jpg')
        cv2.imwrite(test_img_path, test_img)
        
        # 进行多次推理测试速度
        rospy.loginfo("Running inference tests...")
        times = []
        num_tests = 10
        
        for i in range(num_tests):
            start_time = rospy.Time.now()
            
            # 执行推理
            results = self.model(test_img, verbose=False)
            
            end_time = rospy.Time.now()
            inference_time = (end_time - start_time).to_sec()
            times.append(inference_time)
            
            if i == 0:  # 第一次推理的结果
                num_detections = len(results[0].boxes) if results[0].boxes is not None else 0
                rospy.loginfo(f"First inference: {num_detections} detections")
        
        # 计算统计
        avg_time = np.mean(times) * 1000  # 转换为毫秒
        std_time = np.std(times) * 1000
        fps = 1.0 / np.mean(times)
        
        rospy.loginfo(f"Average inference time: {avg_time:.1f} ± {std_time:.1f} ms")
        rospy.loginfo(f"Average FPS: {fps:.1f}")
        
        # 检查是否在合理范围内
        if avg_time < 1000:  # 小于1秒
            rospy.loginfo("✓ Inference speed is reasonable")
            self.test_results['inference_speed'] = True
        else:
            rospy.logwarn("✗ Inference speed is slow")
            self.test_results['inference_speed'] = False
        
        return self.test_results['inference_speed']
    
    def test_result_processing(self):
        """测试结果处理"""
        rospy.loginfo("=== Testing Result Processing ===")
        
        if not hasattr(self, 'model'):
            rospy.logwarn("✗ Model not loaded")
            self.test_results['result_processing'] = False
            return False
        
        # 创建测试图像
        test_img = self.create_test_image()
        
        # 执行推理
        results = self.model(test_img)
        
        if len(results) == 0:
            rospy.logwarn("✗ No results from model")
            self.test_results['result_processing'] = False
            return False
        
        result = results[0]
        
        # 检查是否有检测框
        if result.boxes is not None:
            boxes = result.boxes
            num_detections = len(boxes)
            
            rospy.loginfo(f"Detected {num_detections} objects")
            
            # 提取信息
            for i, box in enumerate(boxes):
                bbox = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls = box.cls[0].cpu().numpy()
                
                rospy.loginfo(f"  Object {i+1}:")
                rospy.loginfo(f"    BBox: {bbox}")
                rospy.loginfo(f"    Confidence: {conf:.2f}")
                rospy.loginfo(f"    Class: {cls}")
            
            # 绘制检测结果
            annotated_img = result.plot()
            
            # 保存结果
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.test_dir, f'detection_{timestamp}.jpg')
            cv2.imwrite(save_path, annotated_img)
            rospy.loginfo(f"✓ Saved annotated image to {save_path}")
            
            # 显示图像
            cv2.imshow('Detection Results', annotated_img)
            cv2.waitKey(3000)
            cv2.destroyAllWindows()
            
            self.test_results['result_processing'] = True
            return True
        else:
            rospy.loginfo("No objects detected (this is normal for test images)")
            self.test_results['result_processing'] = True  # 仍然算通过
            return True
    
    def test_save_load_results(self):
        """测试结果保存和加载"""
        rospy.loginfo("=== Testing Save/Load Results ===")
        
        # 创建测试结果
        test_result = {
            'timestamp': datetime.now().isoformat(),
            'detections': [
                {'bbox': [100, 100, 200, 200], 'confidence': 0.85, 'class': 0},
                {'bbox': [300, 300, 400, 400], 'confidence': 0.72, 'class': 1}
            ],
            'image_size': [640, 480]
        }
        
        # 保存为JSON
        import json
        save_path = os.path.join(self.test_dir, 'test_result.json')
        with open(save_path, 'w') as f:
            json.dump(test_result, f, indent=2)
        
        rospy.loginfo(f"✓ Saved test result to {save_path}")
        
        # 加载并验证
        with open(save_path, 'r') as f:
            loaded_result = json.load(f)
        
        if loaded_result['timestamp'] == test_result['timestamp']:
            rospy.loginfo("✓ Result save/load test passed")
            self.test_results['save_load'] = True
            return True
        else:
            rospy.logwarn("✗ Result save/load test failed")
            self.test_results['save_load'] = False
            return False
    
    def run_tests(self):
        """运行所有测试"""
        rospy.loginfo("Starting vision module tests...")
        
        tests = [
            ("Model Loading", self.test_model_loading),
            ("Inference Speed", self.test_inference_speed),
            ("Result Processing", self.test_result_processing),
            ("Save/Load Results", self.test_save_load_results)
        ]
        
        all_passed = True
        for test_name, test_func in tests:
            rospy.loginfo(f"\n{'='*50}")
            try:
                result = test_func()
                rospy.loginfo(f"{test_name}: {'✓ PASS' if result else '✗ FAIL'}")
                all_passed = all_passed and result
                rospy.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"{test_name} error: {e}")
                all_passed = False
        
        # 输出总结
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo("TEST SUMMARY:")
        for test_name, passed in self.test_results.items():
            status = '✓ PASS' if passed else '✗ FAIL'
            rospy.loginfo(f"  {test_name}: {status}")
        
        rospy.loginfo(f"\nTest results saved in: {os.path.abspath(self.test_dir)}")
        
        return all_passed

if __name__ == '__main__':
    try:
        tester = VisionSoloTest()
        success = tester.run_tests()
        exit(0 if success else 1)
    except rospy.ROSInterruptException:
        pass