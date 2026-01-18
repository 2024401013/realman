#!/usr/bin/env python

# src/rm_vision_control/scripts/vision_detector.py
import rospy
import cv2
import torch
import numpy as np
import json
import os
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class VisionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
        # ==== 需要根据实际情况修改的配置 ====
        # YOLOv8模型路径
        self.crack_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model.pt'      # 裂缝检测模型
        self.target_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/target_model.pt'    # 目标检测模型
        
        # 相机内参
        self.camera_matrix = np.array([
            [643.464233398438, 0, 653.559936523438],  # fx, 0, cx
            [0, 642.685913085938, 402.54541015625],    # 0, fy, cy
            [0, 0, 1]                                  # 0, 0, 1
        ], dtype=np.float32)

        # 畸变系数
        self.dist_coeffs = np.array([
            [-0.0545824803411961],  # k1
            [0.0650298744440079],   # k2
            [-0.000872006814461201],# p1
            [0.00100247433874756],  # p2
            [-0.0212575811892748]   # k3
        ], dtype=np.float32)
        
        # 手眼标定结果（需要根据实际标定修改）
        self.T_camera_to_robot = np.eye(4)  # 相机到机械臂基座的变换矩阵
        # =================================
        
        # 加载YOLOv8模型
        self.crack_model = None
        self.target_model = None
        try:
            self.crack_model = YOLO(self.crack_model_path)
            rospy.loginfo(f"Loaded crack detection model: {self.crack_model_path}")
        except Exception as e:
            rospy.logwarn(f"Failed to load crack model: {e}")
            
        try:
            self.target_model = YOLO(self.target_model_path)
            rospy.loginfo(f"Loaded target detection model: {self.target_model_path}")
        except Exception as e:
            rospy.logwarn(f"Failed to load target model: {e}")
        
        # 订阅相机话题
        self.color_image = None
        self.depth_image = None
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        
        # 创建结果保存目录
        self.result_dir = "detection_results"
        os.makedirs(self.result_dir, exist_ok=True)
        
        rospy.loginfo("VisionDetector initialized")
    
    def color_callback(self, msg):
        """RGB图像回调"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"Error converting color image: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def detect_cracks(self):
        """检测裂缝"""
        if self.color_image is None or self.crack_model is None:
            return {'detected': False}
        
        # 执行YOLOv8检测
        results = self.crack_model(self.color_image)
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            # 获取第一个检测结果
            boxes = results[0].boxes
            bbox = boxes.xyxy[0].cpu().numpy()
            confidence = boxes.conf[0].cpu().numpy()
            
            # 计算中心点
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            # 获取深度值
            depth = None
            if self.depth_image is not None:
                depth = self.depth_image[center_y, center_x] / 1000.0  # 转换为米
                
                # 转换到3D坐标（相机坐标系）
                point_3d_camera = self.pixel_to_3d(center_x, center_y, depth)
                
                # 转换到机械臂基座坐标系
                point_3d_robot = self.transform_to_robot(point_3d_camera)
            else:
                point_3d_robot = None
            
            # 绘制检测结果
            annotated_image = results[0].plot()
            
            result = {
                'detected': True,
                'confidence': float(confidence),
                'bbox': bbox.tolist(),
                'center_pixel': (center_x, center_y),
                'depth': depth,
                'point_3d_robot': point_3d_robot,
                'annotated_image': annotated_image,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            return result
        
        return {'detected': False}
    
    def detect_target(self):
        """检测目标（16个圆点的矩形）"""
        if self.color_image is None or self.target_model is None:
            return None
        
        results = self.target_model(self.color_image)
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            # 获取目标矩形框
            boxes = results[0].boxes
            bbox = boxes.xyxy[0].cpu().numpy()
            
            # 计算中心点
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            # 获取深度值
            if self.depth_image is not None:
                depth = self.depth_image[center_y, center_x] / 1000.0
                
                # 获取中心点的3D坐标
                point_3d_camera = self.pixel_to_3d(center_x, center_y, depth)
                point_3d_robot = self.transform_to_robot(point_3d_camera)
                
                return point_3d_robot
            
        return None
    
    def calculate_16_points(self, center_point, grid_size=0.05, rows=4, cols=4):
        """计算16个点的3D坐标"""
        points = []
        
        # center_point是机械臂基座坐标系下的3D点
        start_x = center_point[0] - (cols - 1) * grid_size / 2
        start_y = center_point[1] - (rows - 1) * grid_size / 2
        
        for i in range(rows):
            for j in range(cols):
                x = start_x + j * grid_size
                y = start_y + i * grid_size
                z = center_point[2]  # 深度相同
                points.append((x, y, z))
        
        return points
    
    def pixel_to_3d(self, u, v, depth):
        """像素坐标转3D坐标（相机坐标系）"""
        if depth is None or depth <= 0:
            rospy.logdebug(f"Invalid depth at ({u}, {v}): {depth}")
            return None
        
        # 反投影
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def transform_to_robot(self, point_camera):
        """将相机坐标系下的点转换到机械臂基座坐标系"""
        if point_camera is None:
            return None
        
        # 转换为齐次坐标
        point_camera_h = np.append(point_camera, 1)
        point_robot_h = np.dot(self.T_camera_to_robot, point_camera_h)
        
        return point_robot_h[:3]
    
    def save_detection_result(self, detection_result, save_image=True):
        """保存检测结果"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        # 保存图像
        if save_image and 'annotated_image' in detection_result:
            image_path = os.path.join(self.result_dir, f"{timestamp}.jpg")
            cv2.imwrite(image_path, detection_result['annotated_image'])
            detection_result['image_path'] = image_path
        
        # 保存元数据
        metadata = {
            'timestamp': detection_result.get('timestamp', rospy.Time.now().to_sec()),
            'detection_time': datetime.now().isoformat(),
            'detected': detection_result.get('detected', False),
            'confidence': detection_result.get('confidence', 0.0),
            'point_3d_robot': detection_result.get('point_3d_robot'),
            'bbox': detection_result.get('bbox'),
            'center_pixel': detection_result.get('center_pixel'),
            'image_path': detection_result.get('image_path', '')
        }
        
        # 保存为JSON
        json_path = os.path.join(self.result_dir, f"{timestamp}.json")
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        rospy.loginfo(f"Detection result saved: {json_path}")
        return json_path