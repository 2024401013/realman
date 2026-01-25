#!/usr/bin/env python
# src/rm_vision_control/scripts/vision_detector.py
import numpy as np
np.bool = bool      # 核心 patch
np.bool_ = np.bool_ # 如果需要 scalar 类型（可选）

import rospy
import cv2
import torch
import json
import os
import tf
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
from datetime import datetime

class VisionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
        # ==== 模型路径 ====
        self.crack_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/crack_best.engine'
        self.target_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/target_model.engine'
        
        # ==== 相机内参（保持你的硬编码参数）====
        self.camera_matrix = np.array([
            [643.464233398438, 0, 653.559936523438],  # fx, 0, cx
            [0, 642.685913085938, 402.54541015625],    # 0, fy, cy
            [0, 0, 1]                                  # 0, 0, 1
        ], dtype=np.float32)
        
        # ==== 深度处理参数 ====
        self.depth_scale = 0.001  # RealSense 深度图缩放因子 (mm to m)
        self.depth_roi_size = 15
        self.depth_min_threshold = 300
        self.depth_max_threshold = 8000
        
        # ==== 相机到机械臂末端的固定偏移 ====
        self.camera_to_link6_trans = np.array([-0.075, 0.0, -0.038])
        self.camera_to_link6_rot = np.array([0.0, 0.0, 0.0, 1.0])  # 四元数（无旋转）
        
        # ==== TF监听器 ====
        self.tf_listener = tf.TransformListener()
        
        # ==== 加载 TensorRT 模型 ====
        self.crack_model = None
        self.target_model = None
        self.load_models()
        self.warmup_models()
        
        # ==== 图像订阅 ====
        self.color_image = None
        self.depth_image = None
        self.image_lock = threading.Lock()
        
        # 订阅图像话题
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        
        # ==== 创建结果保存目录 ====
        self.result_dir = "/home/nvidia/rm_robot/src/rm_vision_control/detection_results"
        os.makedirs(self.result_dir, exist_ok=True)
        rospy.loginfo(f"Saving results to: {self.result_dir}")

        self.crack_annotated_pub = rospy.Publisher('/vision/crack_annotated', Image, queue_size=1)
        self.target_annotated_pub = rospy.Publisher('/vision/target_annotated', Image, queue_size=1)
        
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform(
            self.camera_to_link6_trans,
            self.camera_to_link6_rot,
            rospy.Time.now(),
            "camera_link",
            "Link6"
        )

        rospy.loginfo("VisionDetector initialized.")

    def warmup_models(self):
        """在启动时对两个模型各跑一次 dummy 推理，消除首次延迟"""
        if self.crack_model is not None:
            rospy.loginfo("Warm-up crack model...")
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)  # 随便一张黑图，大小匹配模型输入
            try:
                _ = self.crack_model(dummy_img, conf=0.4, verbose=False)
                rospy.loginfo("Crack model warm-up completed")
            except Exception as e:
                rospy.logwarn(f"Crack model warm-up failed: {e}")

        if self.target_model is not None:
            rospy.loginfo("Warm-up target model...")
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            try:
                _ = self.target_model(dummy_img, conf=0.3, verbose=False)
                rospy.loginfo("Target model warm-up completed")
            except Exception as e:
                rospy.logwarn(f"Target model warm-up failed: {e}")

        rospy.loginfo("All models warmed up. Ready for fast inference.")

    def load_models(self):
        """加载 TensorRT engine 模型"""
        try:
            # 显式指定 task='detect' 和 device='cuda:0'，确保使用 TensorRT + GPU
            self.crack_model = YOLO(self.crack_model_path, task='detect')
            rospy.loginfo(f"Loaded crack TensorRT engine: {self.crack_model_path}")
            rospy.loginfo(f"Crack model device: {self.crack_model.device}")
        except Exception as e:
            rospy.logerr(f"Failed to load crack TensorRT engine: {e}")
            self.crack_model = None
        
        try:
            self.target_model = YOLO(self.target_model_path, task='detect')
            rospy.loginfo(f"Loaded target TensorRT engine: {self.target_model_path}")
            rospy.loginfo(f"Target model device: {self.target_model.device}")
        except Exception as e:
            rospy.logerr(f"Failed to load target TensorRT engine: {e}")
            self.target_model = None

    def color_callback(self, msg):
        """RGB 图像回调"""
        try:
            with self.image_lock:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"Error converting color image: {e}")

    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            with self.image_lock:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def calculate_robust_depth(self, depth_image, x, y, roi_size=10):
        """在指定点周围计算鲁棒的深度值"""
        h, w = depth_image.shape
        
        # 确定ROI边界
        x_min = max(0, int(x) - roi_size)
        x_max = min(w, int(x) + roi_size + 1)
        y_min = max(0, int(y) - roi_size)
        y_max = min(h, int(y) + roi_size + 1)
        
        # 提取ROI区域
        roi = depth_image[y_min:y_max, x_min:x_max]
        
        # 获取有效深度值（排除无效值）
        valid_mask = (roi > self.depth_min_threshold) & (roi < self.depth_max_threshold)
        valid_depths = roi[valid_mask]
        
        if len(valid_depths) == 0:
            # 如果没有有效值，尝试扩大ROI
            roi_size_large = roi_size * 2
            x_min = max(0, int(x) - roi_size_large)
            x_max = min(w, int(x) + roi_size_large + 1)
            y_min = max(0, int(y) - roi_size_large)
            y_max = min(h, int(y) + roi_size_large + 1)
            roi = depth_image[y_min:y_max, x_min:x_max]
            valid_mask = (roi > self.depth_min_threshold) & (roi < self.depth_max_threshold)
            valid_depths = roi[valid_mask]
            
            if len(valid_depths) == 0:
                rospy.logdebug(f"No valid depths at ({x:.1f}, {y:.1f})")
                return 0
        
        # 使用中值滤波（最鲁棒）
        median_depth = np.median(valid_depths)
        
        # 可选：也可以计算均值
        # mean_depth = np.mean(valid_depths)
        
        return median_depth
    
    def pixel_to_3d_camera(self, u, v, depth_mm):
        """像素坐标转3D坐标（相机坐标系）"""
        if depth_mm <= 0:
            return None
        
        # 转换为米
        depth = depth_mm * self.depth_scale
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def transform_to_robot(self, point_camera):
        if point_camera is None:
            rospy.logwarn("point_camera is None")
            return None
        
        try:
            # 使用最新可用时间戳
            time = rospy.Time(0)
            
            # 关键诊断1：打印 link6 当前在 base_link 的真实位置
            (link6_trans, link6_rot) = self.tf_listener.lookupTransform(
                "base_link", "Link6", time
            )
            rospy.loginfo("=== TF 诊断 ===")
            rospy.loginfo(f"Link6 在 base_link 的位置: X={link6_trans[0]:.3f} m, Y={link6_trans[1]:.3f} m, Z={link6_trans[2]:.3f} m")
            rospy.loginfo(f"Link6 旋转四元数: {link6_rot}")
            
            # 打印相机到 link6 的固定偏移（确认是否正确）
            rospy.loginfo(f"相机到 link6 固定偏移: {self.camera_to_link6_trans}")
            
            # 相機 → Link6 變換矩陣
            camera_to_link6_mat = self.tf_listener.fromTranslationRotation(
                self.camera_to_link6_trans, self.camera_to_link6_rot
            )
            
            # 相機坐標 → Link6 坐標
            point_camera_h = np.append(point_camera, 1)
            point_link6_h = np.dot(camera_to_link6_mat, point_camera_h)
            point_link6 = point_link6_h[:3]
            rospy.loginfo(f"点在 link6 坐标系: X={point_link6[0]:.3f}, Y={point_link6[1]:.3f}, Z={point_link6[2]:.3f}")
            
            # Link6 → base_link 變換矩陣
            link6_to_base_mat = self.tf_listener.fromTranslationRotation(link6_trans, link6_rot)
            
            # Link6 坐標 → base_link 坐標
            point_link6_h = np.append(point_link6, 1)
            point_base_h = np.dot(link6_to_base_mat, point_link6_h)
            point_base = point_base_h[:3]
            
            # 最终输出 + 诊断总结
            rospy.loginfo(f"最终点在 base_link: X={point_base[0]:.3f}, Y={point_base[1]:.3f}, Z={point_base[2]:.3f}")
            rospy.loginfo(f"相機深度 Z={point_camera[2]:.3f} m → base_link Z={point_base[2]:.3f} m")
            rospy.loginfo(f"Z 差值: {point_base[2] - point_camera[2]:.3f} m (应该接近 link6 的 Z 或偏移)")
            rospy.loginfo("=== 诊断结束 ===\n")
            
            return point_base
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF transform error (throttled): {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Unexpected error in transform_to_robot: {e}")
            return None
            sss
    def detect_cracks(self):
        """检测裂缝"""
        if self.color_image is None or self.crack_model is None:
            rospy.logdebug("No image or crack model")
            return {'detected': False}
        
        with self.image_lock:
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy() if self.depth_image is not None else None
        
        try:
            results = self.crack_model(color_img, conf=0.4, verbose=False)
        except Exception as e:
            rospy.logerr(f"Crack detection error: {e}")
            return {'detected': False}
        
        # 总是生成带提示的 annotated_image
        annotated_image = results[0].plot()  # Ultralytics 自带画框
        
        detected = len(results[0].boxes) > 0
        
        if detected:
            # 有检测到 - 正常处理
            boxes = results[0].boxes
            bbox = boxes.xyxy[0].cpu().numpy()  # 取第一个（或最高 conf 的）
            confidence = boxes.conf[0].cpu().numpy()
            
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            depth_mm = 0
            point_3d_camera = None
            point_3d_robot = None
            
            if depth_img is not None:
                depth_mm = self.calculate_robust_depth(depth_img, center_x, center_y, self.depth_roi_size)
                if depth_mm > 0:
                    point_3d_camera = self.pixel_to_3d_camera(center_x, center_y, depth_mm)
                    if point_3d_camera is not None:
                        point_3d_robot = self.transform_to_robot(point_3d_camera)
            
            result = {
                'detected': True,
                'confidence': float(confidence),
                'bbox': bbox.tolist(),
                'center_pixel': (center_x, center_y),
                'depth_mm': depth_mm,
                'depth_m': depth_mm * self.depth_scale if depth_mm > 0 else 0,
                'point_3d_camera': point_3d_camera.tolist() if point_3d_camera is not None else None,
                'point_3d_robot': point_3d_robot.tolist() if point_3d_robot is not None else None,
                'timestamp': datetime.now().timestamp(),
                'annotated_image':annotated_image
            }
            
            rospy.loginfo(f"Crack detected! Conf: {confidence:.2f}, Depth: {depth_mm/1000:.3f}m" if depth_mm > 0 else f"Crack detected! Conf: {confidence:.2f} (No valid depth)")
        
        else:
            # 没检测到 - 加文字提示
            cv2.putText(
                annotated_image,
                "No crack detected",
                (50, 100),                    # 位置
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,                          # 大小
                (0, 0, 255),                  # 红色
                3                             # 粗细
            )
            result = {'detected': False}
        
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            annotated_msg.header.stamp = rospy.Time.now()
            annotated_msg.header.frame_id = "camera_color_optical_frame"  # 改成你的相机 frame
            self.crack_annotated_pub.publish(annotated_msg)
            rospy.logdebug("Published annotated crack image (always)")
        except Exception as e:
            rospy.logerr(f"Failed to publish annotated image: {e}")
        
        # 可选：只在检测到时保存结果
        # if result.get('detected', False):
        #     self.save_detection_result(result)
        
        return result

    def detect_target(self):
        """检测目标"""
        if self.color_image is None or self.target_model is None:
            rospy.logdebug("No image or target model")
            return None
        
        with self.image_lock:
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy() if self.depth_image is not None else None
        
        try:
            results = self.target_model(color_img, conf=0.3, verbose=False)
        except Exception as e:
            rospy.logerr(f"Target detection error: {e}")
            return None
        
        annotated_image = results[0].plot()
        
        detected = len(results[0].boxes) > 0
        
        if detected:
            boxes = results[0].boxes
            bbox = boxes.xyxy[0].cpu().numpy()
            confidence = boxes.conf[0].cpu().numpy()
            
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            depth_mm = 0
            point_3d_camera = None
            point_3d_robot = None
            
            if depth_img is not None:
                depth_mm = self.calculate_robust_depth(depth_img, center_x, center_y, self.depth_roi_size)
                if depth_mm > 0:
                    point_3d_camera = self.pixel_to_3d_camera(center_x, center_y, depth_mm)
                    if point_3d_camera is not None:
                        point_3d_robot = self.transform_to_robot(point_3d_camera)
            
            result = {
                'center_2d': (center_x, center_y),
                'center_3d_robot': point_3d_robot.tolist() if point_3d_robot is not None else None,
                'center_3d_camera': point_3d_camera.tolist() if point_3d_camera is not None else None,
                'confidence': float(confidence),
                'depth_mm': depth_mm,
                'depth_m': depth_mm * self.depth_scale if depth_mm > 0 else 0,
                'bbox': bbox.tolist(),
                'timestamp': datetime.now().isoformat()
            }
            
            rospy.loginfo(f"Target detected! Center at ({center_x}, {center_y}), Depth: {depth_mm/1000:.3f}m")
        
        else:
            # 没检测到 
            cv2.putText(
                annotated_image,
                "No target detected",
                (50, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),  # 绿色
                3
            )
            result = None  # 或 {'detected': False}，根据你原有逻辑
        
        # 总是发布
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            annotated_msg.header.stamp = rospy.Time.now()
            annotated_msg.header.frame_id = "camera_color_optical_frame"
            self.target_annotated_pub.publish(annotated_msg)
            rospy.logdebug("Published annotated target image (always)")
        except Exception as e:
            rospy.logerr(f"Failed to publish annotated image: {e}")
        
        return result

    def calculate_16_points(self, center_point_robot, grid_size=0.05, rows=4, cols=4):
        """计算16个点的3D坐标（蛇形顺序）"""
        points = []
        
        if center_point_robot is None or len(center_point_robot) < 3:
            return points
        
        # 计算起始点（左上角）
        start_x = center_point_robot[0]  # x不变（深度方向）
        start_y = center_point_robot[1] - (cols - 1) * grid_size / 2  # y起始（向左偏移）
        start_z = center_point_robot[2] + (rows - 1) * grid_size / 2  # z起始（向上偏移）
        
        for i in range(rows):
            row_z = start_z - i * grid_size  # 从上往下，z递减
            for j in range(cols):
                # 蛇形顺序：奇数行从左到右，偶数行从右到左
                if i % 2 == 0:  # 偶数行（第0,2行）从左到右
                    col_y = start_y + j * grid_size
                else:  # 奇数行（第1,3行）从右到左
                    col_y = start_y + (cols - 1 - j) * grid_size
                
                points.append((start_x, col_y, row_z))
        
        return points
    
    def save_detection_result(self, detection_result, save_image=True):
        """保存检测结果"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        # 保存图像
        if save_image and 'annotated_image' in detection_result:
            image_path = os.path.join(self.result_dir, f"{timestamp}.jpg")
            try:
                cv2.imwrite(image_path, detection_result['annotated_image'])
                detection_result['image_path'] = image_path
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")
        
        # 保存元数据
        metadata = {
            # 'timestamp': detection_result.get('timestamp', rospy.Time.now().to_sec()),
            'detection_time': datetime.now().isoformat(),
            'detected': detection_result.get('detected', False),
            'confidence': detection_result.get('confidence', 0.0),
            'point_3d_robot': detection_result.get('point_3d_robot'),
            'point_3d_camera': detection_result.get('point_3d_camera'),
            'depth_mm': detection_result.get('depth_mm'),
            'depth_m': detection_result.get('depth_m'),
            'bbox': detection_result.get('bbox'),
            'center_pixel': detection_result.get('center_pixel'),
            'image_path': detection_result.get('image_path', '')
        }
        
        # 保存为JSON
        json_path = os.path.join(self.result_dir, f"{timestamp}.json")
        try:
            with open(json_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            rospy.loginfo(f"Detection result saved: {json_path}")
            return json_path
        except Exception as e:
            rospy.logerr(f"Failed to save JSON: {e}")
            return None

if __name__ == "__main__":
    rospy.init_node("vision_detector_node")
    detector = VisionDetector()
    rospy.spin()