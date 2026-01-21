#!/usr/bin/env python

# src/rm_vision_control/scripts/vision_detector.py
import os
os.environ["ORT_DISABLE_OPSET_VALIDATION"] = "1"
os.environ["ORT_DISABLE_ONNX_VERSION_CHECK"] = "1"
os.environ["ORT_DISABLE_AFFINITY"] = "1"

import rospy
import cv2
import torch
import numpy as np
import json
import tf  
from datetime import datetime
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import onnxruntime as ort
import threading
import time


class ONNXDetector:
    """ONNX 模型推理器（适配 Opset 20 + IR 9 + Jetson）"""
    def __init__(self, model_path, conf_threshold=0.25, iou_threshold=0.45):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.session = None
        self.input_name = None
        self.output_name = None
        self.input_shape = None
        self.input_size = 640  # 默认尺寸，加载模型后更新
        self.load_model()
    
    def load_model(self):
        try:
            rospy.loginfo(f"🔍 实际 ORT 版本: {ort.__version__}")
            # 配置 Session
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess_options.intra_op_num_threads = 4
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=sess_options,
                providers=providers
            )

            # ====== 动态获取输入尺寸 ======
            input_shape = self.session.get_inputs()[0].shape
            if len(input_shape) == 4:
                self.input_size = input_shape[2]  # height
                rospy.loginfo(f"ℹ️ 模型输入尺寸: {self.input_size}x{self.input_size}")
            else:
                rospy.logwarn(f"⚠️ 未知输入形状: {input_shape}, 使用默认 640")
                self.input_size = 640
            # ===========================

            # ====== 模型推理验证 ======
            dummy = np.random.randn(1, 3, self.input_size, self.input_size).astype(np.float32)
            self.session.run(None, {self.session.get_inputs()[0].name: dummy})
            rospy.loginfo("✅ 模型推理验证通过")
            # ========================

            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            rospy.loginfo(f"✅ 模型加载成功: {self.model_path}")       

        except Exception as e:
            rospy.logerr(f"❌ 模型加载/验证失败: {e}")
            self.session = None
            self.input_size = 640  

    
    def preprocess(self, image):
        """图像预处理（自动适配模型输入尺寸）"""
        # 调整大小（使用模型输入尺寸）
        img_resized = cv2.resize(image, (self.input_size, self.input_size))
        
        # BGR -> RGB
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        
        # 归一化
        img_normalized = img_rgb.astype(np.float32) / 255.0
        
        # HWC -> CHW
        img_chw = img_normalized.transpose(2, 0, 1)
        
        # 添加批次维度
        img_batch = np.expand_dims(img_chw, axis=0)
        
        return img_batch
    
    def postprocess(self, outputs, original_shape, conf_thresh=None):
        """后处理输出"""
        if conf_thresh is None:
            conf_thresh = self.conf_threshold
        
        if len(outputs) == 0 or outputs[0] is None:
            return []
        
        predictions = outputs[0][0]  # 取第一个批次，形状为 [num_predictions, 6]
        
        if len(predictions) == 0:
            return []
        
        # 过滤低置信度
        if predictions.shape[1] >= 5:
            mask = predictions[:, 4] > conf_thresh
            predictions = predictions[mask]
        
        results = []
        h, w = original_shape[:2]
        
        for pred in predictions:
            if len(pred) >= 6:
                x1, y1, x2, y2, conf, cls = pred[:6]
                
                # 反归一化到原始图像尺寸
                x1 = int(x1 * w / self.input_size)
                y1 = int(y1 * h / self.input_size)
                x2 = int(x2 * w / self.input_size)
                y2 = int(y2 * h / self.input_size)
                
                # 确保坐标在图像范围内
                x1 = max(0, min(x1, w))
                y1 = max(0, min(y1, h))
                x2 = max(0, min(x2, w))
                y2 = max(0, min(y2, h))
                
                # 计算中心点
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                results.append({
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'center': [center_x, center_y],
                    'confidence': float(conf),
                    'class_id': int(cls),
                    'class_name': f'class{int(cls)}'
                })
        
        return results
    
    def detect(self, image, conf_threshold=None):
        """执行检测"""
        if self.session is None:
            rospy.logwarn("ONNX 模型未加载")
            return []
        
        if conf_threshold is None:
            conf_threshold = self.conf_threshold
        
        # 记录推理时间
        start_time = time.time()
        
        try:
            # 预处理（自动适配尺寸）
            img_tensor = self.preprocess(image)
            
            # 推理
            outputs = self.session.run([self.output_name], {self.input_name: img_tensor})
            
            # 后处理
            results = self.postprocess(outputs, image.shape, conf_threshold)
            
            inference_time = (time.time() - start_time) * 1000
            
            if len(results) > 0:
                rospy.logdebug(f"推理时间: {inference_time:.1f}ms, 检测到 {len(results)} 个目标")
            
            return results
            
        except Exception as e:
            rospy.logerr(f"ONNX 推理失败: {e}")
            return []

class VisionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
        # ==== 模型路径（改为转换后的 Opset 20 + IR 9 版本）====
        self.crack_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model.onnx' 
        self.target_model_path = '/home/nvidia/rm_robot/src/rm_vision_control/models/target_model.onnx' 
        
        # ==== 相机内参 ====
        self.camera_matrix = np.array([
            [643.464233398438, 0, 653.559936523438],
            [0, 642.685913085938, 402.54541015625],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # ==== 深度处理参数 ====
        self.depth_scale = 0.001  # mm to m
        self.depth_roi_size = 15
        self.depth_min_threshold = 300
        self.depth_max_threshold = 8000
        
        # ==== 相机到机械臂末端的固定偏移 ====
        self.camera_to_link6_trans = np.array([0.0, -0.075, 0.028])
        self.camera_to_link6_rot = np.array([0.0, 0.0, 0.0, 1.0])
        
        # ==== TF监听器 ====
        self.tf_listener = tf.TransformListener()
        
        # ==== 加载 ONNX 模型 ====
        self.crack_detector = None
        self.target_detector = None
        self.load_onnx_models()
        
        # ==== 图像订阅 ====
        self.color_image = None
        self.depth_image = None
        self.image_lock = threading.Lock()
        
        # 订阅图像话题
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        
        # ==== 检测结果可视化 ====
        self.visualization_enabled = rospy.get_param('~visualization', True)
        
        # ==== 创建结果保存目录 ====
        self.result_dir = "detection_results"
        os.makedirs(self.result_dir, exist_ok=True)
        
        rospy.loginfo("✅ VisionDetector initialized with ONNX models")
    
    def color_callback(self, msg):
        """RGB图像回调"""
        try:
            with self.image_lock:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"❌ Error converting color image: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            with self.image_lock:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            rospy.logerr(f"❌ Error converting depth image: {e}")
    
    def load_onnx_models(self):
        """加载 ONNX 模型"""
        try:
            self.crack_detector = ONNXDetector(self.crack_model_path, conf_threshold=0.55)
            rospy.loginfo(f"✅ Loaded crack ONNX model: {self.crack_model_path}")
        except Exception as e:
            rospy.logerr(f"❌ Failed to load crack ONNX model: {e}")
            self.crack_detector = None
        
        try:
            self.target_detector = ONNXDetector(self.target_model_path, conf_threshold=0.3)
            rospy.loginfo(f"✅ Loaded target ONNX model: {self.target_model_path}")
        except Exception as e:
            rospy.logerr(f"❌ Failed to load target ONNX model: {e}")
            self.target_detector = None
    
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
        
        # 获取有效深度值
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
                rospy.logdebug(f"❌ No valid depths at ({x:.1f}, {y:.1f})")
                return 0
        
        # 使用中值滤波
        median_depth = np.median(valid_depths)
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
        """将相机坐标系点转换到机械臂基座坐标系"""
        if point_camera is None:
            return None
        
        try:
            # 1. 获取link6在base_link下的实时位姿
            (link6_trans, link6_rot) = self.tf_listener.lookupTransform(
                "base_link", "link6", rospy.Time(0)
            )
            
            # 2. 构建「相机→link6」的变换矩阵
            camera_to_link6_mat = self.tf_listener.fromTranslationRotation(
                self.camera_to_link6_trans, self.camera_to_link6_rot
            )
            
            # 3. 相机系点转齐次坐标
            point_camera_h = np.append(point_camera, 1)
            
            # 4. 转换到link6系
            point_link6_h = np.dot(camera_to_link6_mat, point_camera_h)
            point_link6 = point_link6_h[:3]
            
            # 5. 构建「link6→base_link」的变换矩阵
            link6_to_base_mat = self.tf_listener.fromTranslationRotation(link6_trans, link6_rot)
            
            # 6. link6系点转齐次坐标
            point_link6_h = np.append(point_link6, 1)
            
            # 7. 转换到base_link系
            point_base_h = np.dot(link6_to_base_mat, point_link6_h)
            point_base = point_base_h[:3]
            
            return point_base
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"⚠️ TF transform error: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"❌ Unexpected error in transform_to_robot: {e}")
            return None
    
    def visualize_detection(self, image, detections, model_name="detection"):
        """可视化检测结果（添加异常保护）"""
        if not self.visualization_enabled or image is None:
            return image
        
        try:
            img_viz = image.copy()
            
            for det in detections:
                bbox = det.get('bbox', [])
                center = det.get('center', [0, 0])
                confidence = det.get('confidence', 0)
                class_name = det.get('class_name', 'object')
                
                if len(bbox) == 4:
                    x1, y1, x2, y2 = map(int, bbox)
                    
                    # 绘制边界框
                    color = (0, 255, 0)  # 绿色
                    cv2.rectangle(img_viz, (x1, y1), (x2, y2), color, 2)
                    
                    # 绘制中心点
                    cv2.circle(img_viz, (center[0], center[1]), 5, (0, 0, 255), -1)
                    
                    # 绘制标签
                    label = f"{class_name}: {confidence:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    cv2.rectangle(img_viz, (x1, y1 - label_size[1] - 10),
                                 (x1 + label_size[0], y1), color, -1)
                    cv2.putText(img_viz, label, (x1, y1 - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # 显示FPS或模型名称
            cv2.putText(img_viz, f"Model: {model_name}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            return img_viz
        except Exception as e:
            rospy.logerr(f"❌ Visualization error: {e}")
            return image
    
    def detect_cracks(self):
        """检测裂缝"""
        if self.color_image is None or self.crack_detector is None:
            rospy.logdebug("❌ No image or ONNX model for crack detection")
            return {'detected': False}
        
        # 获取图像副本
        with self.image_lock:
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy() if self.depth_image is not None else None
        
        try:
            # 使用 ONNX 模型检测
            detections = self.crack_detector.detect(color_img)
            
        except Exception as e:
            rospy.logerr(f"❌ Crack detection error: {e}")
            return {'detected': False}
        
        if len(detections) > 0:
            # 取置信度最高的检测结果
            detections.sort(key=lambda x: x['confidence'], reverse=True)
            best_det = detections[0]
            
            bbox = best_det['bbox']
            confidence = best_det['confidence']
            center_x, center_y = best_det['center']
            
            # 获取深度值
            depth_mm = 0
            point_3d_camera = None
            point_3d_robot = None
            
            if depth_img is not None:
                depth_mm = self.calculate_robust_depth(depth_img, center_x, center_y, self.depth_roi_size)
                
                if depth_mm > 0:
                    # 转换到3D坐标
                    point_3d_camera = self.pixel_to_3d_camera(center_x, center_y, depth_mm)
                    
                    if point_3d_camera is not None:
                        # 转换到机械臂基座坐标系
                        point_3d_robot = self.transform_to_robot(point_3d_camera)
            
            # 可视化
            annotated_image = self.visualize_detection(color_img, detections, "Crack")
            
            result = {
                'detected': True,
                'confidence': float(confidence),
                'bbox': bbox,
                'center_pixel': (center_x, center_y),
                'depth_mm': depth_mm,
                'depth_m': depth_mm * self.depth_scale if depth_mm > 0 else 0,
                'point_3d_camera': point_3d_camera.tolist() if point_3d_camera is not None else None,
                'point_3d_robot': point_3d_robot.tolist() if point_3d_robot is not None else None,
                'annotated_image': annotated_image,
                'detections': detections,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            if depth_mm > 0:
                rospy.loginfo(f"✅ Crack detected! Conf: {confidence:.2f}, Depth: {depth_mm/1000:.3f}m")
            else:
                rospy.loginfo(f"✅ Crack detected! Conf: {confidence:.2f} (No valid depth)")
            
            return result
        
        return {'detected': False}
    
    def detect_target(self):
        """检测目标（16个圆点的矩形）"""
        if self.color_image is None or self.target_detector is None:
            rospy.logdebug("❌ No image or ONNX model for target detection")
            return None
        
        # 获取图像副本
        with self.image_lock:
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy() if self.depth_image is not None else None
        
        try:
            detections = self.target_detector.detect(color_img)
            
        except Exception as e:
            rospy.logerr(f"❌ Target detection error: {e}")
            return None
        
        if len(detections) > 0:
            # 取置信度最高的检测结果
            detections.sort(key=lambda x: x['confidence'], reverse=True)
            best_det = detections[0]
            
            bbox = best_det['bbox']
            confidence = best_det['confidence']
            center_x, center_y = best_det['center']
            
            # 获取深度值
            depth_mm = 0
            point_3d_camera = None
            point_3d_robot = None
            
            if depth_img is not None:
                depth_mm = self.calculate_robust_depth(depth_img, center_x, center_y, self.depth_roi_size)
                
                if depth_mm > 0:
                    # 获取中心点的3D坐标
                    point_3d_camera = self.pixel_to_3d_camera(center_x, center_y, depth_mm)
                    
                    if point_3d_camera is not None:
                        point_3d_robot = self.transform_to_robot(point_3d_camera)
            
            # 可视化
            annotated_image = self.visualize_detection(color_img, detections, "Target")
            
            result = {
                'center_2d': (center_x, center_y),
                'center_3d_robot': point_3d_robot.tolist() if point_3d_robot is not None else None,
                'center_3d_camera': point_3d_camera.tolist() if point_3d_camera is not None else None,
                'confidence': float(confidence),
                'depth_mm': depth_mm,
                'depth_m': depth_mm * self.depth_scale if depth_mm > 0 else 0,
                'bbox': bbox,
                'detections': detections,
                'annotated_image': annotated_image,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            if point_3d_robot is not None:
                rospy.loginfo(f"✅ Target detected! Center at ({center_x}, {center_y}), "
                             f"Depth: {depth_mm/1000:.3f}m, Robot: {point_3d_robot}")
            
            return result
        
        return None
    
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
                rospy.loginfo(f"💾 Image saved: {image_path}")
            except Exception as e:
                rospy.logerr(f"❌ Failed to save image: {e}")
        
        # 保存元数据
        metadata = {
            'timestamp': detection_result.get('timestamp', rospy.Time.now().to_sec()),
            'detection_time': datetime.now().isoformat(),
            'detected': detection_result.get('detected', False),
            'confidence': detection_result.get('confidence', 0.0),
            'point_3d_robot': detection_result.get('point_3d_robot'),
            'point_3d_camera': detection_result.get('point_3d_camera'),
            'depth_mm': detection_result.get('depth_mm'),
            'depth_m': detection_result.get('depth_m'),
            'bbox': detection_result.get('bbox'),
            'center_pixel': detection_result.get('center_pixel'),
            'image_path': detection_result.get('image_path', ''),
            'model_type': 'ONNX'
        }
        
        # 保存为JSON
        json_path = os.path.join(self.result_dir, f"{timestamp}.json")
        try:
            with open(json_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            rospy.loginfo(f"💾 Detection result saved: {json_path}")
            return json_path
        except Exception as e:
            rospy.logerr(f"❌ Failed to save JSON: {e}")
            return None
    
    def run_detection_loop(self, rate_hz=5):
        """运行检测循环（添加可视化异常保护）"""
        rate = rospy.Rate(rate_hz)
        
        while not rospy.is_shutdown():
            try:
                # 检测裂缝
                crack_result = self.detect_cracks()
                
                if crack_result.get('detected', False):
                    self.save_detection_result(crack_result, save_image=True)
                
                # 检测目标
                target_result = self.detect_target()
                
                if target_result is not None:
                    rospy.loginfo(f"🎯 Target detected at {target_result['center_2d']}")
                    
                    # 计算16个点
                    if target_result['center_3d_robot'] is not None:
                        points_3d = self.calculate_16_points(target_result['center_3d_robot'])
                        rospy.loginfo(f"📐 Calculated 16 points from center")
                        
                        # 这里可以发布到ROS话题或执行其他操作
                        # self.publish_target_points(points_3d)
                
                # 显示可视化窗口（添加异常保护）
                if self.visualization_enabled:
                    try:
                        if crack_result.get('detected', False):
                            cv2.imshow('Crack Detection', crack_result['annotated_image'])
                        
                        if target_result is not None:
                            cv2.imshow('Target Detection', target_result['annotated_image'])
                        
                        cv2.waitKey(1)
                    except Exception as e:
                        rospy.logwarn(f"⚠️ Visualization window error: {e}")
                
            except Exception as e:
                rospy.logerr(f"❌ Error in detection loop: {e}")
            
            rate.sleep()
        
        # 关闭窗口
        try:
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    rospy.init_node("vision_detector_node")
    
    # 获取参数
    detection_rate = rospy.get_param('~detection_rate', 5)
    visualization = rospy.get_param('~visualization', True)
    
    # 创建检测器
    detector = VisionDetector()
    detector.visualization_enabled = visualization
    
    rospy.loginfo(f"🚀 Starting vision detector with ONNX models")
    rospy.loginfo(f"   Detection rate: {detection_rate} Hz")
    rospy.loginfo(f"   Visualization: {visualization}")
    
    # 运行检测循环
    detector.run_detection_loop(rate_hz=detection_rate)
