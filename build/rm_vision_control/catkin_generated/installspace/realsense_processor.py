#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class RealSenseProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 图像数据
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅话题
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        
        # 发布话题（可选）
        self.depth_colormap_pub = rospy.Publisher('/camera/depth_colormap', Image, queue_size=1)
        
        rospy.loginfo("RealSenseProcessor initialized")
    
    def color_callback(self, msg):
        """RGB图像回调"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error in color callback: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            # 转换为CV图像，保持16位深度
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            
            # 可选：发布深度图的彩色可视化
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            colormap_msg = self.bridge.cv2_to_imgmsg(depth_colormap, 'bgr8')
            colormap_msg.header = msg.header
            self.depth_colormap_pub.publish(colormap_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error in depth callback: {e}")
    
    def camera_info_callback(self, msg):
        """相机内参回调"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera intrinsics received")
            rospy.loginfo(f"Camera matrix:\n{self.camera_matrix}")
    
    def pixel_to_3d_camera(self, u, v, depth):
        """像素坐标转换到相机坐标系下的3D点"""
        if self.camera_matrix is None or depth <= 0:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def pixel_to_3d_robot(self, u, v, depth, target_frame='base_link'):
        """像素坐标转换到机械臂基座坐标系下的3D点"""
        # 首先转换到相机坐标系
        point_camera = self.pixel_to_3d_camera(u, v, depth)
        if point_camera is None:
            return None
        
        # 创建PointStamped消息
        point_camera_msg = PointStamped()
        point_camera_msg.header.frame_id = 'camera_color_optical_frame'  # RealSense的坐标系
        point_camera_msg.header.stamp = rospy.Time.now()
        point_camera_msg.point.x = point_camera[0]
        point_camera_msg.point.y = point_camera[1]
        point_camera_msg.point.z = point_camera[2]
        
        try:
            # 转换到目标坐标系
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point_camera_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            point_robot_msg = tf2_geometry_msgs.do_transform_point(
                point_camera_msg, 
                transform
            )
            
            return np.array([
                point_robot_msg.point.x,
                point_robot_msg.point.y,
                point_robot_msg.point.z
            ])
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transformation failed: {e}")
            return None
    
    def get_image(self):
        """获取当前图像"""
        return self.color_image, self.depth_image
    
    def get_depth_at_point(self, u, v):
        """获取指定像素点的深度值"""
        if self.depth_image is None:
            return None
        
        # 检查坐标是否在图像范围内
        height, width = self.depth_image.shape[:2]
        if 0 <= u < width and 0 <= v < height:
            depth = self.depth_image[v, u]  # 注意：OpenCV是(row, col)，所以是(v, u)
            return depth
        return None
    
    def is_ready(self):
        """检查处理器是否准备就绪"""
        return (self.color_image is not None and 
                self.depth_image is not None and 
                self.camera_matrix is not None)