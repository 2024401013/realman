; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Start_Multi_Drag_Teach.msg.html

(cl:defclass <Start_Multi_Drag_Teach> (roslisp-msg-protocol:ros-message)
  ((free_axes
    :reader free_axes
    :initarg :free_axes
    :type (cl:vector cl:integer)
   :initform (cl:make-array 6 :element-type 'cl:integer :initial-element 0))
   (frame
    :reader frame
    :initarg :frame
    :type cl:integer
    :initform 0)
   (singular_wall
    :reader singular_wall
    :initarg :singular_wall
    :type cl:integer
    :initform 0))
)

(cl:defclass Start_Multi_Drag_Teach (<Start_Multi_Drag_Teach>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Start_Multi_Drag_Teach>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Start_Multi_Drag_Teach)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Start_Multi_Drag_Teach> is deprecated: use rm_msgs-msg:Start_Multi_Drag_Teach instead.")))

(cl:ensure-generic-function 'free_axes-val :lambda-list '(m))
(cl:defmethod free_axes-val ((m <Start_Multi_Drag_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:free_axes-val is deprecated.  Use rm_msgs-msg:free_axes instead.")
  (free_axes m))

(cl:ensure-generic-function 'frame-val :lambda-list '(m))
(cl:defmethod frame-val ((m <Start_Multi_Drag_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:frame-val is deprecated.  Use rm_msgs-msg:frame instead.")
  (frame m))

(cl:ensure-generic-function 'singular_wall-val :lambda-list '(m))
(cl:defmethod singular_wall-val ((m <Start_Multi_Drag_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:singular_wall-val is deprecated.  Use rm_msgs-msg:singular_wall instead.")
  (singular_wall m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Start_Multi_Drag_Teach>) ostream)
  "Serializes a message object of type '<Start_Multi_Drag_Teach>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'free_axes))
  (cl:let* ((signed (cl:slot-value msg 'frame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'singular_wall)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Start_Multi_Drag_Teach>) istream)
  "Deserializes a message object of type '<Start_Multi_Drag_Teach>"
  (cl:setf (cl:slot-value msg 'free_axes) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'free_axes)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'singular_wall) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Start_Multi_Drag_Teach>)))
  "Returns string type for a message object of type '<Start_Multi_Drag_Teach>"
  "rm_msgs/Start_Multi_Drag_Teach")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Start_Multi_Drag_Teach)))
  "Returns string type for a message object of type 'Start_Multi_Drag_Teach"
  "rm_msgs/Start_Multi_Drag_Teach")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Start_Multi_Drag_Teach>)))
  "Returns md5sum for a message object of type '<Start_Multi_Drag_Teach>"
  "36a7e83e1c20d27bf9d63d9ab11797e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Start_Multi_Drag_Teach)))
  "Returns md5sum for a message object of type 'Start_Multi_Drag_Teach"
  "36a7e83e1c20d27bf9d63d9ab11797e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Start_Multi_Drag_Teach>)))
  "Returns full string definition for message of type '<Start_Multi_Drag_Teach>"
  (cl:format cl:nil "#uint8 mode~%int32[6] free_axes       # 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动~%int32 frame              # 参考坐标系，0-工作坐标系 1-工具坐标系。~%int32 singular_wall      # 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Start_Multi_Drag_Teach)))
  "Returns full string definition for message of type 'Start_Multi_Drag_Teach"
  (cl:format cl:nil "#uint8 mode~%int32[6] free_axes       # 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动~%int32 frame              # 参考坐标系，0-工作坐标系 1-工具坐标系。~%int32 singular_wall      # 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Start_Multi_Drag_Teach>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'free_axes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Start_Multi_Drag_Teach>))
  "Converts a ROS message object to a list"
  (cl:list 'Start_Multi_Drag_Teach
    (cl:cons ':free_axes (free_axes msg))
    (cl:cons ':frame (frame msg))
    (cl:cons ':singular_wall (singular_wall msg))
))
