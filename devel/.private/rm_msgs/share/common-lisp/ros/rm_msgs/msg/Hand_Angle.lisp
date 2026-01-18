; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Hand_Angle.msg.html

(cl:defclass <Hand_Angle> (roslisp-msg-protocol:ros-message)
  ((hand_angle
    :reader hand_angle
    :initarg :hand_angle
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Hand_Angle (<Hand_Angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hand_Angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hand_Angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Hand_Angle> is deprecated: use rm_msgs-msg:Hand_Angle instead.")))

(cl:ensure-generic-function 'hand_angle-val :lambda-list '(m))
(cl:defmethod hand_angle-val ((m <Hand_Angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:hand_angle-val is deprecated.  Use rm_msgs-msg:hand_angle instead.")
  (hand_angle m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <Hand_Angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:block-val is deprecated.  Use rm_msgs-msg:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hand_Angle>) ostream)
  "Serializes a message object of type '<Hand_Angle>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'hand_angle))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hand_Angle>) istream)
  "Deserializes a message object of type '<Hand_Angle>"
  (cl:setf (cl:slot-value msg 'hand_angle) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'hand_angle)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hand_Angle>)))
  "Returns string type for a message object of type '<Hand_Angle>"
  "rm_msgs/Hand_Angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hand_Angle)))
  "Returns string type for a message object of type 'Hand_Angle"
  "rm_msgs/Hand_Angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hand_Angle>)))
  "Returns md5sum for a message object of type '<Hand_Angle>"
  "56a0667211866c2d94cc70494fe305c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hand_Angle)))
  "Returns md5sum for a message object of type 'Hand_Angle"
  "56a0667211866c2d94cc70494fe305c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hand_Angle>)))
  "Returns full string definition for message of type '<Hand_Angle>"
  (cl:format cl:nil "#设置灵巧手角度，灵巧手有 6 个自由度，从 1~~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转~%~%int16[6] hand_angle	#手指角度数组，范围：0~~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态~%bool block #true 表示阻塞模式，false 表示非阻塞模式。~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hand_Angle)))
  "Returns full string definition for message of type 'Hand_Angle"
  (cl:format cl:nil "#设置灵巧手角度，灵巧手有 6 个自由度，从 1~~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转~%~%int16[6] hand_angle	#手指角度数组，范围：0~~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态~%bool block #true 表示阻塞模式，false 表示非阻塞模式。~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hand_Angle>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'hand_angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hand_Angle>))
  "Converts a ROS message object to a list"
  (cl:list 'Hand_Angle
    (cl:cons ':hand_angle (hand_angle msg))
    (cl:cons ':block (block msg))
))
