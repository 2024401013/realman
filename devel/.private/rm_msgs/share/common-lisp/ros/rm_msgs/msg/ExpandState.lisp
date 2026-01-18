; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude ExpandState.msg.html

(cl:defclass <ExpandState> (roslisp-msg-protocol:ros-message)
  ((get_state
    :reader get_state
    :initarg :get_state
    :type cl:boolean
    :initform cl:nil)
   (pos
    :reader pos
    :initarg :pos
    :type cl:fixnum
    :initform 0)
   (err_flag
    :reader err_flag
    :initarg :err_flag
    :type cl:fixnum
    :initform 0)
   (en_flag
    :reader en_flag
    :initarg :en_flag
    :type cl:fixnum
    :initform 0)
   (current
    :reader current
    :initarg :current
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (joint_id
    :reader joint_id
    :initarg :joint_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ExpandState (<ExpandState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExpandState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExpandState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<ExpandState> is deprecated: use rm_msgs-msg:ExpandState instead.")))

(cl:ensure-generic-function 'get_state-val :lambda-list '(m))
(cl:defmethod get_state-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:get_state-val is deprecated.  Use rm_msgs-msg:get_state instead.")
  (get_state m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:pos-val is deprecated.  Use rm_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'err_flag-val :lambda-list '(m))
(cl:defmethod err_flag-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:err_flag-val is deprecated.  Use rm_msgs-msg:err_flag instead.")
  (err_flag m))

(cl:ensure-generic-function 'en_flag-val :lambda-list '(m))
(cl:defmethod en_flag-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:en_flag-val is deprecated.  Use rm_msgs-msg:en_flag instead.")
  (en_flag m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:current-val is deprecated.  Use rm_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:mode-val is deprecated.  Use rm_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'joint_id-val :lambda-list '(m))
(cl:defmethod joint_id-val ((m <ExpandState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:joint_id-val is deprecated.  Use rm_msgs-msg:joint_id instead.")
  (joint_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExpandState>) ostream)
  "Serializes a message object of type '<ExpandState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'get_state) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'err_flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'en_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'en_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExpandState>) istream)
  "Deserializes a message object of type '<ExpandState>"
    (cl:setf (cl:slot-value msg 'get_state) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'err_flag) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'en_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'en_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'joint_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExpandState>)))
  "Returns string type for a message object of type '<ExpandState>"
  "rm_msgs/ExpandState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExpandState)))
  "Returns string type for a message object of type 'ExpandState"
  "rm_msgs/ExpandState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExpandState>)))
  "Returns md5sum for a message object of type '<ExpandState>"
  "51186bb81aaa7ad5c549fa4631da2340")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExpandState)))
  "Returns md5sum for a message object of type 'ExpandState"
  "51186bb81aaa7ad5c549fa4631da2340")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExpandState>)))
  "Returns full string definition for message of type '<ExpandState>"
  (cl:format cl:nil "bool get_state~%int16 pos	    #扩展关节角度，单位度，精度 0.001°~%int16 err_flag	#驱动错误代码~%uint16 en_flag	#扩展关节使能状态~%uint16 current	#当前驱动电流，单位：mA，精度：1mA。~%byte mode	    #当前扩展关节状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动~%uint16 joint_id	#扩展关节ID~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExpandState)))
  "Returns full string definition for message of type 'ExpandState"
  (cl:format cl:nil "bool get_state~%int16 pos	    #扩展关节角度，单位度，精度 0.001°~%int16 err_flag	#驱动错误代码~%uint16 en_flag	#扩展关节使能状态~%uint16 current	#当前驱动电流，单位：mA，精度：1mA。~%byte mode	    #当前扩展关节状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动~%uint16 joint_id	#扩展关节ID~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExpandState>))
  (cl:+ 0
     1
     2
     2
     2
     2
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExpandState>))
  "Converts a ROS message object to a list"
  (cl:list 'ExpandState
    (cl:cons ':get_state (get_state msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':err_flag (err_flag msg))
    (cl:cons ':en_flag (en_flag msg))
    (cl:cons ':current (current msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':joint_id (joint_id msg))
))
