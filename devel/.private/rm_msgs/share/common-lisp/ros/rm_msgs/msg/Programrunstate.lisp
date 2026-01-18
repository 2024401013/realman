; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Programrunstate.msg.html

(cl:defclass <Programrunstate> (roslisp-msg-protocol:ros-message)
  ((run_state
    :reader run_state
    :initarg :run_state
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (edit_id
    :reader edit_id
    :initarg :edit_id
    :type cl:integer
    :initform 0)
   (plan_num
    :reader plan_num
    :initarg :plan_num
    :type cl:integer
    :initform 0)
   (total_loop
    :reader total_loop
    :initarg :total_loop
    :type cl:integer
    :initform 0)
   (step_mode
    :reader step_mode
    :initarg :step_mode
    :type cl:integer
    :initform 0)
   (plan_speed
    :reader plan_speed
    :initarg :plan_speed
    :type cl:integer
    :initform 0)
   (loop_num
    :reader loop_num
    :initarg :loop_num
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (loop_cont
    :reader loop_cont
    :initarg :loop_cont
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Programrunstate (<Programrunstate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Programrunstate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Programrunstate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Programrunstate> is deprecated: use rm_msgs-msg:Programrunstate instead.")))

(cl:ensure-generic-function 'run_state-val :lambda-list '(m))
(cl:defmethod run_state-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:run_state-val is deprecated.  Use rm_msgs-msg:run_state instead.")
  (run_state m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:id-val is deprecated.  Use rm_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'edit_id-val :lambda-list '(m))
(cl:defmethod edit_id-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:edit_id-val is deprecated.  Use rm_msgs-msg:edit_id instead.")
  (edit_id m))

(cl:ensure-generic-function 'plan_num-val :lambda-list '(m))
(cl:defmethod plan_num-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:plan_num-val is deprecated.  Use rm_msgs-msg:plan_num instead.")
  (plan_num m))

(cl:ensure-generic-function 'total_loop-val :lambda-list '(m))
(cl:defmethod total_loop-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:total_loop-val is deprecated.  Use rm_msgs-msg:total_loop instead.")
  (total_loop m))

(cl:ensure-generic-function 'step_mode-val :lambda-list '(m))
(cl:defmethod step_mode-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:step_mode-val is deprecated.  Use rm_msgs-msg:step_mode instead.")
  (step_mode m))

(cl:ensure-generic-function 'plan_speed-val :lambda-list '(m))
(cl:defmethod plan_speed-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:plan_speed-val is deprecated.  Use rm_msgs-msg:plan_speed instead.")
  (plan_speed m))

(cl:ensure-generic-function 'loop_num-val :lambda-list '(m))
(cl:defmethod loop_num-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:loop_num-val is deprecated.  Use rm_msgs-msg:loop_num instead.")
  (loop_num m))

(cl:ensure-generic-function 'loop_cont-val :lambda-list '(m))
(cl:defmethod loop_cont-val ((m <Programrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:loop_cont-val is deprecated.  Use rm_msgs-msg:loop_cont instead.")
  (loop_cont m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Programrunstate>) ostream)
  "Serializes a message object of type '<Programrunstate>"
  (cl:let* ((signed (cl:slot-value msg 'run_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'edit_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'plan_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'total_loop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'step_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'plan_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'loop_num))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'loop_num))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'loop_cont))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'loop_cont))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Programrunstate>) istream)
  "Deserializes a message object of type '<Programrunstate>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'run_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'edit_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_loop) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'loop_num) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'loop_num)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'loop_cont) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'loop_cont)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Programrunstate>)))
  "Returns string type for a message object of type '<Programrunstate>"
  "rm_msgs/Programrunstate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Programrunstate)))
  "Returns string type for a message object of type 'Programrunstate"
  "rm_msgs/Programrunstate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Programrunstate>)))
  "Returns md5sum for a message object of type '<Programrunstate>"
  "f985b60b566dc245fe01afe38654a230")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Programrunstate)))
  "Returns md5sum for a message object of type 'Programrunstate"
  "f985b60b566dc245fe01afe38654a230")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Programrunstate>)))
  "Returns full string definition for message of type '<Programrunstate>"
  (cl:format cl:nil "int32 run_state   # 运行状态 0 未开始 1运行中 2暂停中~%int32 id          # 运行轨迹编号~%int32 edit_id     # 上次编辑的在线编程编号 id~%int32 plan_num    # 运行行数~%int32 total_loop      # 循环指令数量~%int32 step_mode       # 单步模式，1 为单步模式，0 为非单步模式~%int32 plan_speed      # 全局规划速度比例 1-100~%int32[] loop_num        # 循环行数~%int32[] loop_cont       # 对应循环次数~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Programrunstate)))
  "Returns full string definition for message of type 'Programrunstate"
  (cl:format cl:nil "int32 run_state   # 运行状态 0 未开始 1运行中 2暂停中~%int32 id          # 运行轨迹编号~%int32 edit_id     # 上次编辑的在线编程编号 id~%int32 plan_num    # 运行行数~%int32 total_loop      # 循环指令数量~%int32 step_mode       # 单步模式，1 为单步模式，0 为非单步模式~%int32 plan_speed      # 全局规划速度比例 1-100~%int32[] loop_num        # 循环行数~%int32[] loop_cont       # 对应循环次数~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Programrunstate>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'loop_num) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'loop_cont) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Programrunstate>))
  "Converts a ROS message object to a list"
  (cl:list 'Programrunstate
    (cl:cons ':run_state (run_state msg))
    (cl:cons ':id (id msg))
    (cl:cons ':edit_id (edit_id msg))
    (cl:cons ':plan_num (plan_num msg))
    (cl:cons ':total_loop (total_loop msg))
    (cl:cons ':step_mode (step_mode msg))
    (cl:cons ':plan_speed (plan_speed msg))
    (cl:cons ':loop_num (loop_num msg))
    (cl:cons ':loop_cont (loop_cont msg))
))
