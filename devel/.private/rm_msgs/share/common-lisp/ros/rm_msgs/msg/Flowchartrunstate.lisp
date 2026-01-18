; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Flowchartrunstate.msg.html

(cl:defclass <Flowchartrunstate> (roslisp-msg-protocol:ros-message)
  ((run_state
    :reader run_state
    :initarg :run_state
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (plan_speed
    :reader plan_speed
    :initarg :plan_speed
    :type cl:fixnum
    :initform 0)
   (step_mode
    :reader step_mode
    :initarg :step_mode
    :type cl:fixnum
    :initform 0)
   (modal_id
    :reader modal_id
    :initarg :modal_id
    :type cl:string
    :initform ""))
)

(cl:defclass Flowchartrunstate (<Flowchartrunstate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Flowchartrunstate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Flowchartrunstate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Flowchartrunstate> is deprecated: use rm_msgs-msg:Flowchartrunstate instead.")))

(cl:ensure-generic-function 'run_state-val :lambda-list '(m))
(cl:defmethod run_state-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:run_state-val is deprecated.  Use rm_msgs-msg:run_state instead.")
  (run_state m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:id-val is deprecated.  Use rm_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:name-val is deprecated.  Use rm_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'plan_speed-val :lambda-list '(m))
(cl:defmethod plan_speed-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:plan_speed-val is deprecated.  Use rm_msgs-msg:plan_speed instead.")
  (plan_speed m))

(cl:ensure-generic-function 'step_mode-val :lambda-list '(m))
(cl:defmethod step_mode-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:step_mode-val is deprecated.  Use rm_msgs-msg:step_mode instead.")
  (step_mode m))

(cl:ensure-generic-function 'modal_id-val :lambda-list '(m))
(cl:defmethod modal_id-val ((m <Flowchartrunstate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:modal_id-val is deprecated.  Use rm_msgs-msg:modal_id instead.")
  (modal_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Flowchartrunstate>) ostream)
  "Serializes a message object of type '<Flowchartrunstate>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'run_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'plan_speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step_mode)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modal_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modal_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Flowchartrunstate>) istream)
  "Deserializes a message object of type '<Flowchartrunstate>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'run_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'plan_speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step_mode)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modal_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modal_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Flowchartrunstate>)))
  "Returns string type for a message object of type '<Flowchartrunstate>"
  "rm_msgs/Flowchartrunstate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Flowchartrunstate)))
  "Returns string type for a message object of type 'Flowchartrunstate"
  "rm_msgs/Flowchartrunstate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Flowchartrunstate>)))
  "Returns md5sum for a message object of type '<Flowchartrunstate>"
  "05f1a2e93a88cd1e5f9f80d8af8e987a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Flowchartrunstate)))
  "Returns md5sum for a message object of type 'Flowchartrunstate"
  "05f1a2e93a88cd1e5f9f80d8af8e987a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Flowchartrunstate>)))
  "Returns full string definition for message of type '<Flowchartrunstate>"
  (cl:format cl:nil "uint8 run_state  # 运行状态 0 未开始 1运行中 2暂停中~%uint8 id         # 当前使能的文件id。~%string name # 当前使能的文件名称。~%uint8 plan_speed     # 当前使能的文件全局规划速度比例 1-100。~%uint8 step_mode    # 单步模式，0为空，1为正常, 2为单步。~%string modal_id   # 运行到的流程图块的id。未运行则不返回~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Flowchartrunstate)))
  "Returns full string definition for message of type 'Flowchartrunstate"
  (cl:format cl:nil "uint8 run_state  # 运行状态 0 未开始 1运行中 2暂停中~%uint8 id         # 当前使能的文件id。~%string name # 当前使能的文件名称。~%uint8 plan_speed     # 当前使能的文件全局规划速度比例 1-100。~%uint8 step_mode    # 单步模式，0为空，1为正常, 2为单步。~%string modal_id   # 运行到的流程图块的id。未运行则不返回~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Flowchartrunstate>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'name))
     1
     1
     4 (cl:length (cl:slot-value msg 'modal_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Flowchartrunstate>))
  "Converts a ROS message object to a list"
  (cl:list 'Flowchartrunstate
    (cl:cons ':run_state (run_state msg))
    (cl:cons ':id (id msg))
    (cl:cons ':name (name msg))
    (cl:cons ':plan_speed (plan_speed msg))
    (cl:cons ':step_mode (step_mode msg))
    (cl:cons ':modal_id (modal_id msg))
))
