; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Trajectoryinfo.msg.html

(cl:defclass <Trajectoryinfo> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (create_time
    :reader create_time
    :initarg :create_time
    :type cl:string
    :initform "")
   (point_num
    :reader point_num
    :initarg :point_num
    :type cl:integer
    :initform 0))
)

(cl:defclass Trajectoryinfo (<Trajectoryinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trajectoryinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trajectoryinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Trajectoryinfo> is deprecated: use rm_msgs-msg:Trajectoryinfo instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Trajectoryinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:name-val is deprecated.  Use rm_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'create_time-val :lambda-list '(m))
(cl:defmethod create_time-val ((m <Trajectoryinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:create_time-val is deprecated.  Use rm_msgs-msg:create_time instead.")
  (create_time m))

(cl:ensure-generic-function 'point_num-val :lambda-list '(m))
(cl:defmethod point_num-val ((m <Trajectoryinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:point_num-val is deprecated.  Use rm_msgs-msg:point_num instead.")
  (point_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trajectoryinfo>) ostream)
  "Serializes a message object of type '<Trajectoryinfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'create_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'create_time))
  (cl:let* ((signed (cl:slot-value msg 'point_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trajectoryinfo>) istream)
  "Deserializes a message object of type '<Trajectoryinfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'create_time) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'create_time) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'point_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trajectoryinfo>)))
  "Returns string type for a message object of type '<Trajectoryinfo>"
  "rm_msgs/Trajectoryinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectoryinfo)))
  "Returns string type for a message object of type 'Trajectoryinfo"
  "rm_msgs/Trajectoryinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trajectoryinfo>)))
  "Returns md5sum for a message object of type '<Trajectoryinfo>"
  "e9fb2008a0ef07a81b5bc4d72b577068")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trajectoryinfo)))
  "Returns md5sum for a message object of type 'Trajectoryinfo"
  "e9fb2008a0ef07a81b5bc4d72b577068")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trajectoryinfo>)))
  "Returns full string definition for message of type '<Trajectoryinfo>"
  (cl:format cl:nil "string name          #轨迹名称	~%string create_time   #创建时间~%int32 point_num      #轨迹点数量~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trajectoryinfo)))
  "Returns full string definition for message of type 'Trajectoryinfo"
  (cl:format cl:nil "string name          #轨迹名称	~%string create_time   #创建时间~%int32 point_num      #轨迹点数量~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trajectoryinfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'create_time))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trajectoryinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'Trajectoryinfo
    (cl:cons ':name (name msg))
    (cl:cons ':create_time (create_time msg))
    (cl:cons ':point_num (point_num msg))
))
