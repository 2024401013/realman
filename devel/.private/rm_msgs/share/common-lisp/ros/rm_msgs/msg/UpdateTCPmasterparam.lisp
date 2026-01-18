; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude UpdateTCPmasterparam.msg.html

(cl:defclass <UpdateTCPmasterparam> (roslisp-msg-protocol:ros-message)
  ((master_name
    :reader master_name
    :initarg :master_name
    :type cl:string
    :initform "")
   (new_name
    :reader new_name
    :initarg :new_name
    :type cl:string
    :initform "")
   (ip
    :reader ip
    :initarg :ip
    :type cl:string
    :initform "")
   (port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0))
)

(cl:defclass UpdateTCPmasterparam (<UpdateTCPmasterparam>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateTCPmasterparam>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateTCPmasterparam)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<UpdateTCPmasterparam> is deprecated: use rm_msgs-msg:UpdateTCPmasterparam instead.")))

(cl:ensure-generic-function 'master_name-val :lambda-list '(m))
(cl:defmethod master_name-val ((m <UpdateTCPmasterparam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:master_name-val is deprecated.  Use rm_msgs-msg:master_name instead.")
  (master_name m))

(cl:ensure-generic-function 'new_name-val :lambda-list '(m))
(cl:defmethod new_name-val ((m <UpdateTCPmasterparam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:new_name-val is deprecated.  Use rm_msgs-msg:new_name instead.")
  (new_name m))

(cl:ensure-generic-function 'ip-val :lambda-list '(m))
(cl:defmethod ip-val ((m <UpdateTCPmasterparam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:ip-val is deprecated.  Use rm_msgs-msg:ip instead.")
  (ip m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <UpdateTCPmasterparam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:port-val is deprecated.  Use rm_msgs-msg:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateTCPmasterparam>) ostream)
  "Serializes a message object of type '<UpdateTCPmasterparam>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'master_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'master_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'new_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'new_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ip))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ip))
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateTCPmasterparam>) istream)
  "Deserializes a message object of type '<UpdateTCPmasterparam>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'master_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'master_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'new_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ip) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ip) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateTCPmasterparam>)))
  "Returns string type for a message object of type '<UpdateTCPmasterparam>"
  "rm_msgs/UpdateTCPmasterparam")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateTCPmasterparam)))
  "Returns string type for a message object of type 'UpdateTCPmasterparam"
  "rm_msgs/UpdateTCPmasterparam")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateTCPmasterparam>)))
  "Returns md5sum for a message object of type '<UpdateTCPmasterparam>"
  "76137ea31de5ca5b7a105efb1f13f4c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateTCPmasterparam)))
  "Returns md5sum for a message object of type 'UpdateTCPmasterparam"
  "76137ea31de5ca5b7a105efb1f13f4c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateTCPmasterparam>)))
  "Returns full string definition for message of type '<UpdateTCPmasterparam>"
  (cl:format cl:nil "string master_name #要修改的tcp主站名称~%string new_name #新名称~%string ip #新IP~%int32 port #新端口~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateTCPmasterparam)))
  "Returns full string definition for message of type 'UpdateTCPmasterparam"
  (cl:format cl:nil "string master_name #要修改的tcp主站名称~%string new_name #新名称~%string ip #新IP~%int32 port #新端口~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateTCPmasterparam>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'master_name))
     4 (cl:length (cl:slot-value msg 'new_name))
     4 (cl:length (cl:slot-value msg 'ip))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateTCPmasterparam>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateTCPmasterparam
    (cl:cons ':master_name (master_name msg))
    (cl:cons ':new_name (new_name msg))
    (cl:cons ':ip (ip msg))
    (cl:cons ':port (port msg))
))
