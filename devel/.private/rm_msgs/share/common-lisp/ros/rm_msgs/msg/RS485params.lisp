; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude RS485params.msg.html

(cl:defclass <RS485params> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (baudrate
    :reader baudrate
    :initarg :baudrate
    :type cl:integer
    :initform 0))
)

(cl:defclass RS485params (<RS485params>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RS485params>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RS485params)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<RS485params> is deprecated: use rm_msgs-msg:RS485params instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <RS485params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:mode-val is deprecated.  Use rm_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'baudrate-val :lambda-list '(m))
(cl:defmethod baudrate-val ((m <RS485params>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:baudrate-val is deprecated.  Use rm_msgs-msg:baudrate instead.")
  (baudrate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RS485params>) ostream)
  "Serializes a message object of type '<RS485params>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'baudrate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RS485params>) istream)
  "Deserializes a message object of type '<RS485params>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'baudrate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RS485params>)))
  "Returns string type for a message object of type '<RS485params>"
  "rm_msgs/RS485params")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RS485params)))
  "Returns string type for a message object of type 'RS485params"
  "rm_msgs/RS485params")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RS485params>)))
  "Returns md5sum for a message object of type '<RS485params>"
  "78bc9e6cebd1458721f892a614d5c54c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RS485params)))
  "Returns md5sum for a message object of type 'RS485params"
  "78bc9e6cebd1458721f892a614d5c54c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RS485params>)))
  "Returns full string definition for message of type '<RS485params>"
  (cl:format cl:nil "int32 mode~%int32 baudrate~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RS485params)))
  "Returns full string definition for message of type 'RS485params"
  (cl:format cl:nil "int32 mode~%int32 baudrate~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RS485params>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RS485params>))
  "Converts a ROS message object to a list"
  (cl:list 'RS485params
    (cl:cons ':mode (mode msg))
    (cl:cons ':baudrate (baudrate msg))
))
