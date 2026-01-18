; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Expand_Position.msg.html

(cl:defclass <Expand_Position> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Expand_Position (<Expand_Position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Expand_Position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Expand_Position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Expand_Position> is deprecated: use rm_msgs-msg:Expand_Position instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Expand_Position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:pos-val is deprecated.  Use rm_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Expand_Position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:speed-val is deprecated.  Use rm_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Expand_Position>) ostream)
  "Serializes a message object of type '<Expand_Position>"
  (cl:let* ((signed (cl:slot-value msg 'pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Expand_Position>) istream)
  "Deserializes a message object of type '<Expand_Position>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Expand_Position>)))
  "Returns string type for a message object of type '<Expand_Position>"
  "rm_msgs/Expand_Position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Expand_Position)))
  "Returns string type for a message object of type 'Expand_Position"
  "rm_msgs/Expand_Position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Expand_Position>)))
  "Returns md5sum for a message object of type '<Expand_Position>"
  "464e5f6ba74ca32e53b631f4eeffd5ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Expand_Position)))
  "Returns md5sum for a message object of type 'Expand_Position"
  "464e5f6ba74ca32e53b631f4eeffd5ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Expand_Position>)))
  "Returns full string definition for message of type '<Expand_Position>"
  (cl:format cl:nil "int16 pos        #扩展关节位置，单位 ~%int16 speed         #速度百分比，1~~100~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Expand_Position)))
  "Returns full string definition for message of type 'Expand_Position"
  (cl:format cl:nil "int16 pos        #扩展关节位置，单位 ~%int16 speed         #速度百分比，1~~100~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Expand_Position>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Expand_Position>))
  "Converts a ROS message object to a list"
  (cl:list 'Expand_Position
    (cl:cons ':pos (pos msg))
    (cl:cons ':speed (speed msg))
))
