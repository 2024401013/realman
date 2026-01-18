; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Expand_Speed.msg.html

(cl:defclass <Expand_Speed> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Expand_Speed (<Expand_Speed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Expand_Speed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Expand_Speed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Expand_Speed> is deprecated: use rm_msgs-msg:Expand_Speed instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Expand_Speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:speed-val is deprecated.  Use rm_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Expand_Speed>) ostream)
  "Serializes a message object of type '<Expand_Speed>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Expand_Speed>) istream)
  "Deserializes a message object of type '<Expand_Speed>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Expand_Speed>)))
  "Returns string type for a message object of type '<Expand_Speed>"
  "rm_msgs/Expand_Speed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Expand_Speed)))
  "Returns string type for a message object of type 'Expand_Speed"
  "rm_msgs/Expand_Speed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Expand_Speed>)))
  "Returns md5sum for a message object of type '<Expand_Speed>"
  "368a599b530468ee137b04eea511a3ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Expand_Speed)))
  "Returns md5sum for a message object of type 'Expand_Speed"
  "368a599b530468ee137b04eea511a3ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Expand_Speed>)))
  "Returns full string definition for message of type '<Expand_Speed>"
  (cl:format cl:nil "int16 speed         #扩展关节速度环控制速度百分比，-100~~100~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Expand_Speed)))
  "Returns full string definition for message of type 'Expand_Speed"
  (cl:format cl:nil "int16 speed         #扩展关节速度环控制速度百分比，-100~~100~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Expand_Speed>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Expand_Speed>))
  "Converts a ROS message object to a list"
  (cl:list 'Expand_Speed
    (cl:cons ':speed (speed msg))
))
