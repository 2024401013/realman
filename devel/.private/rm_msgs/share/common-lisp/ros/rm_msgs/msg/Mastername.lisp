; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Mastername.msg.html

(cl:defclass <Mastername> (roslisp-msg-protocol:ros-message)
  ((master_name
    :reader master_name
    :initarg :master_name
    :type cl:string
    :initform ""))
)

(cl:defclass Mastername (<Mastername>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mastername>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mastername)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Mastername> is deprecated: use rm_msgs-msg:Mastername instead.")))

(cl:ensure-generic-function 'master_name-val :lambda-list '(m))
(cl:defmethod master_name-val ((m <Mastername>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:master_name-val is deprecated.  Use rm_msgs-msg:master_name instead.")
  (master_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mastername>) ostream)
  "Serializes a message object of type '<Mastername>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'master_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'master_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mastername>) istream)
  "Deserializes a message object of type '<Mastername>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'master_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'master_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mastername>)))
  "Returns string type for a message object of type '<Mastername>"
  "rm_msgs/Mastername")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mastername)))
  "Returns string type for a message object of type 'Mastername"
  "rm_msgs/Mastername")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mastername>)))
  "Returns md5sum for a message object of type '<Mastername>"
  "85cc930e49a075f3a896b60d7598ba17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mastername)))
  "Returns md5sum for a message object of type 'Mastername"
  "85cc930e49a075f3a896b60d7598ba17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mastername>)))
  "Returns full string definition for message of type '<Mastername>"
  (cl:format cl:nil "string master_name~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mastername)))
  "Returns full string definition for message of type 'Mastername"
  (cl:format cl:nil "string master_name~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mastername>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'master_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mastername>))
  "Converts a ROS message object to a list"
  (cl:list 'Mastername
    (cl:cons ':master_name (master_name msg))
))
