; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Modbusreaddata.msg.html

(cl:defclass <Modbusreaddata> (roslisp-msg-protocol:ros-message)
  ((read_data
    :reader read_data
    :initarg :read_data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Modbusreaddata (<Modbusreaddata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Modbusreaddata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Modbusreaddata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Modbusreaddata> is deprecated: use rm_msgs-msg:Modbusreaddata instead.")))

(cl:ensure-generic-function 'read_data-val :lambda-list '(m))
(cl:defmethod read_data-val ((m <Modbusreaddata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:read_data-val is deprecated.  Use rm_msgs-msg:read_data instead.")
  (read_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Modbusreaddata>) ostream)
  "Serializes a message object of type '<Modbusreaddata>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'read_data))))
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
   (cl:slot-value msg 'read_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Modbusreaddata>) istream)
  "Deserializes a message object of type '<Modbusreaddata>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'read_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'read_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Modbusreaddata>)))
  "Returns string type for a message object of type '<Modbusreaddata>"
  "rm_msgs/Modbusreaddata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Modbusreaddata)))
  "Returns string type for a message object of type 'Modbusreaddata"
  "rm_msgs/Modbusreaddata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Modbusreaddata>)))
  "Returns md5sum for a message object of type '<Modbusreaddata>"
  "85133f00e9641b3d842dc178852ca264")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Modbusreaddata)))
  "Returns md5sum for a message object of type 'Modbusreaddata"
  "85133f00e9641b3d842dc178852ca264")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Modbusreaddata>)))
  "Returns full string definition for message of type '<Modbusreaddata>"
  (cl:format cl:nil "int32[] read_data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Modbusreaddata)))
  "Returns full string definition for message of type 'Modbusreaddata"
  (cl:format cl:nil "int32[] read_data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Modbusreaddata>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'read_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Modbusreaddata>))
  "Converts a ROS message object to a list"
  (cl:list 'Modbusreaddata
    (cl:cons ':read_data (read_data msg))
))
