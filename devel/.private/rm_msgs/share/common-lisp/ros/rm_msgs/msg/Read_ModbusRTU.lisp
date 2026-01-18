; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Read_ModbusRTU.msg.html

(cl:defclass <Read_ModbusRTU> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0)
   (device
    :reader device
    :initarg :device
    :type cl:integer
    :initform 0)
   (num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass Read_ModbusRTU (<Read_ModbusRTU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Read_ModbusRTU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Read_ModbusRTU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Read_ModbusRTU> is deprecated: use rm_msgs-msg:Read_ModbusRTU instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <Read_ModbusRTU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:address-val is deprecated.  Use rm_msgs-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'device-val :lambda-list '(m))
(cl:defmethod device-val ((m <Read_ModbusRTU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:device-val is deprecated.  Use rm_msgs-msg:device instead.")
  (device m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <Read_ModbusRTU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:num-val is deprecated.  Use rm_msgs-msg:num instead.")
  (num m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Read_ModbusRTU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:type-val is deprecated.  Use rm_msgs-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Read_ModbusRTU>) ostream)
  "Serializes a message object of type '<Read_ModbusRTU>"
  (cl:let* ((signed (cl:slot-value msg 'address)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'device)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Read_ModbusRTU>) istream)
  "Deserializes a message object of type '<Read_ModbusRTU>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'address) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'device) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Read_ModbusRTU>)))
  "Returns string type for a message object of type '<Read_ModbusRTU>"
  "rm_msgs/Read_ModbusRTU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Read_ModbusRTU)))
  "Returns string type for a message object of type 'Read_ModbusRTU"
  "rm_msgs/Read_ModbusRTU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Read_ModbusRTU>)))
  "Returns md5sum for a message object of type '<Read_ModbusRTU>"
  "58a66370687b1c7bdeac52f168ac52f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Read_ModbusRTU)))
  "Returns md5sum for a message object of type 'Read_ModbusRTU"
  "58a66370687b1c7bdeac52f168ac52f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Read_ModbusRTU>)))
  "Returns full string definition for message of type '<Read_ModbusRTU>"
  (cl:format cl:nil "int32 address #线圈起始地址。~%int32 device #外设设备地址。~%int32 num #线圈数量。~%int32 type #0-控制器端modbus主机；1-工具端modbus主机。~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Read_ModbusRTU)))
  "Returns full string definition for message of type 'Read_ModbusRTU"
  (cl:format cl:nil "int32 address #线圈起始地址。~%int32 device #外设设备地址。~%int32 num #线圈数量。~%int32 type #0-控制器端modbus主机；1-工具端modbus主机。~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Read_ModbusRTU>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Read_ModbusRTU>))
  "Converts a ROS message object to a list"
  (cl:list 'Read_ModbusRTU
    (cl:cons ':address (address msg))
    (cl:cons ':device (device msg))
    (cl:cons ':num (num msg))
    (cl:cons ':type (type msg))
))
