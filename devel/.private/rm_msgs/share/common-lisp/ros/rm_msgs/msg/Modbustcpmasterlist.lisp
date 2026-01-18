; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Modbustcpmasterlist.msg.html

(cl:defclass <Modbustcpmasterlist> (roslisp-msg-protocol:ros-message)
  ((master_list
    :reader master_list
    :initarg :master_list
    :type (cl:vector rm_msgs-msg:Modbustcpmasterinfo)
   :initform (cl:make-array 0 :element-type 'rm_msgs-msg:Modbustcpmasterinfo :initial-element (cl:make-instance 'rm_msgs-msg:Modbustcpmasterinfo))))
)

(cl:defclass Modbustcpmasterlist (<Modbustcpmasterlist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Modbustcpmasterlist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Modbustcpmasterlist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Modbustcpmasterlist> is deprecated: use rm_msgs-msg:Modbustcpmasterlist instead.")))

(cl:ensure-generic-function 'master_list-val :lambda-list '(m))
(cl:defmethod master_list-val ((m <Modbustcpmasterlist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:master_list-val is deprecated.  Use rm_msgs-msg:master_list instead.")
  (master_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Modbustcpmasterlist>) ostream)
  "Serializes a message object of type '<Modbustcpmasterlist>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'master_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'master_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Modbustcpmasterlist>) istream)
  "Deserializes a message object of type '<Modbustcpmasterlist>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'master_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'master_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rm_msgs-msg:Modbustcpmasterinfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Modbustcpmasterlist>)))
  "Returns string type for a message object of type '<Modbustcpmasterlist>"
  "rm_msgs/Modbustcpmasterlist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Modbustcpmasterlist)))
  "Returns string type for a message object of type 'Modbustcpmasterlist"
  "rm_msgs/Modbustcpmasterlist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Modbustcpmasterlist>)))
  "Returns md5sum for a message object of type '<Modbustcpmasterlist>"
  "4b8086f234a37bbe3de0d031d6dca80f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Modbustcpmasterlist)))
  "Returns md5sum for a message object of type 'Modbustcpmasterlist"
  "4b8086f234a37bbe3de0d031d6dca80f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Modbustcpmasterlist>)))
  "Returns full string definition for message of type '<Modbustcpmasterlist>"
  (cl:format cl:nil "Modbustcpmasterinfo[] master_list   # 返回符合的TCP主站列表~%~%================================================================================~%MSG: rm_msgs/Modbustcpmasterinfo~%string master_name # Modbus 主站名称，最大长度15个字符~%string ip          # TCP主站 IP 地址~%int32 port         # TCP主站端口号	~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Modbustcpmasterlist)))
  "Returns full string definition for message of type 'Modbustcpmasterlist"
  (cl:format cl:nil "Modbustcpmasterinfo[] master_list   # 返回符合的TCP主站列表~%~%================================================================================~%MSG: rm_msgs/Modbustcpmasterinfo~%string master_name # Modbus 主站名称，最大长度15个字符~%string ip          # TCP主站 IP 地址~%int32 port         # TCP主站端口号	~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Modbustcpmasterlist>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'master_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Modbustcpmasterlist>))
  "Converts a ROS message object to a list"
  (cl:list 'Modbustcpmasterlist
    (cl:cons ':master_list (master_list msg))
))
