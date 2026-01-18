; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Jointversion.msg.html

(cl:defclass <Jointversion> (roslisp-msg-protocol:ros-message)
  ((joint_version
    :reader joint_version
    :initarg :joint_version
    :type (cl:vector cl:string)
   :initform (cl:make-array 7 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Jointversion (<Jointversion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Jointversion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Jointversion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Jointversion> is deprecated: use rm_msgs-msg:Jointversion instead.")))

(cl:ensure-generic-function 'joint_version-val :lambda-list '(m))
(cl:defmethod joint_version-val ((m <Jointversion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:joint_version-val is deprecated.  Use rm_msgs-msg:joint_version instead.")
  (joint_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Jointversion>) ostream)
  "Serializes a message object of type '<Jointversion>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Jointversion>) istream)
  "Deserializes a message object of type '<Jointversion>"
  (cl:setf (cl:slot-value msg 'joint_version) (cl:make-array 7))
  (cl:let ((vals (cl:slot-value msg 'joint_version)))
    (cl:dotimes (i 7)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Jointversion>)))
  "Returns string type for a message object of type '<Jointversion>"
  "rm_msgs/Jointversion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Jointversion)))
  "Returns string type for a message object of type 'Jointversion"
  "rm_msgs/Jointversion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Jointversion>)))
  "Returns md5sum for a message object of type '<Jointversion>"
  "34fe8048af36c99d06616a2279b77cd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Jointversion)))
  "Returns md5sum for a message object of type 'Jointversion"
  "34fe8048af36c99d06616a2279b77cd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Jointversion>)))
  "Returns full string definition for message of type '<Jointversion>"
  (cl:format cl:nil "string[7] joint_version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Jointversion)))
  "Returns full string definition for message of type 'Jointversion"
  (cl:format cl:nil "string[7] joint_version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Jointversion>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_version) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Jointversion>))
  "Converts a ROS message object to a list"
  (cl:list 'Jointversion
    (cl:cons ':joint_version (joint_version msg))
))
