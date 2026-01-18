; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Arm_Software_Version.msg.html

(cl:defclass <Arm_Software_Version> (roslisp-msg-protocol:ros-message)
  ((product_version
    :reader product_version
    :initarg :product_version
    :type cl:string
    :initform "")
   (algorithm_info
    :reader algorithm_info
    :initarg :algorithm_info
    :type cl:string
    :initform "")
   (ctrl_info
    :reader ctrl_info
    :initarg :ctrl_info
    :type rm_msgs-msg:Softwarebuildinfo
    :initform (cl:make-instance 'rm_msgs-msg:Softwarebuildinfo))
   (dynamic_info
    :reader dynamic_info
    :initarg :dynamic_info
    :type cl:string
    :initform "")
   (plan_info
    :reader plan_info
    :initarg :plan_info
    :type rm_msgs-msg:Softwarebuildinfo
    :initform (cl:make-instance 'rm_msgs-msg:Softwarebuildinfo))
   (controller_version
    :reader controller_version
    :initarg :controller_version
    :type cl:string
    :initform "")
   (com_info
    :reader com_info
    :initarg :com_info
    :type rm_msgs-msg:Softwarebuildinfo
    :initform (cl:make-instance 'rm_msgs-msg:Softwarebuildinfo))
   (program_info
    :reader program_info
    :initarg :program_info
    :type rm_msgs-msg:Softwarebuildinfo
    :initform (cl:make-instance 'rm_msgs-msg:Softwarebuildinfo)))
)

(cl:defclass Arm_Software_Version (<Arm_Software_Version>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Arm_Software_Version>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Arm_Software_Version)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Arm_Software_Version> is deprecated: use rm_msgs-msg:Arm_Software_Version instead.")))

(cl:ensure-generic-function 'product_version-val :lambda-list '(m))
(cl:defmethod product_version-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:product_version-val is deprecated.  Use rm_msgs-msg:product_version instead.")
  (product_version m))

(cl:ensure-generic-function 'algorithm_info-val :lambda-list '(m))
(cl:defmethod algorithm_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:algorithm_info-val is deprecated.  Use rm_msgs-msg:algorithm_info instead.")
  (algorithm_info m))

(cl:ensure-generic-function 'ctrl_info-val :lambda-list '(m))
(cl:defmethod ctrl_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:ctrl_info-val is deprecated.  Use rm_msgs-msg:ctrl_info instead.")
  (ctrl_info m))

(cl:ensure-generic-function 'dynamic_info-val :lambda-list '(m))
(cl:defmethod dynamic_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:dynamic_info-val is deprecated.  Use rm_msgs-msg:dynamic_info instead.")
  (dynamic_info m))

(cl:ensure-generic-function 'plan_info-val :lambda-list '(m))
(cl:defmethod plan_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:plan_info-val is deprecated.  Use rm_msgs-msg:plan_info instead.")
  (plan_info m))

(cl:ensure-generic-function 'controller_version-val :lambda-list '(m))
(cl:defmethod controller_version-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:controller_version-val is deprecated.  Use rm_msgs-msg:controller_version instead.")
  (controller_version m))

(cl:ensure-generic-function 'com_info-val :lambda-list '(m))
(cl:defmethod com_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:com_info-val is deprecated.  Use rm_msgs-msg:com_info instead.")
  (com_info m))

(cl:ensure-generic-function 'program_info-val :lambda-list '(m))
(cl:defmethod program_info-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:program_info-val is deprecated.  Use rm_msgs-msg:program_info instead.")
  (program_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Arm_Software_Version>) ostream)
  "Serializes a message object of type '<Arm_Software_Version>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'product_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'product_version))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'algorithm_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'algorithm_info))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ctrl_info) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dynamic_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dynamic_info))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan_info) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller_version))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'com_info) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'program_info) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Arm_Software_Version>) istream)
  "Deserializes a message object of type '<Arm_Software_Version>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'product_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'product_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'algorithm_info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'algorithm_info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ctrl_info) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dynamic_info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dynamic_info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan_info) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'com_info) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'program_info) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Arm_Software_Version>)))
  "Returns string type for a message object of type '<Arm_Software_Version>"
  "rm_msgs/Arm_Software_Version")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Arm_Software_Version)))
  "Returns string type for a message object of type 'Arm_Software_Version"
  "rm_msgs/Arm_Software_Version")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Arm_Software_Version>)))
  "Returns md5sum for a message object of type '<Arm_Software_Version>"
  "f90eb6551bbde202ac1756e5a3ee2f6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Arm_Software_Version)))
  "Returns md5sum for a message object of type 'Arm_Software_Version"
  "f90eb6551bbde202ac1756e5a3ee2f6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Arm_Software_Version>)))
  "Returns full string definition for message of type '<Arm_Software_Version>"
  (cl:format cl:nil "string product_version~%string algorithm_info~%Softwarebuildinfo ctrl_info~%string dynamic_info #3~%Softwarebuildinfo plan_info #3~%string controller_version #4~%Softwarebuildinfo com_info #4~%Softwarebuildinfo program_info #4~%================================================================================~%MSG: rm_msgs/Softwarebuildinfo~%string build_time~%string version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Arm_Software_Version)))
  "Returns full string definition for message of type 'Arm_Software_Version"
  (cl:format cl:nil "string product_version~%string algorithm_info~%Softwarebuildinfo ctrl_info~%string dynamic_info #3~%Softwarebuildinfo plan_info #3~%string controller_version #4~%Softwarebuildinfo com_info #4~%Softwarebuildinfo program_info #4~%================================================================================~%MSG: rm_msgs/Softwarebuildinfo~%string build_time~%string version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Arm_Software_Version>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'product_version))
     4 (cl:length (cl:slot-value msg 'algorithm_info))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ctrl_info))
     4 (cl:length (cl:slot-value msg 'dynamic_info))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan_info))
     4 (cl:length (cl:slot-value msg 'controller_version))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'com_info))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'program_info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Arm_Software_Version>))
  "Converts a ROS message object to a list"
  (cl:list 'Arm_Software_Version
    (cl:cons ':product_version (product_version msg))
    (cl:cons ':algorithm_info (algorithm_info msg))
    (cl:cons ':ctrl_info (ctrl_info msg))
    (cl:cons ':dynamic_info (dynamic_info msg))
    (cl:cons ':plan_info (plan_info msg))
    (cl:cons ':controller_version (controller_version msg))
    (cl:cons ':com_info (com_info msg))
    (cl:cons ':program_info (program_info msg))
))
