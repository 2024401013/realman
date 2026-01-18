; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Softwarebuildinfo.msg.html

(cl:defclass <Softwarebuildinfo> (roslisp-msg-protocol:ros-message)
  ((build_time
    :reader build_time
    :initarg :build_time
    :type cl:string
    :initform "")
   (version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass Softwarebuildinfo (<Softwarebuildinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Softwarebuildinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Softwarebuildinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Softwarebuildinfo> is deprecated: use rm_msgs-msg:Softwarebuildinfo instead.")))

(cl:ensure-generic-function 'build_time-val :lambda-list '(m))
(cl:defmethod build_time-val ((m <Softwarebuildinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:build_time-val is deprecated.  Use rm_msgs-msg:build_time instead.")
  (build_time m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <Softwarebuildinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:version-val is deprecated.  Use rm_msgs-msg:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Softwarebuildinfo>) ostream)
  "Serializes a message object of type '<Softwarebuildinfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'build_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'build_time))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Softwarebuildinfo>) istream)
  "Deserializes a message object of type '<Softwarebuildinfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'build_time) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'build_time) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Softwarebuildinfo>)))
  "Returns string type for a message object of type '<Softwarebuildinfo>"
  "rm_msgs/Softwarebuildinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Softwarebuildinfo)))
  "Returns string type for a message object of type 'Softwarebuildinfo"
  "rm_msgs/Softwarebuildinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Softwarebuildinfo>)))
  "Returns md5sum for a message object of type '<Softwarebuildinfo>"
  "7075122606129dab433ee6c359a4c404")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Softwarebuildinfo)))
  "Returns md5sum for a message object of type 'Softwarebuildinfo"
  "7075122606129dab433ee6c359a4c404")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Softwarebuildinfo>)))
  "Returns full string definition for message of type '<Softwarebuildinfo>"
  (cl:format cl:nil "string build_time~%string version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Softwarebuildinfo)))
  "Returns full string definition for message of type 'Softwarebuildinfo"
  (cl:format cl:nil "string build_time~%string version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Softwarebuildinfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'build_time))
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Softwarebuildinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'Softwarebuildinfo
    (cl:cons ':build_time (build_time msg))
    (cl:cons ':version (version msg))
))
