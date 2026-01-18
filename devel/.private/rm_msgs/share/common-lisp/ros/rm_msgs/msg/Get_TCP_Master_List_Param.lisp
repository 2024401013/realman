; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Get_TCP_Master_List_Param.msg.html

(cl:defclass <Get_TCP_Master_List_Param> (roslisp-msg-protocol:ros-message)
  ((page_num
    :reader page_num
    :initarg :page_num
    :type cl:integer
    :initform 0)
   (page_size
    :reader page_size
    :initarg :page_size
    :type cl:integer
    :initform 0)
   (vague_search
    :reader vague_search
    :initarg :vague_search
    :type cl:string
    :initform ""))
)

(cl:defclass Get_TCP_Master_List_Param (<Get_TCP_Master_List_Param>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_TCP_Master_List_Param>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_TCP_Master_List_Param)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Get_TCP_Master_List_Param> is deprecated: use rm_msgs-msg:Get_TCP_Master_List_Param instead.")))

(cl:ensure-generic-function 'page_num-val :lambda-list '(m))
(cl:defmethod page_num-val ((m <Get_TCP_Master_List_Param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:page_num-val is deprecated.  Use rm_msgs-msg:page_num instead.")
  (page_num m))

(cl:ensure-generic-function 'page_size-val :lambda-list '(m))
(cl:defmethod page_size-val ((m <Get_TCP_Master_List_Param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:page_size-val is deprecated.  Use rm_msgs-msg:page_size instead.")
  (page_size m))

(cl:ensure-generic-function 'vague_search-val :lambda-list '(m))
(cl:defmethod vague_search-val ((m <Get_TCP_Master_List_Param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:vague_search-val is deprecated.  Use rm_msgs-msg:vague_search instead.")
  (vague_search m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_TCP_Master_List_Param>) ostream)
  "Serializes a message object of type '<Get_TCP_Master_List_Param>"
  (cl:let* ((signed (cl:slot-value msg 'page_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'page_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'vague_search))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'vague_search))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_TCP_Master_List_Param>) istream)
  "Deserializes a message object of type '<Get_TCP_Master_List_Param>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'page_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'page_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vague_search) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'vague_search) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_TCP_Master_List_Param>)))
  "Returns string type for a message object of type '<Get_TCP_Master_List_Param>"
  "rm_msgs/Get_TCP_Master_List_Param")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_TCP_Master_List_Param)))
  "Returns string type for a message object of type 'Get_TCP_Master_List_Param"
  "rm_msgs/Get_TCP_Master_List_Param")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_TCP_Master_List_Param>)))
  "Returns md5sum for a message object of type '<Get_TCP_Master_List_Param>"
  "cbe34dfd8b66421bedf70042e463f5a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_TCP_Master_List_Param)))
  "Returns md5sum for a message object of type 'Get_TCP_Master_List_Param"
  "cbe34dfd8b66421bedf70042e463f5a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_TCP_Master_List_Param>)))
  "Returns full string definition for message of type '<Get_TCP_Master_List_Param>"
  (cl:format cl:nil "int32 page_num       # 页码~%int32 page_size      # 每页大小~%string vague_search  # 模糊搜索	~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_TCP_Master_List_Param)))
  "Returns full string definition for message of type 'Get_TCP_Master_List_Param"
  (cl:format cl:nil "int32 page_num       # 页码~%int32 page_size      # 每页大小~%string vague_search  # 模糊搜索	~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_TCP_Master_List_Param>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'vague_search))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_TCP_Master_List_Param>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_TCP_Master_List_Param
    (cl:cons ':page_num (page_num msg))
    (cl:cons ':page_size (page_size msg))
    (cl:cons ':vague_search (vague_search msg))
))
