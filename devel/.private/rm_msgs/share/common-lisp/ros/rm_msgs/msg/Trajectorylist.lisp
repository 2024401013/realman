; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Trajectorylist.msg.html

(cl:defclass <Trajectorylist> (roslisp-msg-protocol:ros-message)
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
   (total_size
    :reader total_size
    :initarg :total_size
    :type cl:integer
    :initform 0)
   (vague_search
    :reader vague_search
    :initarg :vague_search
    :type cl:string
    :initform "")
   (tra_list
    :reader tra_list
    :initarg :tra_list
    :type (cl:vector rm_msgs-msg:Trajectoryinfo)
   :initform (cl:make-array 0 :element-type 'rm_msgs-msg:Trajectoryinfo :initial-element (cl:make-instance 'rm_msgs-msg:Trajectoryinfo))))
)

(cl:defclass Trajectorylist (<Trajectorylist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trajectorylist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trajectorylist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Trajectorylist> is deprecated: use rm_msgs-msg:Trajectorylist instead.")))

(cl:ensure-generic-function 'page_num-val :lambda-list '(m))
(cl:defmethod page_num-val ((m <Trajectorylist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:page_num-val is deprecated.  Use rm_msgs-msg:page_num instead.")
  (page_num m))

(cl:ensure-generic-function 'page_size-val :lambda-list '(m))
(cl:defmethod page_size-val ((m <Trajectorylist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:page_size-val is deprecated.  Use rm_msgs-msg:page_size instead.")
  (page_size m))

(cl:ensure-generic-function 'total_size-val :lambda-list '(m))
(cl:defmethod total_size-val ((m <Trajectorylist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:total_size-val is deprecated.  Use rm_msgs-msg:total_size instead.")
  (total_size m))

(cl:ensure-generic-function 'vague_search-val :lambda-list '(m))
(cl:defmethod vague_search-val ((m <Trajectorylist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:vague_search-val is deprecated.  Use rm_msgs-msg:vague_search instead.")
  (vague_search m))

(cl:ensure-generic-function 'tra_list-val :lambda-list '(m))
(cl:defmethod tra_list-val ((m <Trajectorylist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:tra_list-val is deprecated.  Use rm_msgs-msg:tra_list instead.")
  (tra_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trajectorylist>) ostream)
  "Serializes a message object of type '<Trajectorylist>"
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
  (cl:let* ((signed (cl:slot-value msg 'total_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tra_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tra_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trajectorylist>) istream)
  "Deserializes a message object of type '<Trajectorylist>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vague_search) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'vague_search) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tra_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tra_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rm_msgs-msg:Trajectoryinfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trajectorylist>)))
  "Returns string type for a message object of type '<Trajectorylist>"
  "rm_msgs/Trajectorylist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectorylist)))
  "Returns string type for a message object of type 'Trajectorylist"
  "rm_msgs/Trajectorylist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trajectorylist>)))
  "Returns md5sum for a message object of type '<Trajectorylist>"
  "8ffd0485c7441f6956165c1adeb616bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trajectorylist)))
  "Returns md5sum for a message object of type 'Trajectorylist"
  "8ffd0485c7441f6956165c1adeb616bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trajectorylist>)))
  "Returns full string definition for message of type '<Trajectorylist>"
  (cl:format cl:nil "int32 page_num      # 页码~%int32 page_size     # 每页大小~%int32 total_size    # 列表长度~%string vague_search  # 模糊搜索 ~%Trajectoryinfo[] tra_list  # 返回符合的轨迹列表~%~%================================================================================~%MSG: rm_msgs/Trajectoryinfo~%string name          #轨迹名称	~%string create_time   #创建时间~%int32 point_num      #轨迹点数量~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trajectorylist)))
  "Returns full string definition for message of type 'Trajectorylist"
  (cl:format cl:nil "int32 page_num      # 页码~%int32 page_size     # 每页大小~%int32 total_size    # 列表长度~%string vague_search  # 模糊搜索 ~%Trajectoryinfo[] tra_list  # 返回符合的轨迹列表~%~%================================================================================~%MSG: rm_msgs/Trajectoryinfo~%string name          #轨迹名称	~%string create_time   #创建时间~%int32 point_num      #轨迹点数量~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trajectorylist>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'vague_search))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tra_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trajectorylist>))
  "Converts a ROS message object to a list"
  (cl:list 'Trajectorylist
    (cl:cons ':page_num (page_num msg))
    (cl:cons ':page_size (page_size msg))
    (cl:cons ':total_size (total_size msg))
    (cl:cons ':vague_search (vague_search msg))
    (cl:cons ':tra_list (tra_list msg))
))
