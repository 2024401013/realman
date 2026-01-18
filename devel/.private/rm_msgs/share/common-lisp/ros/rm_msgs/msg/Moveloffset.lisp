; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Moveloffset.msg.html

(cl:defclass <Moveloffset> (roslisp-msg-protocol:ros-message)
  ((offset
    :reader offset
    :initarg :offset
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0)
   (r
    :reader r
    :initarg :r
    :type cl:integer
    :initform 0)
   (trajectory_connect
    :reader trajectory_connect
    :initarg :trajectory_connect
    :type cl:boolean
    :initform cl:nil)
   (frame_type
    :reader frame_type
    :initarg :frame_type
    :type cl:boolean
    :initform cl:nil)
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Moveloffset (<Moveloffset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Moveloffset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Moveloffset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Moveloffset> is deprecated: use rm_msgs-msg:Moveloffset instead.")))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:offset-val is deprecated.  Use rm_msgs-msg:offset instead.")
  (offset m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:speed-val is deprecated.  Use rm_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:r-val is deprecated.  Use rm_msgs-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'trajectory_connect-val :lambda-list '(m))
(cl:defmethod trajectory_connect-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:trajectory_connect-val is deprecated.  Use rm_msgs-msg:trajectory_connect instead.")
  (trajectory_connect m))

(cl:ensure-generic-function 'frame_type-val :lambda-list '(m))
(cl:defmethod frame_type-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:frame_type-val is deprecated.  Use rm_msgs-msg:frame_type instead.")
  (frame_type m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <Moveloffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:block-val is deprecated.  Use rm_msgs-msg:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Moveloffset>) ostream)
  "Serializes a message object of type '<Moveloffset>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'offset) ostream)
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'r)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trajectory_connect) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'frame_type) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Moveloffset>) istream)
  "Deserializes a message object of type '<Moveloffset>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'offset) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'r) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'trajectory_connect) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'frame_type) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Moveloffset>)))
  "Returns string type for a message object of type '<Moveloffset>"
  "rm_msgs/Moveloffset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Moveloffset)))
  "Returns string type for a message object of type 'Moveloffset"
  "rm_msgs/Moveloffset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Moveloffset>)))
  "Returns md5sum for a message object of type '<Moveloffset>"
  "af2efa1aef4c53564271cdf8a17ca144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Moveloffset)))
  "Returns md5sum for a message object of type 'Moveloffset"
  "af2efa1aef4c53564271cdf8a17ca144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Moveloffset>)))
  "Returns full string definition for message of type '<Moveloffset>"
  (cl:format cl:nil "geometry_msgs/Pose offset~%int32 speed~%int32 r~%bool trajectory_connect~%bool frame_type~%bool block~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Moveloffset)))
  "Returns full string definition for message of type 'Moveloffset"
  (cl:format cl:nil "geometry_msgs/Pose offset~%int32 speed~%int32 r~%bool trajectory_connect~%bool frame_type~%bool block~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Moveloffset>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'offset))
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Moveloffset>))
  "Converts a ROS message object to a list"
  (cl:list 'Moveloffset
    (cl:cons ':offset (offset msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':r (r msg))
    (cl:cons ':trajectory_connect (trajectory_connect msg))
    (cl:cons ':frame_type (frame_type msg))
    (cl:cons ':block (block msg))
))
