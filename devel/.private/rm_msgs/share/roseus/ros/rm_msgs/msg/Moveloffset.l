;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Moveloffset)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Moveloffset (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::MOVELOFFSET")
  (make-package "RM_MSGS::MOVELOFFSET"))

(in-package "ROS")
;;//! \htmlinclude Moveloffset.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rm_msgs::Moveloffset
  :super ros::object
  :slots (_offset _speed _r _trajectory_connect _frame_type _block ))

(defmethod rm_msgs::Moveloffset
  (:init
   (&key
    ((:offset __offset) (instance geometry_msgs::Pose :init))
    ((:speed __speed) 0)
    ((:r __r) 0)
    ((:trajectory_connect __trajectory_connect) nil)
    ((:frame_type __frame_type) nil)
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _offset __offset)
   (setq _speed (round __speed))
   (setq _r (round __r))
   (setq _trajectory_connect __trajectory_connect)
   (setq _frame_type __frame_type)
   (setq _block __block)
   self)
  (:offset
   (&rest __offset)
   (if (keywordp (car __offset))
       (send* _offset __offset)
     (progn
       (if __offset (setq _offset (car __offset)))
       _offset)))
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:trajectory_connect
   (&optional (__trajectory_connect :null))
   (if (not (eq __trajectory_connect :null)) (setq _trajectory_connect __trajectory_connect)) _trajectory_connect)
  (:frame_type
   (&optional (__frame_type :null))
   (if (not (eq __frame_type :null)) (setq _frame_type __frame_type)) _frame_type)
  (:block
   (&optional (__block :null))
   (if (not (eq __block :null)) (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _offset
    (send _offset :serialization-length)
    ;; int32 _speed
    4
    ;; int32 _r
    4
    ;; bool _trajectory_connect
    1
    ;; bool _frame_type
    1
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _offset
       (send _offset :serialize s)
     ;; int32 _speed
       (write-long _speed s)
     ;; int32 _r
       (write-long _r s)
     ;; bool _trajectory_connect
       (if _trajectory_connect (write-byte -1 s) (write-byte 0 s))
     ;; bool _frame_type
       (if _frame_type (write-byte -1 s) (write-byte 0 s))
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _offset
     (send _offset :deserialize buf ptr-) (incf ptr- (send _offset :serialization-length))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _r
     (setq _r (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _trajectory_connect
     (setq _trajectory_connect (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _frame_type
     (setq _frame_type (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rm_msgs::Moveloffset :md5sum-) "af2efa1aef4c53564271cdf8a17ca144")
(setf (get rm_msgs::Moveloffset :datatype-) "rm_msgs/Moveloffset")
(setf (get rm_msgs::Moveloffset :definition-)
      "geometry_msgs/Pose offset
int32 speed
int32 r
bool trajectory_connect
bool frame_type
bool block
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :rm_msgs/Moveloffset "af2efa1aef4c53564271cdf8a17ca144")


