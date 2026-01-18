;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Expand_Position)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Expand_Position (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::EXPAND_POSITION")
  (make-package "RM_MSGS::EXPAND_POSITION"))

(in-package "ROS")
;;//! \htmlinclude Expand_Position.msg.html


(defclass rm_msgs::Expand_Position
  :super ros::object
  :slots (_pos _speed ))

(defmethod rm_msgs::Expand_Position
  (:init
   (&key
    ((:pos __pos) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _pos (round __pos))
   (setq _speed (round __speed))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int16 _pos
    2
    ;; int16 _speed
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _pos
       (write-word _pos s)
     ;; int16 _speed
       (write-word _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _pos
     (setq _pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _speed
     (setq _speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get rm_msgs::Expand_Position :md5sum-) "464e5f6ba74ca32e53b631f4eeffd5ab")
(setf (get rm_msgs::Expand_Position :datatype-) "rm_msgs/Expand_Position")
(setf (get rm_msgs::Expand_Position :definition-)
      "int16 pos        #扩展关节位置，单位 
int16 speed         #速度百分比，1~100

")



(provide :rm_msgs/Expand_Position "464e5f6ba74ca32e53b631f4eeffd5ab")


