;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Expand_Speed)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Expand_Speed (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::EXPAND_SPEED")
  (make-package "RM_MSGS::EXPAND_SPEED"))

(in-package "ROS")
;;//! \htmlinclude Expand_Speed.msg.html


(defclass rm_msgs::Expand_Speed
  :super ros::object
  :slots (_speed ))

(defmethod rm_msgs::Expand_Speed
  (:init
   (&key
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int16 _speed
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _speed
       (write-word _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _speed
     (setq _speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get rm_msgs::Expand_Speed :md5sum-) "368a599b530468ee137b04eea511a3ec")
(setf (get rm_msgs::Expand_Speed :datatype-) "rm_msgs/Expand_Speed")
(setf (get rm_msgs::Expand_Speed :definition-)
      "int16 speed         #扩展关节速度环控制速度百分比，-100~100
")



(provide :rm_msgs/Expand_Speed "368a599b530468ee137b04eea511a3ec")


