;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::RS485params)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'RS485params (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::RS485PARAMS")
  (make-package "RM_MSGS::RS485PARAMS"))

(in-package "ROS")
;;//! \htmlinclude RS485params.msg.html


(defclass rm_msgs::RS485params
  :super ros::object
  :slots (_mode _baudrate ))

(defmethod rm_msgs::RS485params
  (:init
   (&key
    ((:mode __mode) 0)
    ((:baudrate __baudrate) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _baudrate (round __baudrate))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:baudrate
   (&optional __baudrate)
   (if __baudrate (setq _baudrate __baudrate)) _baudrate)
  (:serialization-length
   ()
   (+
    ;; int32 _mode
    4
    ;; int32 _baudrate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mode
       (write-long _mode s)
     ;; int32 _baudrate
       (write-long _baudrate s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _baudrate
     (setq _baudrate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::RS485params :md5sum-) "78bc9e6cebd1458721f892a614d5c54c")
(setf (get rm_msgs::RS485params :datatype-) "rm_msgs/RS485params")
(setf (get rm_msgs::RS485params :definition-)
      "int32 mode
int32 baudrate
")



(provide :rm_msgs/RS485params "78bc9e6cebd1458721f892a614d5c54c")


