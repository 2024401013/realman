;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Mastername)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Mastername (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::MASTERNAME")
  (make-package "RM_MSGS::MASTERNAME"))

(in-package "ROS")
;;//! \htmlinclude Mastername.msg.html


(defclass rm_msgs::Mastername
  :super ros::object
  :slots (_master_name ))

(defmethod rm_msgs::Mastername
  (:init
   (&key
    ((:master_name __master_name) "")
    )
   (send-super :init)
   (setq _master_name (string __master_name))
   self)
  (:master_name
   (&optional __master_name)
   (if __master_name (setq _master_name __master_name)) _master_name)
  (:serialization-length
   ()
   (+
    ;; string _master_name
    4 (length _master_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _master_name
       (write-long (length _master_name) s) (princ _master_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _master_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _master_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rm_msgs::Mastername :md5sum-) "85cc930e49a075f3a896b60d7598ba17")
(setf (get rm_msgs::Mastername :datatype-) "rm_msgs/Mastername")
(setf (get rm_msgs::Mastername :definition-)
      "string master_name
")



(provide :rm_msgs/Mastername "85cc930e49a075f3a896b60d7598ba17")


