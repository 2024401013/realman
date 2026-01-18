;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Jointversion)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Jointversion (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::JOINTVERSION")
  (make-package "RM_MSGS::JOINTVERSION"))

(in-package "ROS")
;;//! \htmlinclude Jointversion.msg.html


(defclass rm_msgs::Jointversion
  :super ros::object
  :slots (_joint_version ))

(defmethod rm_msgs::Jointversion
  (:init
   (&key
    ((:joint_version __joint_version) (let (r) (dotimes (i 7) (push "" r)) r))
    )
   (send-super :init)
   (setq _joint_version __joint_version)
   self)
  (:joint_version
   (&optional __joint_version)
   (if __joint_version (setq _joint_version __joint_version)) _joint_version)
  (:serialization-length
   ()
   (+
    ;; string[7] _joint_version
    (apply #'+ 4 (length _joint_version)    7)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[7] _joint_version
     (write-long (length _joint_version) s)
     (dolist (elem _joint_version)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[7] _joint_version
   (dotimes (i (length _joint_version))
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_version i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     )
   ;;
   self)
  )

(setf (get rm_msgs::Jointversion :md5sum-) "34fe8048af36c99d06616a2279b77cd6")
(setf (get rm_msgs::Jointversion :datatype-) "rm_msgs/Jointversion")
(setf (get rm_msgs::Jointversion :definition-)
      "string[7] joint_version
")



(provide :rm_msgs/Jointversion "34fe8048af36c99d06616a2279b77cd6")


