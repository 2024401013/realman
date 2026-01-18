;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Softwarebuildinfo)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Softwarebuildinfo (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::SOFTWAREBUILDINFO")
  (make-package "RM_MSGS::SOFTWAREBUILDINFO"))

(in-package "ROS")
;;//! \htmlinclude Softwarebuildinfo.msg.html


(defclass rm_msgs::Softwarebuildinfo
  :super ros::object
  :slots (_build_time _version ))

(defmethod rm_msgs::Softwarebuildinfo
  (:init
   (&key
    ((:build_time __build_time) "")
    ((:version __version) "")
    )
   (send-super :init)
   (setq _build_time (string __build_time))
   (setq _version (string __version))
   self)
  (:build_time
   (&optional __build_time)
   (if __build_time (setq _build_time __build_time)) _build_time)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:serialization-length
   ()
   (+
    ;; string _build_time
    4 (length _build_time)
    ;; string _version
    4 (length _version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _build_time
       (write-long (length _build_time) s) (princ _build_time s)
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _build_time
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _build_time (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rm_msgs::Softwarebuildinfo :md5sum-) "7075122606129dab433ee6c359a4c404")
(setf (get rm_msgs::Softwarebuildinfo :datatype-) "rm_msgs/Softwarebuildinfo")
(setf (get rm_msgs::Softwarebuildinfo :definition-)
      "string build_time
string version
")



(provide :rm_msgs/Softwarebuildinfo "7075122606129dab433ee6c359a4c404")


