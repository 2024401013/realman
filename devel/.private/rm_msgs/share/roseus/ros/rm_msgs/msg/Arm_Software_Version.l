;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Arm_Software_Version)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Arm_Software_Version (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::ARM_SOFTWARE_VERSION")
  (make-package "RM_MSGS::ARM_SOFTWARE_VERSION"))

(in-package "ROS")
;;//! \htmlinclude Arm_Software_Version.msg.html


(defclass rm_msgs::Arm_Software_Version
  :super ros::object
  :slots (_product_version _algorithm_info _ctrl_info _dynamic_info _plan_info _controller_version _com_info _program_info ))

(defmethod rm_msgs::Arm_Software_Version
  (:init
   (&key
    ((:product_version __product_version) "")
    ((:algorithm_info __algorithm_info) "")
    ((:ctrl_info __ctrl_info) (instance rm_msgs::Softwarebuildinfo :init))
    ((:dynamic_info __dynamic_info) "")
    ((:plan_info __plan_info) (instance rm_msgs::Softwarebuildinfo :init))
    ((:controller_version __controller_version) "")
    ((:com_info __com_info) (instance rm_msgs::Softwarebuildinfo :init))
    ((:program_info __program_info) (instance rm_msgs::Softwarebuildinfo :init))
    )
   (send-super :init)
   (setq _product_version (string __product_version))
   (setq _algorithm_info (string __algorithm_info))
   (setq _ctrl_info __ctrl_info)
   (setq _dynamic_info (string __dynamic_info))
   (setq _plan_info __plan_info)
   (setq _controller_version (string __controller_version))
   (setq _com_info __com_info)
   (setq _program_info __program_info)
   self)
  (:product_version
   (&optional __product_version)
   (if __product_version (setq _product_version __product_version)) _product_version)
  (:algorithm_info
   (&optional __algorithm_info)
   (if __algorithm_info (setq _algorithm_info __algorithm_info)) _algorithm_info)
  (:ctrl_info
   (&rest __ctrl_info)
   (if (keywordp (car __ctrl_info))
       (send* _ctrl_info __ctrl_info)
     (progn
       (if __ctrl_info (setq _ctrl_info (car __ctrl_info)))
       _ctrl_info)))
  (:dynamic_info
   (&optional __dynamic_info)
   (if __dynamic_info (setq _dynamic_info __dynamic_info)) _dynamic_info)
  (:plan_info
   (&rest __plan_info)
   (if (keywordp (car __plan_info))
       (send* _plan_info __plan_info)
     (progn
       (if __plan_info (setq _plan_info (car __plan_info)))
       _plan_info)))
  (:controller_version
   (&optional __controller_version)
   (if __controller_version (setq _controller_version __controller_version)) _controller_version)
  (:com_info
   (&rest __com_info)
   (if (keywordp (car __com_info))
       (send* _com_info __com_info)
     (progn
       (if __com_info (setq _com_info (car __com_info)))
       _com_info)))
  (:program_info
   (&rest __program_info)
   (if (keywordp (car __program_info))
       (send* _program_info __program_info)
     (progn
       (if __program_info (setq _program_info (car __program_info)))
       _program_info)))
  (:serialization-length
   ()
   (+
    ;; string _product_version
    4 (length _product_version)
    ;; string _algorithm_info
    4 (length _algorithm_info)
    ;; rm_msgs/Softwarebuildinfo _ctrl_info
    (send _ctrl_info :serialization-length)
    ;; string _dynamic_info
    4 (length _dynamic_info)
    ;; rm_msgs/Softwarebuildinfo _plan_info
    (send _plan_info :serialization-length)
    ;; string _controller_version
    4 (length _controller_version)
    ;; rm_msgs/Softwarebuildinfo _com_info
    (send _com_info :serialization-length)
    ;; rm_msgs/Softwarebuildinfo _program_info
    (send _program_info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _product_version
       (write-long (length _product_version) s) (princ _product_version s)
     ;; string _algorithm_info
       (write-long (length _algorithm_info) s) (princ _algorithm_info s)
     ;; rm_msgs/Softwarebuildinfo _ctrl_info
       (send _ctrl_info :serialize s)
     ;; string _dynamic_info
       (write-long (length _dynamic_info) s) (princ _dynamic_info s)
     ;; rm_msgs/Softwarebuildinfo _plan_info
       (send _plan_info :serialize s)
     ;; string _controller_version
       (write-long (length _controller_version) s) (princ _controller_version s)
     ;; rm_msgs/Softwarebuildinfo _com_info
       (send _com_info :serialize s)
     ;; rm_msgs/Softwarebuildinfo _program_info
       (send _program_info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _product_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _product_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _algorithm_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _algorithm_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; rm_msgs/Softwarebuildinfo _ctrl_info
     (send _ctrl_info :deserialize buf ptr-) (incf ptr- (send _ctrl_info :serialization-length))
   ;; string _dynamic_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dynamic_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; rm_msgs/Softwarebuildinfo _plan_info
     (send _plan_info :deserialize buf ptr-) (incf ptr- (send _plan_info :serialization-length))
   ;; string _controller_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; rm_msgs/Softwarebuildinfo _com_info
     (send _com_info :deserialize buf ptr-) (incf ptr- (send _com_info :serialization-length))
   ;; rm_msgs/Softwarebuildinfo _program_info
     (send _program_info :deserialize buf ptr-) (incf ptr- (send _program_info :serialization-length))
   ;;
   self)
  )

(setf (get rm_msgs::Arm_Software_Version :md5sum-) "f90eb6551bbde202ac1756e5a3ee2f6d")
(setf (get rm_msgs::Arm_Software_Version :datatype-) "rm_msgs/Arm_Software_Version")
(setf (get rm_msgs::Arm_Software_Version :definition-)
      "string product_version
string algorithm_info
Softwarebuildinfo ctrl_info
string dynamic_info #3
Softwarebuildinfo plan_info #3
string controller_version #4
Softwarebuildinfo com_info #4
Softwarebuildinfo program_info #4
================================================================================
MSG: rm_msgs/Softwarebuildinfo
string build_time
string version
")



(provide :rm_msgs/Arm_Software_Version "f90eb6551bbde202ac1756e5a3ee2f6d")


