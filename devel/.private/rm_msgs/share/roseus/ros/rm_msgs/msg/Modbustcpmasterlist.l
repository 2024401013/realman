;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Modbustcpmasterlist)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Modbustcpmasterlist (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::MODBUSTCPMASTERLIST")
  (make-package "RM_MSGS::MODBUSTCPMASTERLIST"))

(in-package "ROS")
;;//! \htmlinclude Modbustcpmasterlist.msg.html


(defclass rm_msgs::Modbustcpmasterlist
  :super ros::object
  :slots (_master_list ))

(defmethod rm_msgs::Modbustcpmasterlist
  (:init
   (&key
    ((:master_list __master_list) ())
    )
   (send-super :init)
   (setq _master_list __master_list)
   self)
  (:master_list
   (&rest __master_list)
   (if (keywordp (car __master_list))
       (send* _master_list __master_list)
     (progn
       (if __master_list (setq _master_list (car __master_list)))
       _master_list)))
  (:serialization-length
   ()
   (+
    ;; rm_msgs/Modbustcpmasterinfo[] _master_list
    (apply #'+ (send-all _master_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; rm_msgs/Modbustcpmasterinfo[] _master_list
     (write-long (length _master_list) s)
     (dolist (elem _master_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; rm_msgs/Modbustcpmasterinfo[] _master_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _master_list (let (r) (dotimes (i n) (push (instance rm_msgs::Modbustcpmasterinfo :init) r)) r))
     (dolist (elem- _master_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rm_msgs::Modbustcpmasterlist :md5sum-) "4b8086f234a37bbe3de0d031d6dca80f")
(setf (get rm_msgs::Modbustcpmasterlist :datatype-) "rm_msgs/Modbustcpmasterlist")
(setf (get rm_msgs::Modbustcpmasterlist :definition-)
      "Modbustcpmasterinfo[] master_list   # 返回符合的TCP主站列表

================================================================================
MSG: rm_msgs/Modbustcpmasterinfo
string master_name # Modbus 主站名称，最大长度15个字符
string ip          # TCP主站 IP 地址
int32 port         # TCP主站端口号	
")



(provide :rm_msgs/Modbustcpmasterlist "4b8086f234a37bbe3de0d031d6dca80f")


