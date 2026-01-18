;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Modbustcpmasterinfo)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Modbustcpmasterinfo (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::MODBUSTCPMASTERINFO")
  (make-package "RM_MSGS::MODBUSTCPMASTERINFO"))

(in-package "ROS")
;;//! \htmlinclude Modbustcpmasterinfo.msg.html


(defclass rm_msgs::Modbustcpmasterinfo
  :super ros::object
  :slots (_master_name _ip _port ))

(defmethod rm_msgs::Modbustcpmasterinfo
  (:init
   (&key
    ((:master_name __master_name) "")
    ((:ip __ip) "")
    ((:port __port) 0)
    )
   (send-super :init)
   (setq _master_name (string __master_name))
   (setq _ip (string __ip))
   (setq _port (round __port))
   self)
  (:master_name
   (&optional __master_name)
   (if __master_name (setq _master_name __master_name)) _master_name)
  (:ip
   (&optional __ip)
   (if __ip (setq _ip __ip)) _ip)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:serialization-length
   ()
   (+
    ;; string _master_name
    4 (length _master_name)
    ;; string _ip
    4 (length _ip)
    ;; int32 _port
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _master_name
       (write-long (length _master_name) s) (princ _master_name s)
     ;; string _ip
       (write-long (length _ip) s) (princ _ip s)
     ;; int32 _port
       (write-long _port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _master_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _master_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Modbustcpmasterinfo :md5sum-) "5135e7787693648e691b7b714a55fa9e")
(setf (get rm_msgs::Modbustcpmasterinfo :datatype-) "rm_msgs/Modbustcpmasterinfo")
(setf (get rm_msgs::Modbustcpmasterinfo :definition-)
      "string master_name # Modbus 主站名称，最大长度15个字符
string ip          # TCP主站 IP 地址
int32 port         # TCP主站端口号	
")



(provide :rm_msgs/Modbustcpmasterinfo "5135e7787693648e691b7b714a55fa9e")


