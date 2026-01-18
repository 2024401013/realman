;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Read_TCPandRTU)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Read_TCPandRTU (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::READ_TCPANDRTU")
  (make-package "RM_MSGS::READ_TCPANDRTU"))

(in-package "ROS")
;;//! \htmlinclude Read_TCPandRTU.msg.html


(defclass rm_msgs::Read_TCPandRTU
  :super ros::object
  :slots (_address _num _ip _port _master_name _device _type ))

(defmethod rm_msgs::Read_TCPandRTU
  (:init
   (&key
    ((:address __address) 0)
    ((:num __num) 0)
    ((:ip __ip) "")
    ((:port __port) 0)
    ((:master_name __master_name) "")
    ((:device __device) 0)
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _num (round __num))
   (setq _ip (string __ip))
   (setq _port (round __port))
   (setq _master_name (string __master_name))
   (setq _device (round __device))
   (setq _type (round __type))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:ip
   (&optional __ip)
   (if __ip (setq _ip __ip)) _ip)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:master_name
   (&optional __master_name)
   (if __master_name (setq _master_name __master_name)) _master_name)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; int32 _address
    4
    ;; int32 _num
    4
    ;; string _ip
    4 (length _ip)
    ;; int32 _port
    4
    ;; string _master_name
    4 (length _master_name)
    ;; int32 _device
    4
    ;; int32 _type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _address
       (write-long _address s)
     ;; int32 _num
       (write-long _num s)
     ;; string _ip
       (write-long (length _ip) s) (princ _ip s)
     ;; int32 _port
       (write-long _port s)
     ;; string _master_name
       (write-long (length _master_name) s) (princ _master_name s)
     ;; int32 _device
       (write-long _device s)
     ;; int32 _type
       (write-long _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _master_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _master_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Read_TCPandRTU :md5sum-) "90a88b29976390e1b70c71f43af18e9c")
(setf (get rm_msgs::Read_TCPandRTU :datatype-) "rm_msgs/Read_TCPandRTU")
(setf (get rm_msgs::Read_TCPandRTU :definition-)
      "int32 address
int32 num
string ip
int32 port
string master_name
int32 device #外设设备地址。
int32 type #0-控制器端modbus主机；1-工具端modbus主机。3-TCP模式
")



(provide :rm_msgs/Read_TCPandRTU "90a88b29976390e1b70c71f43af18e9c")


