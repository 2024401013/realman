;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Read_ModbusRTU)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Read_ModbusRTU (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::READ_MODBUSRTU")
  (make-package "RM_MSGS::READ_MODBUSRTU"))

(in-package "ROS")
;;//! \htmlinclude Read_ModbusRTU.msg.html


(defclass rm_msgs::Read_ModbusRTU
  :super ros::object
  :slots (_address _device _num _type ))

(defmethod rm_msgs::Read_ModbusRTU
  (:init
   (&key
    ((:address __address) 0)
    ((:device __device) 0)
    ((:num __num) 0)
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _device (round __device))
   (setq _num (round __num))
   (setq _type (round __type))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; int32 _address
    4
    ;; int32 _device
    4
    ;; int32 _num
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
     ;; int32 _device
       (write-long _device s)
     ;; int32 _num
       (write-long _num s)
     ;; int32 _type
       (write-long _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Read_ModbusRTU :md5sum-) "58a66370687b1c7bdeac52f168ac52f1")
(setf (get rm_msgs::Read_ModbusRTU :datatype-) "rm_msgs/Read_ModbusRTU")
(setf (get rm_msgs::Read_ModbusRTU :definition-)
      "int32 address #线圈起始地址。
int32 device #外设设备地址。
int32 num #线圈数量。
int32 type #0-控制器端modbus主机；1-工具端modbus主机。
")



(provide :rm_msgs/Read_ModbusRTU "58a66370687b1c7bdeac52f168ac52f1")


