;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Write_ModbusRTU)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Write_ModbusRTU (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::WRITE_MODBUSRTU")
  (make-package "RM_MSGS::WRITE_MODBUSRTU"))

(in-package "ROS")
;;//! \htmlinclude Write_ModbusRTU.msg.html


(defclass rm_msgs::Write_ModbusRTU
  :super ros::object
  :slots (_address _data _type _device ))

(defmethod rm_msgs::Write_ModbusRTU
  (:init
   (&key
    ((:address __address) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:type __type) 0)
    ((:device __device) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _data __data)
   (setq _type (round __type))
   (setq _device (round __device))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:serialization-length
   ()
   (+
    ;; int32 _address
    4
    ;; int32[] _data
    (* 4    (length _data)) 4
    ;; int32 _type
    4
    ;; int32 _device
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _address
       (write-long _address s)
     ;; int32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-long (elt _data i) s)
       )
     ;; int32 _type
       (write-long _type s)
     ;; int32 _device
       (write-long _device s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Write_ModbusRTU :md5sum-) "af497b1a335cfca6101c39a0e6e4ca1a")
(setf (get rm_msgs::Write_ModbusRTU :datatype-) "rm_msgs/Write_ModbusRTU")
(setf (get rm_msgs::Write_ModbusRTU :definition-)
      "int32 address
int32[] data
int32 type
int32 device
")



(provide :rm_msgs/Write_ModbusRTU "af497b1a335cfca6101c39a0e6e4ca1a")


