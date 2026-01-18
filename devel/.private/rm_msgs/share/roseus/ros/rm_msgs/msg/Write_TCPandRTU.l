;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Write_TCPandRTU)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Write_TCPandRTU (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::WRITE_TCPANDRTU")
  (make-package "RM_MSGS::WRITE_TCPANDRTU"))

(in-package "ROS")
;;//! \htmlinclude Write_TCPandRTU.msg.html


(defclass rm_msgs::Write_TCPandRTU
  :super ros::object
  :slots (_address _data _ip _port _master_name _type _device ))

(defmethod rm_msgs::Write_TCPandRTU
  (:init
   (&key
    ((:address __address) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:ip __ip) "")
    ((:port __port) 0)
    ((:master_name __master_name) "")
    ((:type __type) 0)
    ((:device __device) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _data __data)
   (setq _ip (string __ip))
   (setq _port (round __port))
   (setq _master_name (string __master_name))
   (setq _type (round __type))
   (setq _device (round __device))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:ip
   (&optional __ip)
   (if __ip (setq _ip __ip)) _ip)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:master_name
   (&optional __master_name)
   (if __master_name (setq _master_name __master_name)) _master_name)
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
    ;; string _ip
    4 (length _ip)
    ;; int32 _port
    4
    ;; string _master_name
    4 (length _master_name)
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
     ;; string _ip
       (write-long (length _ip) s) (princ _ip s)
     ;; int32 _port
       (write-long _port s)
     ;; string _master_name
       (write-long (length _master_name) s) (princ _master_name s)
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
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _master_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _master_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Write_TCPandRTU :md5sum-) "c74acded0fd8ce10898127e85288096e")
(setf (get rm_msgs::Write_TCPandRTU :datatype-) "rm_msgs/Write_TCPandRTU")
(setf (get rm_msgs::Write_TCPandRTU :definition-)
      "int32 address
int32[] data
string ip
int32 port
string master_name
int32 type
int32 device
")



(provide :rm_msgs/Write_TCPandRTU "c74acded0fd8ce10898127e85288096e")


