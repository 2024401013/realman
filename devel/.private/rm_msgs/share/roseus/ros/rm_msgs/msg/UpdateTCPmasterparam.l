;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::UpdateTCPmasterparam)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'UpdateTCPmasterparam (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::UPDATETCPMASTERPARAM")
  (make-package "RM_MSGS::UPDATETCPMASTERPARAM"))

(in-package "ROS")
;;//! \htmlinclude UpdateTCPmasterparam.msg.html


(defclass rm_msgs::UpdateTCPmasterparam
  :super ros::object
  :slots (_master_name _new_name _ip _port ))

(defmethod rm_msgs::UpdateTCPmasterparam
  (:init
   (&key
    ((:master_name __master_name) "")
    ((:new_name __new_name) "")
    ((:ip __ip) "")
    ((:port __port) 0)
    )
   (send-super :init)
   (setq _master_name (string __master_name))
   (setq _new_name (string __new_name))
   (setq _ip (string __ip))
   (setq _port (round __port))
   self)
  (:master_name
   (&optional __master_name)
   (if __master_name (setq _master_name __master_name)) _master_name)
  (:new_name
   (&optional __new_name)
   (if __new_name (setq _new_name __new_name)) _new_name)
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
    ;; string _new_name
    4 (length _new_name)
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
     ;; string _new_name
       (write-long (length _new_name) s) (princ _new_name s)
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
   ;; string _new_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _new_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::UpdateTCPmasterparam :md5sum-) "76137ea31de5ca5b7a105efb1f13f4c7")
(setf (get rm_msgs::UpdateTCPmasterparam :datatype-) "rm_msgs/UpdateTCPmasterparam")
(setf (get rm_msgs::UpdateTCPmasterparam :definition-)
      "string master_name #要修改的tcp主站名称
string new_name #新名称
string ip #新IP
int32 port #新端口
")



(provide :rm_msgs/UpdateTCPmasterparam "76137ea31de5ca5b7a105efb1f13f4c7")


