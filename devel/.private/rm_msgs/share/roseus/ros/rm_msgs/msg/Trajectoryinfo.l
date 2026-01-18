;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Trajectoryinfo)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Trajectoryinfo (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::TRAJECTORYINFO")
  (make-package "RM_MSGS::TRAJECTORYINFO"))

(in-package "ROS")
;;//! \htmlinclude Trajectoryinfo.msg.html


(defclass rm_msgs::Trajectoryinfo
  :super ros::object
  :slots (_name _create_time _point_num ))

(defmethod rm_msgs::Trajectoryinfo
  (:init
   (&key
    ((:name __name) "")
    ((:create_time __create_time) "")
    ((:point_num __point_num) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _create_time (string __create_time))
   (setq _point_num (round __point_num))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:create_time
   (&optional __create_time)
   (if __create_time (setq _create_time __create_time)) _create_time)
  (:point_num
   (&optional __point_num)
   (if __point_num (setq _point_num __point_num)) _point_num)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _create_time
    4 (length _create_time)
    ;; int32 _point_num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _create_time
       (write-long (length _create_time) s) (princ _create_time s)
     ;; int32 _point_num
       (write-long _point_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _create_time
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _create_time (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _point_num
     (setq _point_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Trajectoryinfo :md5sum-) "e9fb2008a0ef07a81b5bc4d72b577068")
(setf (get rm_msgs::Trajectoryinfo :datatype-) "rm_msgs/Trajectoryinfo")
(setf (get rm_msgs::Trajectoryinfo :definition-)
      "string name          #轨迹名称	
string create_time   #创建时间
int32 point_num      #轨迹点数量
")



(provide :rm_msgs/Trajectoryinfo "e9fb2008a0ef07a81b5bc4d72b577068")


