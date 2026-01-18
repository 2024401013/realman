;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Modbusreaddata)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Modbusreaddata (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::MODBUSREADDATA")
  (make-package "RM_MSGS::MODBUSREADDATA"))

(in-package "ROS")
;;//! \htmlinclude Modbusreaddata.msg.html


(defclass rm_msgs::Modbusreaddata
  :super ros::object
  :slots (_read_data ))

(defmethod rm_msgs::Modbusreaddata
  (:init
   (&key
    ((:read_data __read_data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _read_data __read_data)
   self)
  (:read_data
   (&optional __read_data)
   (if __read_data (setq _read_data __read_data)) _read_data)
  (:serialization-length
   ()
   (+
    ;; int32[] _read_data
    (* 4    (length _read_data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _read_data
     (write-long (length _read_data) s)
     (dotimes (i (length _read_data))
       (write-long (elt _read_data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _read_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _read_data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _read_data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rm_msgs::Modbusreaddata :md5sum-) "85133f00e9641b3d842dc178852ca264")
(setf (get rm_msgs::Modbusreaddata :datatype-) "rm_msgs/Modbusreaddata")
(setf (get rm_msgs::Modbusreaddata :definition-)
      "int32[] read_data
")



(provide :rm_msgs/Modbusreaddata "85133f00e9641b3d842dc178852ca264")


