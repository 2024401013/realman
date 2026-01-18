;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Register_Data)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Register_Data (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::REGISTER_DATA")
  (make-package "RM_MSGS::REGISTER_DATA"))

(in-package "ROS")
;;//! \htmlinclude Register_Data.msg.html


(defclass rm_msgs::Register_Data
  :super ros::object
  :slots (_state _data ))

(defmethod rm_msgs::Register_Data
  (:init
   (&key
    ((:state __state) nil)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _state __state)
   (setq _data __data)
   self)
  (:state
   (&optional (__state :null))
   (if (not (eq __state :null)) (setq _state __state)) _state)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ;; uint16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;; uint16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get rm_msgs::Register_Data :md5sum-) "05f2c487394b400e881ba98c819c8442")
(setf (get rm_msgs::Register_Data :datatype-) "rm_msgs/Register_Data")
(setf (get rm_msgs::Register_Data :definition-)
      "bool state
uint16[] data
")



(provide :rm_msgs/Register_Data "05f2c487394b400e881ba98c819c8442")


