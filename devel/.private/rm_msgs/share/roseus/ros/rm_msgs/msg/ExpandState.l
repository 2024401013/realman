;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::ExpandState)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'ExpandState (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::EXPANDSTATE")
  (make-package "RM_MSGS::EXPANDSTATE"))

(in-package "ROS")
;;//! \htmlinclude ExpandState.msg.html


(defclass rm_msgs::ExpandState
  :super ros::object
  :slots (_get_state _pos _err_flag _en_flag _current _mode _joint_id ))

(defmethod rm_msgs::ExpandState
  (:init
   (&key
    ((:get_state __get_state) nil)
    ((:pos __pos) 0)
    ((:err_flag __err_flag) 0)
    ((:en_flag __en_flag) 0)
    ((:current __current) 0)
    ((:mode __mode) 0)
    ((:joint_id __joint_id) 0)
    )
   (send-super :init)
   (setq _get_state __get_state)
   (setq _pos (round __pos))
   (setq _err_flag (round __err_flag))
   (setq _en_flag (round __en_flag))
   (setq _current (round __current))
   (setq _mode (round __mode))
   (setq _joint_id (round __joint_id))
   self)
  (:get_state
   (&optional (__get_state :null))
   (if (not (eq __get_state :null)) (setq _get_state __get_state)) _get_state)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:err_flag
   (&optional __err_flag)
   (if __err_flag (setq _err_flag __err_flag)) _err_flag)
  (:en_flag
   (&optional __en_flag)
   (if __en_flag (setq _en_flag __en_flag)) _en_flag)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:joint_id
   (&optional __joint_id)
   (if __joint_id (setq _joint_id __joint_id)) _joint_id)
  (:serialization-length
   ()
   (+
    ;; bool _get_state
    1
    ;; int16 _pos
    2
    ;; int16 _err_flag
    2
    ;; uint16 _en_flag
    2
    ;; uint16 _current
    2
    ;; byte _mode
    1
    ;; uint16 _joint_id
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _get_state
       (if _get_state (write-byte -1 s) (write-byte 0 s))
     ;; int16 _pos
       (write-word _pos s)
     ;; int16 _err_flag
       (write-word _err_flag s)
     ;; uint16 _en_flag
       (write-word _en_flag s)
     ;; uint16 _current
       (write-word _current s)
     ;; byte _mode
       (write-byte _mode s)
     ;; uint16 _joint_id
       (write-word _joint_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _get_state
     (setq _get_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _pos
     (setq _pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _err_flag
     (setq _err_flag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _en_flag
     (setq _en_flag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _current
     (setq _current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; byte _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; uint16 _joint_id
     (setq _joint_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get rm_msgs::ExpandState :md5sum-) "51186bb81aaa7ad5c549fa4631da2340")
(setf (get rm_msgs::ExpandState :datatype-) "rm_msgs/ExpandState")
(setf (get rm_msgs::ExpandState :definition-)
      "bool get_state
int16 pos	    #扩展关节角度，单位度，精度 0.001°
int16 err_flag	#驱动错误代码
uint16 en_flag	#扩展关节使能状态
uint16 current	#当前驱动电流，单位：mA，精度：1mA。
byte mode	    #当前扩展关节状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
uint16 joint_id	#扩展关节ID
")



(provide :rm_msgs/ExpandState "51186bb81aaa7ad5c549fa4631da2340")


