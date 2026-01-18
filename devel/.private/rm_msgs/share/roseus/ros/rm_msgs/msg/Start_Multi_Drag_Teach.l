;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Start_Multi_Drag_Teach)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Start_Multi_Drag_Teach (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::START_MULTI_DRAG_TEACH")
  (make-package "RM_MSGS::START_MULTI_DRAG_TEACH"))

(in-package "ROS")
;;//! \htmlinclude Start_Multi_Drag_Teach.msg.html


(defclass rm_msgs::Start_Multi_Drag_Teach
  :super ros::object
  :slots (_free_axes _frame _singular_wall ))

(defmethod rm_msgs::Start_Multi_Drag_Teach
  (:init
   (&key
    ((:free_axes __free_axes) (make-array 6 :initial-element 0 :element-type :integer))
    ((:frame __frame) 0)
    ((:singular_wall __singular_wall) 0)
    )
   (send-super :init)
   (setq _free_axes __free_axes)
   (setq _frame (round __frame))
   (setq _singular_wall (round __singular_wall))
   self)
  (:free_axes
   (&optional __free_axes)
   (if __free_axes (setq _free_axes __free_axes)) _free_axes)
  (:frame
   (&optional __frame)
   (if __frame (setq _frame __frame)) _frame)
  (:singular_wall
   (&optional __singular_wall)
   (if __singular_wall (setq _singular_wall __singular_wall)) _singular_wall)
  (:serialization-length
   ()
   (+
    ;; int32[6] _free_axes
    (* 4    6)
    ;; int32 _frame
    4
    ;; int32 _singular_wall
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[6] _free_axes
     (dotimes (i 6)
       (write-long (elt _free_axes i) s)
       )
     ;; int32 _frame
       (write-long _frame s)
     ;; int32 _singular_wall
       (write-long _singular_wall s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[6] _free_axes
   (dotimes (i (length _free_axes))
     (setf (elt _free_axes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; int32 _frame
     (setq _frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _singular_wall
     (setq _singular_wall (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Start_Multi_Drag_Teach :md5sum-) "36a7e83e1c20d27bf9d63d9ab11797e1")
(setf (get rm_msgs::Start_Multi_Drag_Teach :datatype-) "rm_msgs/Start_Multi_Drag_Teach")
(setf (get rm_msgs::Start_Multi_Drag_Teach :definition-)
      "#uint8 mode
int32[6] free_axes       # 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动
int32 frame              # 参考坐标系，0-工作坐标系 1-工具坐标系。
int32 singular_wall      # 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙

")



(provide :rm_msgs/Start_Multi_Drag_Teach "36a7e83e1c20d27bf9d63d9ab11797e1")


