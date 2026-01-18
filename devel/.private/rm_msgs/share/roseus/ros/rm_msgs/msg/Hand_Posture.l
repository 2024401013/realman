;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Hand_Posture)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Hand_Posture (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::HAND_POSTURE")
  (make-package "RM_MSGS::HAND_POSTURE"))

(in-package "ROS")
;;//! \htmlinclude Hand_Posture.msg.html


(defclass rm_msgs::Hand_Posture
  :super ros::object
  :slots (_posture_num _block ))

(defmethod rm_msgs::Hand_Posture
  (:init
   (&key
    ((:posture_num __posture_num) 0)
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _posture_num (round __posture_num))
   (setq _block __block)
   self)
  (:posture_num
   (&optional __posture_num)
   (if __posture_num (setq _posture_num __posture_num)) _posture_num)
  (:block
   (&optional (__block :null))
   (if (not (eq __block :null)) (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; uint16 _posture_num
    2
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _posture_num
       (write-word _posture_num s)
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _posture_num
     (setq _posture_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rm_msgs::Hand_Posture :md5sum-) "072a222dc3f5190c1b4a2fdf87a54900")
(setf (get rm_msgs::Hand_Posture :datatype-) "rm_msgs/Hand_Posture")
(setf (get rm_msgs::Hand_Posture :definition-)
      "#设置灵巧手手势
uint16 posture_num         #预先保存在灵巧手内的手势序号，范围：1~40
bool block #true 表示阻塞模式，false 表示非阻塞模式。
")



(provide :rm_msgs/Hand_Posture "072a222dc3f5190c1b4a2fdf87a54900")


