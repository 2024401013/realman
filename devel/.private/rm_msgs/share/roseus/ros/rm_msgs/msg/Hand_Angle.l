;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Hand_Angle)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Hand_Angle (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::HAND_ANGLE")
  (make-package "RM_MSGS::HAND_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude Hand_Angle.msg.html


(defclass rm_msgs::Hand_Angle
  :super ros::object
  :slots (_hand_angle _block ))

(defmethod rm_msgs::Hand_Angle
  (:init
   (&key
    ((:hand_angle __hand_angle) (make-array 6 :initial-element 0 :element-type :integer))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _hand_angle __hand_angle)
   (setq _block __block)
   self)
  (:hand_angle
   (&optional __hand_angle)
   (if __hand_angle (setq _hand_angle __hand_angle)) _hand_angle)
  (:block
   (&optional (__block :null))
   (if (not (eq __block :null)) (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; int16[6] _hand_angle
    (* 2    6)
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[6] _hand_angle
     (dotimes (i 6)
       (write-word (elt _hand_angle i) s)
       )
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[6] _hand_angle
   (dotimes (i (length _hand_angle))
     (setf (elt _hand_angle i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rm_msgs::Hand_Angle :md5sum-) "56a0667211866c2d94cc70494fe305c8")
(setf (get rm_msgs::Hand_Angle :datatype-) "rm_msgs/Hand_Angle")
(setf (get rm_msgs::Hand_Angle :definition-)
      "#设置灵巧手角度，灵巧手有 6 个自由度，从 1~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转

int16[6] hand_angle	#手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态
bool block #true 表示阻塞模式，false 表示非阻塞模式。

")



(provide :rm_msgs/Hand_Angle "56a0667211866c2d94cc70494fe305c8")


