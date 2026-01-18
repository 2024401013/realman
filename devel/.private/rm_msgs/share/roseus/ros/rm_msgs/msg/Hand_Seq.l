;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Hand_Seq)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Hand_Seq (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::HAND_SEQ")
  (make-package "RM_MSGS::HAND_SEQ"))

(in-package "ROS")
;;//! \htmlinclude Hand_Seq.msg.html


(defclass rm_msgs::Hand_Seq
  :super ros::object
  :slots (_seq_num _block ))

(defmethod rm_msgs::Hand_Seq
  (:init
   (&key
    ((:seq_num __seq_num) 0)
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _seq_num (round __seq_num))
   (setq _block __block)
   self)
  (:seq_num
   (&optional __seq_num)
   (if __seq_num (setq _seq_num __seq_num)) _seq_num)
  (:block
   (&optional (__block :null))
   (if (not (eq __block :null)) (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; uint16 _seq_num
    2
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _seq_num
       (write-word _seq_num s)
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _seq_num
     (setq _seq_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rm_msgs::Hand_Seq :md5sum-) "f00b164e44237e7253d6d77c16c73f25")
(setf (get rm_msgs::Hand_Seq :datatype-) "rm_msgs/Hand_Seq")
(setf (get rm_msgs::Hand_Seq :definition-)
      "#设置灵巧手动作序列
uint16 seq_num         #预先保存在灵巧手内的序列序号，范围：1~40
bool block #true 表示阻塞模式，false 表示非阻塞模式。
")



(provide :rm_msgs/Hand_Seq "f00b164e44237e7253d6d77c16c73f25")


