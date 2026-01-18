;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Flowchartrunstate)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Flowchartrunstate (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::FLOWCHARTRUNSTATE")
  (make-package "RM_MSGS::FLOWCHARTRUNSTATE"))

(in-package "ROS")
;;//! \htmlinclude Flowchartrunstate.msg.html


(defclass rm_msgs::Flowchartrunstate
  :super ros::object
  :slots (_run_state _id _name _plan_speed _step_mode _modal_id ))

(defmethod rm_msgs::Flowchartrunstate
  (:init
   (&key
    ((:run_state __run_state) 0)
    ((:id __id) 0)
    ((:name __name) "")
    ((:plan_speed __plan_speed) 0)
    ((:step_mode __step_mode) 0)
    ((:modal_id __modal_id) "")
    )
   (send-super :init)
   (setq _run_state (round __run_state))
   (setq _id (round __id))
   (setq _name (string __name))
   (setq _plan_speed (round __plan_speed))
   (setq _step_mode (round __step_mode))
   (setq _modal_id (string __modal_id))
   self)
  (:run_state
   (&optional __run_state)
   (if __run_state (setq _run_state __run_state)) _run_state)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:plan_speed
   (&optional __plan_speed)
   (if __plan_speed (setq _plan_speed __plan_speed)) _plan_speed)
  (:step_mode
   (&optional __step_mode)
   (if __step_mode (setq _step_mode __step_mode)) _step_mode)
  (:modal_id
   (&optional __modal_id)
   (if __modal_id (setq _modal_id __modal_id)) _modal_id)
  (:serialization-length
   ()
   (+
    ;; uint8 _run_state
    1
    ;; uint8 _id
    1
    ;; string _name
    4 (length _name)
    ;; uint8 _plan_speed
    1
    ;; uint8 _step_mode
    1
    ;; string _modal_id
    4 (length _modal_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _run_state
       (write-byte _run_state s)
     ;; uint8 _id
       (write-byte _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint8 _plan_speed
       (write-byte _plan_speed s)
     ;; uint8 _step_mode
       (write-byte _step_mode s)
     ;; string _modal_id
       (write-long (length _modal_id) s) (princ _modal_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _run_state
     (setq _run_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _plan_speed
     (setq _plan_speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _step_mode
     (setq _step_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _modal_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modal_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rm_msgs::Flowchartrunstate :md5sum-) "05f1a2e93a88cd1e5f9f80d8af8e987a")
(setf (get rm_msgs::Flowchartrunstate :datatype-) "rm_msgs/Flowchartrunstate")
(setf (get rm_msgs::Flowchartrunstate :definition-)
      "uint8 run_state  # 运行状态 0 未开始 1运行中 2暂停中
uint8 id         # 当前使能的文件id。
string name # 当前使能的文件名称。
uint8 plan_speed     # 当前使能的文件全局规划速度比例 1-100。
uint8 step_mode    # 单步模式，0为空，1为正常, 2为单步。
string modal_id   # 运行到的流程图块的id。未运行则不返回
")



(provide :rm_msgs/Flowchartrunstate "05f1a2e93a88cd1e5f9f80d8af8e987a")


