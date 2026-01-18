;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Programrunstate)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Programrunstate (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::PROGRAMRUNSTATE")
  (make-package "RM_MSGS::PROGRAMRUNSTATE"))

(in-package "ROS")
;;//! \htmlinclude Programrunstate.msg.html


(defclass rm_msgs::Programrunstate
  :super ros::object
  :slots (_run_state _id _edit_id _plan_num _total_loop _step_mode _plan_speed _loop_num _loop_cont ))

(defmethod rm_msgs::Programrunstate
  (:init
   (&key
    ((:run_state __run_state) 0)
    ((:id __id) 0)
    ((:edit_id __edit_id) 0)
    ((:plan_num __plan_num) 0)
    ((:total_loop __total_loop) 0)
    ((:step_mode __step_mode) 0)
    ((:plan_speed __plan_speed) 0)
    ((:loop_num __loop_num) (make-array 0 :initial-element 0 :element-type :integer))
    ((:loop_cont __loop_cont) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _run_state (round __run_state))
   (setq _id (round __id))
   (setq _edit_id (round __edit_id))
   (setq _plan_num (round __plan_num))
   (setq _total_loop (round __total_loop))
   (setq _step_mode (round __step_mode))
   (setq _plan_speed (round __plan_speed))
   (setq _loop_num __loop_num)
   (setq _loop_cont __loop_cont)
   self)
  (:run_state
   (&optional __run_state)
   (if __run_state (setq _run_state __run_state)) _run_state)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:edit_id
   (&optional __edit_id)
   (if __edit_id (setq _edit_id __edit_id)) _edit_id)
  (:plan_num
   (&optional __plan_num)
   (if __plan_num (setq _plan_num __plan_num)) _plan_num)
  (:total_loop
   (&optional __total_loop)
   (if __total_loop (setq _total_loop __total_loop)) _total_loop)
  (:step_mode
   (&optional __step_mode)
   (if __step_mode (setq _step_mode __step_mode)) _step_mode)
  (:plan_speed
   (&optional __plan_speed)
   (if __plan_speed (setq _plan_speed __plan_speed)) _plan_speed)
  (:loop_num
   (&optional __loop_num)
   (if __loop_num (setq _loop_num __loop_num)) _loop_num)
  (:loop_cont
   (&optional __loop_cont)
   (if __loop_cont (setq _loop_cont __loop_cont)) _loop_cont)
  (:serialization-length
   ()
   (+
    ;; int32 _run_state
    4
    ;; int32 _id
    4
    ;; int32 _edit_id
    4
    ;; int32 _plan_num
    4
    ;; int32 _total_loop
    4
    ;; int32 _step_mode
    4
    ;; int32 _plan_speed
    4
    ;; int32[] _loop_num
    (* 4    (length _loop_num)) 4
    ;; int32[] _loop_cont
    (* 4    (length _loop_cont)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _run_state
       (write-long _run_state s)
     ;; int32 _id
       (write-long _id s)
     ;; int32 _edit_id
       (write-long _edit_id s)
     ;; int32 _plan_num
       (write-long _plan_num s)
     ;; int32 _total_loop
       (write-long _total_loop s)
     ;; int32 _step_mode
       (write-long _step_mode s)
     ;; int32 _plan_speed
       (write-long _plan_speed s)
     ;; int32[] _loop_num
     (write-long (length _loop_num) s)
     (dotimes (i (length _loop_num))
       (write-long (elt _loop_num i) s)
       )
     ;; int32[] _loop_cont
     (write-long (length _loop_cont) s)
     (dotimes (i (length _loop_cont))
       (write-long (elt _loop_cont i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _run_state
     (setq _run_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _edit_id
     (setq _edit_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _plan_num
     (setq _plan_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _total_loop
     (setq _total_loop (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _step_mode
     (setq _step_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _plan_speed
     (setq _plan_speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _loop_num
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _loop_num (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _loop_num i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _loop_cont
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _loop_cont (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _loop_cont i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rm_msgs::Programrunstate :md5sum-) "f985b60b566dc245fe01afe38654a230")
(setf (get rm_msgs::Programrunstate :datatype-) "rm_msgs/Programrunstate")
(setf (get rm_msgs::Programrunstate :definition-)
      "int32 run_state   # 运行状态 0 未开始 1运行中 2暂停中
int32 id          # 运行轨迹编号
int32 edit_id     # 上次编辑的在线编程编号 id
int32 plan_num    # 运行行数
int32 total_loop      # 循环指令数量
int32 step_mode       # 单步模式，1 为单步模式，0 为非单步模式
int32 plan_speed      # 全局规划速度比例 1-100
int32[] loop_num        # 循环行数
int32[] loop_cont       # 对应循环次数
")



(provide :rm_msgs/Programrunstate "f985b60b566dc245fe01afe38654a230")


