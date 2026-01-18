;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Trajectorylist)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Trajectorylist (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::TRAJECTORYLIST")
  (make-package "RM_MSGS::TRAJECTORYLIST"))

(in-package "ROS")
;;//! \htmlinclude Trajectorylist.msg.html


(defclass rm_msgs::Trajectorylist
  :super ros::object
  :slots (_page_num _page_size _total_size _vague_search _tra_list ))

(defmethod rm_msgs::Trajectorylist
  (:init
   (&key
    ((:page_num __page_num) 0)
    ((:page_size __page_size) 0)
    ((:total_size __total_size) 0)
    ((:vague_search __vague_search) "")
    ((:tra_list __tra_list) ())
    )
   (send-super :init)
   (setq _page_num (round __page_num))
   (setq _page_size (round __page_size))
   (setq _total_size (round __total_size))
   (setq _vague_search (string __vague_search))
   (setq _tra_list __tra_list)
   self)
  (:page_num
   (&optional __page_num)
   (if __page_num (setq _page_num __page_num)) _page_num)
  (:page_size
   (&optional __page_size)
   (if __page_size (setq _page_size __page_size)) _page_size)
  (:total_size
   (&optional __total_size)
   (if __total_size (setq _total_size __total_size)) _total_size)
  (:vague_search
   (&optional __vague_search)
   (if __vague_search (setq _vague_search __vague_search)) _vague_search)
  (:tra_list
   (&rest __tra_list)
   (if (keywordp (car __tra_list))
       (send* _tra_list __tra_list)
     (progn
       (if __tra_list (setq _tra_list (car __tra_list)))
       _tra_list)))
  (:serialization-length
   ()
   (+
    ;; int32 _page_num
    4
    ;; int32 _page_size
    4
    ;; int32 _total_size
    4
    ;; string _vague_search
    4 (length _vague_search)
    ;; rm_msgs/Trajectoryinfo[] _tra_list
    (apply #'+ (send-all _tra_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _page_num
       (write-long _page_num s)
     ;; int32 _page_size
       (write-long _page_size s)
     ;; int32 _total_size
       (write-long _total_size s)
     ;; string _vague_search
       (write-long (length _vague_search) s) (princ _vague_search s)
     ;; rm_msgs/Trajectoryinfo[] _tra_list
     (write-long (length _tra_list) s)
     (dolist (elem _tra_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _page_num
     (setq _page_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _page_size
     (setq _page_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _total_size
     (setq _total_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _vague_search
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _vague_search (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; rm_msgs/Trajectoryinfo[] _tra_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tra_list (let (r) (dotimes (i n) (push (instance rm_msgs::Trajectoryinfo :init) r)) r))
     (dolist (elem- _tra_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rm_msgs::Trajectorylist :md5sum-) "8ffd0485c7441f6956165c1adeb616bc")
(setf (get rm_msgs::Trajectorylist :datatype-) "rm_msgs/Trajectorylist")
(setf (get rm_msgs::Trajectorylist :definition-)
      "int32 page_num      # 页码
int32 page_size     # 每页大小
int32 total_size    # 列表长度
string vague_search  # 模糊搜索 
Trajectoryinfo[] tra_list  # 返回符合的轨迹列表

================================================================================
MSG: rm_msgs/Trajectoryinfo
string name          #轨迹名称	
string create_time   #创建时间
int32 point_num      #轨迹点数量
")



(provide :rm_msgs/Trajectorylist "8ffd0485c7441f6956165c1adeb616bc")


