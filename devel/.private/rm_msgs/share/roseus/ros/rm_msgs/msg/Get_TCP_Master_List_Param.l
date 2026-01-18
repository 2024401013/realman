;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Get_TCP_Master_List_Param)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Get_TCP_Master_List_Param (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::GET_TCP_MASTER_LIST_PARAM")
  (make-package "RM_MSGS::GET_TCP_MASTER_LIST_PARAM"))

(in-package "ROS")
;;//! \htmlinclude Get_TCP_Master_List_Param.msg.html


(defclass rm_msgs::Get_TCP_Master_List_Param
  :super ros::object
  :slots (_page_num _page_size _vague_search ))

(defmethod rm_msgs::Get_TCP_Master_List_Param
  (:init
   (&key
    ((:page_num __page_num) 0)
    ((:page_size __page_size) 0)
    ((:vague_search __vague_search) "")
    )
   (send-super :init)
   (setq _page_num (round __page_num))
   (setq _page_size (round __page_size))
   (setq _vague_search (string __vague_search))
   self)
  (:page_num
   (&optional __page_num)
   (if __page_num (setq _page_num __page_num)) _page_num)
  (:page_size
   (&optional __page_size)
   (if __page_size (setq _page_size __page_size)) _page_size)
  (:vague_search
   (&optional __vague_search)
   (if __vague_search (setq _vague_search __vague_search)) _vague_search)
  (:serialization-length
   ()
   (+
    ;; int32 _page_num
    4
    ;; int32 _page_size
    4
    ;; string _vague_search
    4 (length _vague_search)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _page_num
       (write-long _page_num s)
     ;; int32 _page_size
       (write-long _page_size s)
     ;; string _vague_search
       (write-long (length _vague_search) s) (princ _vague_search s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _page_num
     (setq _page_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _page_size
     (setq _page_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _vague_search
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _vague_search (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rm_msgs::Get_TCP_Master_List_Param :md5sum-) "cbe34dfd8b66421bedf70042e463f5a2")
(setf (get rm_msgs::Get_TCP_Master_List_Param :datatype-) "rm_msgs/Get_TCP_Master_List_Param")
(setf (get rm_msgs::Get_TCP_Master_List_Param :definition-)
      "int32 page_num       # 页码
int32 page_size      # 每页大小
string vague_search  # 模糊搜索	
")



(provide :rm_msgs/Get_TCP_Master_List_Param "cbe34dfd8b66421bedf70042e463f5a2")


