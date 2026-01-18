;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Gettrajectorylist)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Gettrajectorylist (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::GETTRAJECTORYLIST")
  (make-package "RM_MSGS::GETTRAJECTORYLIST"))

(in-package "ROS")
;;//! \htmlinclude Gettrajectorylist.msg.html


(defclass rm_msgs::Gettrajectorylist
  :super ros::object
  :slots (_page_num _page_size _vague_search ))

(defmethod rm_msgs::Gettrajectorylist
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

(setf (get rm_msgs::Gettrajectorylist :md5sum-) "cbe34dfd8b66421bedf70042e463f5a2")
(setf (get rm_msgs::Gettrajectorylist :datatype-) "rm_msgs/Gettrajectorylist")
(setf (get rm_msgs::Gettrajectorylist :definition-)
      "int32 page_num
int32 page_size
string vague_search
")



(provide :rm_msgs/Gettrajectorylist "cbe34dfd8b66421bedf70042e463f5a2")


