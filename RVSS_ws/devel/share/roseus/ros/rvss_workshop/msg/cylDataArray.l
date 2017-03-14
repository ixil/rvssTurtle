;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::cylDataArray)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'cylDataArray (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::CYLDATAARRAY")
  (make-package "RVSS_WORKSHOP::CYLDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude cylDataArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rvss_workshop::cylDataArray
  :super ros::object
  :slots (_header _cylinders ))

(defmethod rvss_workshop::cylDataArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cylinders __cylinders) (let (r) (dotimes (i 0) (push (instance rvss_workshop::cylMsg :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _cylinders __cylinders)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cylinders
   (&rest __cylinders)
   (if (keywordp (car __cylinders))
       (send* _cylinders __cylinders)
     (progn
       (if __cylinders (setq _cylinders (car __cylinders)))
       _cylinders)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; rvss_workshop/cylMsg[] _cylinders
    (apply #'+ (send-all _cylinders :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; rvss_workshop/cylMsg[] _cylinders
     (write-long (length _cylinders) s)
     (dolist (elem _cylinders)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; rvss_workshop/cylMsg[] _cylinders
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cylinders (let (r) (dotimes (i n) (push (instance rvss_workshop::cylMsg :init) r)) r))
     (dolist (elem- _cylinders)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rvss_workshop::cylDataArray :md5sum-) "59182e50b0c2a27f7f6f0f07e63a5e57")
(setf (get rvss_workshop::cylDataArray :datatype-) "rvss_workshop/cylDataArray")
(setf (get rvss_workshop::cylDataArray :definition-)
      "Header header
cylMsg[] cylinders

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: rvss_workshop/cylMsg
Header header
float32 Zrobot
float32 Xrobot
uint8 label
float32[] covariance

")



(provide :rvss_workshop/cylDataArray "59182e50b0c2a27f7f6f0f07e63a5e57")


