;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::objDataArray)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'objDataArray (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::OBJDATAARRAY")
  (make-package "RVSS_WORKSHOP::OBJDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude objDataArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rvss_workshop::objDataArray
  :super ros::object
  :slots (_header _objects ))

(defmethod rvss_workshop::objDataArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance rvss_workshop::objMsg :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; rvss_workshop/objMsg[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; rvss_workshop/objMsg[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; rvss_workshop/objMsg[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance rvss_workshop::objMsg :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rvss_workshop::objDataArray :md5sum-) "6fb4a2149a0d8c8d308c11de8ad29ad3")
(setf (get rvss_workshop::objDataArray :datatype-) "rvss_workshop/objDataArray")
(setf (get rvss_workshop::objDataArray :definition-)
      "Header header
objMsg[] objects

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
MSG: rvss_workshop/objMsg
Header header
float32 Zrobot
float32 Xrobot
uint8 label
float32[] covariance

")



(provide :rvss_workshop/objDataArray "6fb4a2149a0d8c8d308c11de8ad29ad3")


