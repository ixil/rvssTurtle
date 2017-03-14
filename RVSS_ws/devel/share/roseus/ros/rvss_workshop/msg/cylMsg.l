;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::cylMsg)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'cylMsg (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::CYLMSG")
  (make-package "RVSS_WORKSHOP::CYLMSG"))

(in-package "ROS")
;;//! \htmlinclude cylMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rvss_workshop::cylMsg
  :super ros::object
  :slots (_header _Zrobot _Xrobot _label _covariance ))

(defmethod rvss_workshop::cylMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:Zrobot __Zrobot) 0.0)
    ((:Xrobot __Xrobot) 0.0)
    ((:label __label) 0)
    ((:covariance __covariance) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _Zrobot (float __Zrobot))
   (setq _Xrobot (float __Xrobot))
   (setq _label (round __label))
   (setq _covariance __covariance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:Zrobot
   (&optional __Zrobot)
   (if __Zrobot (setq _Zrobot __Zrobot)) _Zrobot)
  (:Xrobot
   (&optional __Xrobot)
   (if __Xrobot (setq _Xrobot __Xrobot)) _Xrobot)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _Zrobot
    4
    ;; float32 _Xrobot
    4
    ;; uint8 _label
    1
    ;; float32[] _covariance
    (* 4    (length _covariance)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _Zrobot
       (sys::poke _Zrobot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Xrobot
       (sys::poke _Xrobot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _label
       (write-byte _label s)
     ;; float32[] _covariance
     (write-long (length _covariance) s)
     (dotimes (i (length _covariance))
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _Zrobot
     (setq _Zrobot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Xrobot
     (setq _Xrobot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _label
     (setq _label (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32[] _covariance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _covariance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _covariance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rvss_workshop::cylMsg :md5sum-) "92039726cacaf0739b94657d27987f60")
(setf (get rvss_workshop::cylMsg :datatype-) "rvss_workshop/cylMsg")
(setf (get rvss_workshop::cylMsg :definition-)
      "Header header
float32 Zrobot
float32 Xrobot
uint8 label
float32[] covariance

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

")



(provide :rvss_workshop/cylMsg "92039726cacaf0739b94657d27987f60")


