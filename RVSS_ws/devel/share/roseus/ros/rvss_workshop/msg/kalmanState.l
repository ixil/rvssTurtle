;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::kalmanState)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'kalmanState (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::KALMANSTATE")
  (make-package "RVSS_WORKSHOP::KALMANSTATE"))

(in-package "ROS")
;;//! \htmlinclude kalmanState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rvss_workshop::kalmanState
  :super ros::object
  :slots (_header _stateMean _stateCovariance _seenLandmarks ))

(defmethod rvss_workshop::kalmanState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stateMean __stateMean) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateCovariance __stateCovariance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:seenLandmarks __seenLandmarks) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _stateMean __stateMean)
   (setq _stateCovariance __stateCovariance)
   (setq _seenLandmarks __seenLandmarks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:stateMean
   (&optional __stateMean)
   (if __stateMean (setq _stateMean __stateMean)) _stateMean)
  (:stateCovariance
   (&optional __stateCovariance)
   (if __stateCovariance (setq _stateCovariance __stateCovariance)) _stateCovariance)
  (:seenLandmarks
   (&optional __seenLandmarks)
   (if __seenLandmarks (setq _seenLandmarks __seenLandmarks)) _seenLandmarks)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _stateMean
    (* 4    (length _stateMean)) 4
    ;; float32[] _stateCovariance
    (* 4    (length _stateCovariance)) 4
    ;; float32[] _seenLandmarks
    (* 4    (length _seenLandmarks)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _stateMean
     (write-long (length _stateMean) s)
     (dotimes (i (length _stateMean))
       (sys::poke (elt _stateMean i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateCovariance
     (write-long (length _stateCovariance) s)
     (dotimes (i (length _stateCovariance))
       (sys::poke (elt _stateCovariance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _seenLandmarks
     (write-long (length _seenLandmarks) s)
     (dotimes (i (length _seenLandmarks))
       (sys::poke (elt _seenLandmarks i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _stateMean
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateMean (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateMean i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateCovariance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateCovariance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateCovariance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _seenLandmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _seenLandmarks (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _seenLandmarks i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rvss_workshop::kalmanState :md5sum-) "0f14a0eeede5ba8bdddce3e0e54f3254")
(setf (get rvss_workshop::kalmanState :datatype-) "rvss_workshop/kalmanState")
(setf (get rvss_workshop::kalmanState :definition-)
      "Header header
float32[] stateMean
float32[] stateCovariance
float32[] seenLandmarks

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



(provide :rvss_workshop/kalmanState "0f14a0eeede5ba8bdddce3e0e54f3254")


