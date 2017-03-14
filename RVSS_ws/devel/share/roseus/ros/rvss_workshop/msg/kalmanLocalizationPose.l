;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::kalmanLocalizationPose)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'kalmanLocalizationPose (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::KALMANLOCALIZATIONPOSE")
  (make-package "RVSS_WORKSHOP::KALMANLOCALIZATIONPOSE"))

(in-package "ROS")
;;//! \htmlinclude kalmanLocalizationPose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rvss_workshop::kalmanLocalizationPose
  :super ros::object
  :slots (_header _stateMean _stateCovariance ))

(defmethod rvss_workshop::kalmanLocalizationPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stateMean __stateMean) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateCovariance __stateCovariance) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _stateMean __stateMean)
   (setq _stateCovariance __stateCovariance)
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
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _stateMean
    (* 4    (length _stateMean)) 4
    ;; float32[] _stateCovariance
    (* 4    (length _stateCovariance)) 4
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
   ;;
   self)
  )

(setf (get rvss_workshop::kalmanLocalizationPose :md5sum-) "3241c9839fd6056d6120443c4c3f5013")
(setf (get rvss_workshop::kalmanLocalizationPose :datatype-) "rvss_workshop/kalmanLocalizationPose")
(setf (get rvss_workshop::kalmanLocalizationPose :definition-)
      "Header header
float32[] stateMean
float32[] stateCovariance

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



(provide :rvss_workshop/kalmanLocalizationPose "3241c9839fd6056d6120443c4c3f5013")


