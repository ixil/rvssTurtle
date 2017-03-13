; Auto-generated. Do not edit!


(cl:in-package rvss_workshop-msg)


;//! \htmlinclude kalmanLocalizationPose.msg.html

(cl:defclass <kalmanLocalizationPose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (stateMean
    :reader stateMean
    :initarg :stateMean
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (stateCovariance
    :reader stateCovariance
    :initarg :stateCovariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass kalmanLocalizationPose (<kalmanLocalizationPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kalmanLocalizationPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kalmanLocalizationPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvss_workshop-msg:<kalmanLocalizationPose> is deprecated: use rvss_workshop-msg:kalmanLocalizationPose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <kalmanLocalizationPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:header-val is deprecated.  Use rvss_workshop-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'stateMean-val :lambda-list '(m))
(cl:defmethod stateMean-val ((m <kalmanLocalizationPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:stateMean-val is deprecated.  Use rvss_workshop-msg:stateMean instead.")
  (stateMean m))

(cl:ensure-generic-function 'stateCovariance-val :lambda-list '(m))
(cl:defmethod stateCovariance-val ((m <kalmanLocalizationPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:stateCovariance-val is deprecated.  Use rvss_workshop-msg:stateCovariance instead.")
  (stateCovariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kalmanLocalizationPose>) ostream)
  "Serializes a message object of type '<kalmanLocalizationPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stateMean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'stateMean))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stateCovariance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'stateCovariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kalmanLocalizationPose>) istream)
  "Deserializes a message object of type '<kalmanLocalizationPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stateMean) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stateMean)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stateCovariance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stateCovariance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kalmanLocalizationPose>)))
  "Returns string type for a message object of type '<kalmanLocalizationPose>"
  "rvss_workshop/kalmanLocalizationPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kalmanLocalizationPose)))
  "Returns string type for a message object of type 'kalmanLocalizationPose"
  "rvss_workshop/kalmanLocalizationPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kalmanLocalizationPose>)))
  "Returns md5sum for a message object of type '<kalmanLocalizationPose>"
  "3241c9839fd6056d6120443c4c3f5013")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kalmanLocalizationPose)))
  "Returns md5sum for a message object of type 'kalmanLocalizationPose"
  "3241c9839fd6056d6120443c4c3f5013")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kalmanLocalizationPose>)))
  "Returns full string definition for message of type '<kalmanLocalizationPose>"
  (cl:format cl:nil "Header header~%float32[] stateMean~%float32[] stateCovariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kalmanLocalizationPose)))
  "Returns full string definition for message of type 'kalmanLocalizationPose"
  (cl:format cl:nil "Header header~%float32[] stateMean~%float32[] stateCovariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kalmanLocalizationPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stateMean) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stateCovariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kalmanLocalizationPose>))
  "Converts a ROS message object to a list"
  (cl:list 'kalmanLocalizationPose
    (cl:cons ':header (header msg))
    (cl:cons ':stateMean (stateMean msg))
    (cl:cons ':stateCovariance (stateCovariance msg))
))
