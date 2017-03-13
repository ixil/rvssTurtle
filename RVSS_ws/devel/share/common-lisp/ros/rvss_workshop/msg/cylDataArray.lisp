; Auto-generated. Do not edit!


(cl:in-package rvss_workshop-msg)


;//! \htmlinclude cylDataArray.msg.html

(cl:defclass <cylDataArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cylinders
    :reader cylinders
    :initarg :cylinders
    :type (cl:vector rvss_workshop-msg:cylMsg)
   :initform (cl:make-array 0 :element-type 'rvss_workshop-msg:cylMsg :initial-element (cl:make-instance 'rvss_workshop-msg:cylMsg))))
)

(cl:defclass cylDataArray (<cylDataArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cylDataArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cylDataArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvss_workshop-msg:<cylDataArray> is deprecated: use rvss_workshop-msg:cylDataArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <cylDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:header-val is deprecated.  Use rvss_workshop-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cylinders-val :lambda-list '(m))
(cl:defmethod cylinders-val ((m <cylDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:cylinders-val is deprecated.  Use rvss_workshop-msg:cylinders instead.")
  (cylinders m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cylDataArray>) ostream)
  "Serializes a message object of type '<cylDataArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cylinders))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cylinders))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cylDataArray>) istream)
  "Deserializes a message object of type '<cylDataArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cylinders) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cylinders)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rvss_workshop-msg:cylMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cylDataArray>)))
  "Returns string type for a message object of type '<cylDataArray>"
  "rvss_workshop/cylDataArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cylDataArray)))
  "Returns string type for a message object of type 'cylDataArray"
  "rvss_workshop/cylDataArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cylDataArray>)))
  "Returns md5sum for a message object of type '<cylDataArray>"
  "59182e50b0c2a27f7f6f0f07e63a5e57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cylDataArray)))
  "Returns md5sum for a message object of type 'cylDataArray"
  "59182e50b0c2a27f7f6f0f07e63a5e57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cylDataArray>)))
  "Returns full string definition for message of type '<cylDataArray>"
  (cl:format cl:nil "Header header~%cylMsg[] cylinders~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rvss_workshop/cylMsg~%Header header~%float32 Zrobot~%float32 Xrobot~%uint8 label~%float32[] covariance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cylDataArray)))
  "Returns full string definition for message of type 'cylDataArray"
  (cl:format cl:nil "Header header~%cylMsg[] cylinders~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rvss_workshop/cylMsg~%Header header~%float32 Zrobot~%float32 Xrobot~%uint8 label~%float32[] covariance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cylDataArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cylinders) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cylDataArray>))
  "Converts a ROS message object to a list"
  (cl:list 'cylDataArray
    (cl:cons ':header (header msg))
    (cl:cons ':cylinders (cylinders msg))
))
