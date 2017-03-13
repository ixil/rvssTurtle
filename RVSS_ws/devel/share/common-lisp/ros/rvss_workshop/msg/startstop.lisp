; Auto-generated. Do not edit!


(cl:in-package rvss_workshop-msg)


;//! \htmlinclude startstop.msg.html

(cl:defclass <startstop> (roslisp-msg-protocol:ros-message)
  ((startstop
    :reader startstop
    :initarg :startstop
    :type cl:fixnum
    :initform 0))
)

(cl:defclass startstop (<startstop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <startstop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'startstop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvss_workshop-msg:<startstop> is deprecated: use rvss_workshop-msg:startstop instead.")))

(cl:ensure-generic-function 'startstop-val :lambda-list '(m))
(cl:defmethod startstop-val ((m <startstop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:startstop-val is deprecated.  Use rvss_workshop-msg:startstop instead.")
  (startstop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <startstop>) ostream)
  "Serializes a message object of type '<startstop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'startstop)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <startstop>) istream)
  "Deserializes a message object of type '<startstop>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'startstop)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<startstop>)))
  "Returns string type for a message object of type '<startstop>"
  "rvss_workshop/startstop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'startstop)))
  "Returns string type for a message object of type 'startstop"
  "rvss_workshop/startstop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<startstop>)))
  "Returns md5sum for a message object of type '<startstop>"
  "221ff8f0d7a331524376c6ba7a726ca6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'startstop)))
  "Returns md5sum for a message object of type 'startstop"
  "221ff8f0d7a331524376c6ba7a726ca6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<startstop>)))
  "Returns full string definition for message of type '<startstop>"
  (cl:format cl:nil "uint8 startstop ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'startstop)))
  "Returns full string definition for message of type 'startstop"
  (cl:format cl:nil "uint8 startstop ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <startstop>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <startstop>))
  "Converts a ROS message object to a list"
  (cl:list 'startstop
    (cl:cons ':startstop (startstop msg))
))
