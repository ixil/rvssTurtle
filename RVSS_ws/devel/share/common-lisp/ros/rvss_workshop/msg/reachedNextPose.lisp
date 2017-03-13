; Auto-generated. Do not edit!


(cl:in-package rvss_workshop-msg)


;//! \htmlinclude reachedNextPose.msg.html

(cl:defclass <reachedNextPose> (roslisp-msg-protocol:ros-message)
  ((reachedNextPose
    :reader reachedNextPose
    :initarg :reachedNextPose
    :type cl:fixnum
    :initform 0))
)

(cl:defclass reachedNextPose (<reachedNextPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reachedNextPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reachedNextPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvss_workshop-msg:<reachedNextPose> is deprecated: use rvss_workshop-msg:reachedNextPose instead.")))

(cl:ensure-generic-function 'reachedNextPose-val :lambda-list '(m))
(cl:defmethod reachedNextPose-val ((m <reachedNextPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:reachedNextPose-val is deprecated.  Use rvss_workshop-msg:reachedNextPose instead.")
  (reachedNextPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reachedNextPose>) ostream)
  "Serializes a message object of type '<reachedNextPose>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reachedNextPose)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reachedNextPose>) istream)
  "Deserializes a message object of type '<reachedNextPose>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reachedNextPose)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reachedNextPose>)))
  "Returns string type for a message object of type '<reachedNextPose>"
  "rvss_workshop/reachedNextPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reachedNextPose)))
  "Returns string type for a message object of type 'reachedNextPose"
  "rvss_workshop/reachedNextPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reachedNextPose>)))
  "Returns md5sum for a message object of type '<reachedNextPose>"
  "599100cb215b4875f186eebbd5850cec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reachedNextPose)))
  "Returns md5sum for a message object of type 'reachedNextPose"
  "599100cb215b4875f186eebbd5850cec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reachedNextPose>)))
  "Returns full string definition for message of type '<reachedNextPose>"
  (cl:format cl:nil "uint8 reachedNextPose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reachedNextPose)))
  "Returns full string definition for message of type 'reachedNextPose"
  (cl:format cl:nil "uint8 reachedNextPose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reachedNextPose>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reachedNextPose>))
  "Converts a ROS message object to a list"
  (cl:list 'reachedNextPose
    (cl:cons ':reachedNextPose (reachedNextPose msg))
))
