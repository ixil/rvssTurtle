; Auto-generated. Do not edit!


(cl:in-package rvss_workshop-msg)


;//! \htmlinclude nextPose.msg.html

(cl:defclass <nextPose> (roslisp-msg-protocol:ros-message)
  ((nextPose
    :reader nextPose
    :initarg :nextPose
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass nextPose (<nextPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nextPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nextPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvss_workshop-msg:<nextPose> is deprecated: use rvss_workshop-msg:nextPose instead.")))

(cl:ensure-generic-function 'nextPose-val :lambda-list '(m))
(cl:defmethod nextPose-val ((m <nextPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvss_workshop-msg:nextPose-val is deprecated.  Use rvss_workshop-msg:nextPose instead.")
  (nextPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nextPose>) ostream)
  "Serializes a message object of type '<nextPose>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nextPose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'nextPose))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nextPose>) istream)
  "Deserializes a message object of type '<nextPose>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nextPose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nextPose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nextPose>)))
  "Returns string type for a message object of type '<nextPose>"
  "rvss_workshop/nextPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nextPose)))
  "Returns string type for a message object of type 'nextPose"
  "rvss_workshop/nextPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nextPose>)))
  "Returns md5sum for a message object of type '<nextPose>"
  "472aa9e9259455af5554754a873ea739")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nextPose)))
  "Returns md5sum for a message object of type 'nextPose"
  "472aa9e9259455af5554754a873ea739")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nextPose>)))
  "Returns full string definition for message of type '<nextPose>"
  (cl:format cl:nil "float32[] nextPose~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nextPose)))
  "Returns full string definition for message of type 'nextPose"
  (cl:format cl:nil "float32[] nextPose~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nextPose>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nextPose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nextPose>))
  "Converts a ROS message object to a list"
  (cl:list 'nextPose
    (cl:cons ':nextPose (nextPose msg))
))
