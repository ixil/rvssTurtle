;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::nextPose)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'nextPose (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::NEXTPOSE")
  (make-package "RVSS_WORKSHOP::NEXTPOSE"))

(in-package "ROS")
;;//! \htmlinclude nextPose.msg.html


(defclass rvss_workshop::nextPose
  :super ros::object
  :slots (_nextPose ))

(defmethod rvss_workshop::nextPose
  (:init
   (&key
    ((:nextPose __nextPose) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _nextPose __nextPose)
   self)
  (:nextPose
   (&optional __nextPose)
   (if __nextPose (setq _nextPose __nextPose)) _nextPose)
  (:serialization-length
   ()
   (+
    ;; float32[] _nextPose
    (* 4    (length _nextPose)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _nextPose
     (write-long (length _nextPose) s)
     (dotimes (i (length _nextPose))
       (sys::poke (elt _nextPose i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _nextPose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nextPose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _nextPose i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get rvss_workshop::nextPose :md5sum-) "472aa9e9259455af5554754a873ea739")
(setf (get rvss_workshop::nextPose :datatype-) "rvss_workshop/nextPose")
(setf (get rvss_workshop::nextPose :definition-)
      "float32[] nextPose
")



(provide :rvss_workshop/nextPose "472aa9e9259455af5554754a873ea739")


