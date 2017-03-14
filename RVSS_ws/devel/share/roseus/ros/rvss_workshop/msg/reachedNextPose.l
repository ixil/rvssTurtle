;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::reachedNextPose)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'reachedNextPose (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::REACHEDNEXTPOSE")
  (make-package "RVSS_WORKSHOP::REACHEDNEXTPOSE"))

(in-package "ROS")
;;//! \htmlinclude reachedNextPose.msg.html


(defclass rvss_workshop::reachedNextPose
  :super ros::object
  :slots (_reachedNextPose ))

(defmethod rvss_workshop::reachedNextPose
  (:init
   (&key
    ((:reachedNextPose __reachedNextPose) 0)
    )
   (send-super :init)
   (setq _reachedNextPose (round __reachedNextPose))
   self)
  (:reachedNextPose
   (&optional __reachedNextPose)
   (if __reachedNextPose (setq _reachedNextPose __reachedNextPose)) _reachedNextPose)
  (:serialization-length
   ()
   (+
    ;; uint8 _reachedNextPose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _reachedNextPose
       (write-byte _reachedNextPose s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _reachedNextPose
     (setq _reachedNextPose (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rvss_workshop::reachedNextPose :md5sum-) "599100cb215b4875f186eebbd5850cec")
(setf (get rvss_workshop::reachedNextPose :datatype-) "rvss_workshop/reachedNextPose")
(setf (get rvss_workshop::reachedNextPose :definition-)
      "uint8 reachedNextPose

")



(provide :rvss_workshop/reachedNextPose "599100cb215b4875f186eebbd5850cec")


