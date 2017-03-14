;; Auto-generated. Do not edit!


(when (boundp 'rvss_workshop::startstop)
  (if (not (find-package "RVSS_WORKSHOP"))
    (make-package "RVSS_WORKSHOP"))
  (shadow 'startstop (find-package "RVSS_WORKSHOP")))
(unless (find-package "RVSS_WORKSHOP::STARTSTOP")
  (make-package "RVSS_WORKSHOP::STARTSTOP"))

(in-package "ROS")
;;//! \htmlinclude startstop.msg.html


(defclass rvss_workshop::startstop
  :super ros::object
  :slots (_startstop ))

(defmethod rvss_workshop::startstop
  (:init
   (&key
    ((:startstop __startstop) 0)
    )
   (send-super :init)
   (setq _startstop (round __startstop))
   self)
  (:startstop
   (&optional __startstop)
   (if __startstop (setq _startstop __startstop)) _startstop)
  (:serialization-length
   ()
   (+
    ;; uint8 _startstop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _startstop
       (write-byte _startstop s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _startstop
     (setq _startstop (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rvss_workshop::startstop :md5sum-) "221ff8f0d7a331524376c6ba7a726ca6")
(setf (get rvss_workshop::startstop :datatype-) "rvss_workshop/startstop")
(setf (get rvss_workshop::startstop :definition-)
      "uint8 startstop 

")



(provide :rvss_workshop/startstop "221ff8f0d7a331524376c6ba7a726ca6")


