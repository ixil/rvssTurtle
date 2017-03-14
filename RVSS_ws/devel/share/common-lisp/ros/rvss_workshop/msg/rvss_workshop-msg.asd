
(cl:in-package :asdf)

(defsystem "rvss_workshop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "kalmanState" :depends-on ("_package_kalmanState"))
    (:file "_package_kalmanState" :depends-on ("_package"))
    (:file "cylDataArray" :depends-on ("_package_cylDataArray"))
    (:file "_package_cylDataArray" :depends-on ("_package"))
    (:file "objMsg" :depends-on ("_package_objMsg"))
    (:file "_package_objMsg" :depends-on ("_package"))
    (:file "startstop" :depends-on ("_package_startstop"))
    (:file "_package_startstop" :depends-on ("_package"))
    (:file "kalmanLocalizationState" :depends-on ("_package_kalmanLocalizationState"))
    (:file "_package_kalmanLocalizationState" :depends-on ("_package"))
    (:file "kalmanLocalizationPose" :depends-on ("_package_kalmanLocalizationPose"))
    (:file "_package_kalmanLocalizationPose" :depends-on ("_package"))
    (:file "cylMsg" :depends-on ("_package_cylMsg"))
    (:file "_package_cylMsg" :depends-on ("_package"))
    (:file "reachedNextPose" :depends-on ("_package_reachedNextPose"))
    (:file "_package_reachedNextPose" :depends-on ("_package"))
    (:file "nextPose" :depends-on ("_package_nextPose"))
    (:file "_package_nextPose" :depends-on ("_package"))
    (:file "objDataArray" :depends-on ("_package_objDataArray"))
    (:file "_package_objDataArray" :depends-on ("_package"))
  ))