(require "package://jaxon_ros_interface/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "hrpsys_ext_rtc")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "hrpsys_ext_rtc")

;; CollisionDetectorService
(defmethod rtm-ros-robot-interface
  (:start-collision-detection
   ()
   "Enable collision detection."
   (send self :collisiondetector2service_enablecollisiondetection))
  (:stop-collision-detection
   ()
   "Disable collision detection."
   (send self :collisiondetector2service_disablecollisiondetection))
  )

;; SoftErrorLimiterService
(defmethod rtm-ros-robot-interface
  (:set-soft-error-limit
    (name limit)
    "Set SoftErrorLimiter servo error limit [rad] with given name."
    (send self :softerrorlimiter2service_setservoerrorlimit :name name :limit limit)
    )
  )

;; EmergencyStopperService
(def-set-get-param-method
  'hrpsys_ext_rtc::hrpsys_ext_rtc_EmergencyStopper2Service_EmergencyStopper2Param
  :set-emergency-stopper-param :get-emergency-stopper-param :get-emergency-stopper-param-arguments
  :emergencystopper2service_setemergencystopper2param :emergencystopper2service_getemergencystopper2param)

(defmethod rtm-ros-robot-interface
  (:emergency-stop-motion
   (name)
   "Stop motion emergently in EmergencyStopper."
   (send self :emergencystopper2service_stopmotion :name name))
  (:emergency-release-motion
   (name)
   "Release emergency motion stopping in EmergencyStopper."
   (send self :emergencystopper2service_releasemotion :name name))
  (:emergency-start-torque
   (name)
   "Release emergency motion stopping in EmergencyStopper."
   (send self :emergencystopper2service_starttorque :name name))
  (:emergency-mode
   ()
   "Returns emergency mode."
   (cdr (assoc :emergency-mode robot-state))
   )
  )

