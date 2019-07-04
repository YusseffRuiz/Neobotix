;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetDefaultVel)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetDefaultVel (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETDEFAULTVEL")
  (make-package "COB_SRVS::SETDEFAULTVEL"))
(unless (find-package "COB_SRVS::SETDEFAULTVELREQUEST")
  (make-package "COB_SRVS::SETDEFAULTVELREQUEST"))
(unless (find-package "COB_SRVS::SETDEFAULTVELRESPONSE")
  (make-package "COB_SRVS::SETDEFAULTVELRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetDefaultVelRequest
  :super ros::object
  :slots (_default_vel ))

(defmethod cob_srvs::SetDefaultVelRequest
  (:init
   (&key
    ((:default_vel __default_vel) 0.0)
    )
   (send-super :init)
   (setq _default_vel (float __default_vel))
   self)
  (:default_vel
   (&optional __default_vel)
   (if __default_vel (setq _default_vel __default_vel)) _default_vel)
  (:serialization-length
   ()
   (+
    ;; float32 _default_vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _default_vel
       (sys::poke _default_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _default_vel
     (setq _default_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cob_srvs::SetDefaultVelResponse
  :super ros::object
  :slots (_success _error_message ))

(defmethod cob_srvs::SetDefaultVelResponse
  (:init
   (&key
    ((:success __success) (instance std_msgs::Bool :init))
    ((:error_message __error_message) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _error_message __error_message)
   self)
  (:success
   (&rest __success)
   (if (keywordp (car __success))
       (send* _success __success)
     (progn
       (if __success (setq _success (car __success)))
       _success)))
  (:error_message
   (&rest __error_message)
   (if (keywordp (car __error_message))
       (send* _error_message __error_message)
     (progn
       (if __error_message (setq _error_message (car __error_message)))
       _error_message)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Bool _success
    (send _success :serialization-length)
    ;; std_msgs/String _error_message
    (send _error_message :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Bool _success
       (send _success :serialize s)
     ;; std_msgs/String _error_message
       (send _error_message :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Bool _success
     (send _success :deserialize buf ptr-) (incf ptr- (send _success :serialization-length))
   ;; std_msgs/String _error_message
     (send _error_message :deserialize buf ptr-) (incf ptr- (send _error_message :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::SetDefaultVel
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetDefaultVel :md5sum-) "a7159156625f755343070ad39bfaf3a3")
(setf (get cob_srvs::SetDefaultVel :datatype-) "cob_srvs/SetDefaultVel")
(setf (get cob_srvs::SetDefaultVel :request) cob_srvs::SetDefaultVelRequest)
(setf (get cob_srvs::SetDefaultVel :response) cob_srvs::SetDefaultVelResponse)

(defmethod cob_srvs::SetDefaultVelRequest
  (:response () (instance cob_srvs::SetDefaultVelResponse :init)))

(setf (get cob_srvs::SetDefaultVelRequest :md5sum-) "a7159156625f755343070ad39bfaf3a3")
(setf (get cob_srvs::SetDefaultVelRequest :datatype-) "cob_srvs/SetDefaultVelRequest")
(setf (get cob_srvs::SetDefaultVelRequest :definition-)
      "float32 default_vel
---
std_msgs/Bool success
std_msgs/String error_message


================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetDefaultVelResponse :md5sum-) "a7159156625f755343070ad39bfaf3a3")
(setf (get cob_srvs::SetDefaultVelResponse :datatype-) "cob_srvs/SetDefaultVelResponse")
(setf (get cob_srvs::SetDefaultVelResponse :definition-)
      "float32 default_vel
---
std_msgs/Bool success
std_msgs/String error_message


================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetDefaultVel "a7159156625f755343070ad39bfaf3a3")


