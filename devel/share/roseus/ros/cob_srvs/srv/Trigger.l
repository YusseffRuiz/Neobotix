;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::Trigger)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'Trigger (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::TRIGGER")
  (make-package "COB_SRVS::TRIGGER"))
(unless (find-package "COB_SRVS::TRIGGERREQUEST")
  (make-package "COB_SRVS::TRIGGERREQUEST"))
(unless (find-package "COB_SRVS::TRIGGERRESPONSE")
  (make-package "COB_SRVS::TRIGGERRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::TriggerRequest
  :super ros::object
  :slots ())

(defmethod cob_srvs::TriggerRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cob_srvs::TriggerResponse
  :super ros::object
  :slots (_success _error_message ))

(defmethod cob_srvs::TriggerResponse
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

(defclass cob_srvs::Trigger
  :super ros::object
  :slots ())

(setf (get cob_srvs::Trigger :md5sum-) "bdad515e8f4c013bd650d3407753d2b8")
(setf (get cob_srvs::Trigger :datatype-) "cob_srvs/Trigger")
(setf (get cob_srvs::Trigger :request) cob_srvs::TriggerRequest)
(setf (get cob_srvs::Trigger :response) cob_srvs::TriggerResponse)

(defmethod cob_srvs::TriggerRequest
  (:response () (instance cob_srvs::TriggerResponse :init)))

(setf (get cob_srvs::TriggerRequest :md5sum-) "bdad515e8f4c013bd650d3407753d2b8")
(setf (get cob_srvs::TriggerRequest :datatype-) "cob_srvs/TriggerRequest")
(setf (get cob_srvs::TriggerRequest :definition-)
      "---
std_msgs/Bool success
std_msgs/String error_message


================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::TriggerResponse :md5sum-) "bdad515e8f4c013bd650d3407753d2b8")
(setf (get cob_srvs::TriggerResponse :datatype-) "cob_srvs/TriggerResponse")
(setf (get cob_srvs::TriggerResponse :definition-)
      "---
std_msgs/Bool success
std_msgs/String error_message


================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/Trigger "bdad515e8f4c013bd650d3407753d2b8")


