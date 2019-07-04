;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetOperationMode)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetOperationMode (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETOPERATIONMODE")
  (make-package "COB_SRVS::SETOPERATIONMODE"))
(unless (find-package "COB_SRVS::SETOPERATIONMODEREQUEST")
  (make-package "COB_SRVS::SETOPERATIONMODEREQUEST"))
(unless (find-package "COB_SRVS::SETOPERATIONMODERESPONSE")
  (make-package "COB_SRVS::SETOPERATIONMODERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetOperationModeRequest
  :super ros::object
  :slots (_operation_mode ))

(defmethod cob_srvs::SetOperationModeRequest
  (:init
   (&key
    ((:operation_mode __operation_mode) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _operation_mode __operation_mode)
   self)
  (:operation_mode
   (&rest __operation_mode)
   (if (keywordp (car __operation_mode))
       (send* _operation_mode __operation_mode)
     (progn
       (if __operation_mode (setq _operation_mode (car __operation_mode)))
       _operation_mode)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _operation_mode
    (send _operation_mode :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _operation_mode
       (send _operation_mode :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _operation_mode
     (send _operation_mode :deserialize buf ptr-) (incf ptr- (send _operation_mode :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::SetOperationModeResponse
  :super ros::object
  :slots (_success _error_message ))

(defmethod cob_srvs::SetOperationModeResponse
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

(defclass cob_srvs::SetOperationMode
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetOperationMode :md5sum-) "588a6152f0ed57d725118cc82be8479a")
(setf (get cob_srvs::SetOperationMode :datatype-) "cob_srvs/SetOperationMode")
(setf (get cob_srvs::SetOperationMode :request) cob_srvs::SetOperationModeRequest)
(setf (get cob_srvs::SetOperationMode :response) cob_srvs::SetOperationModeResponse)

(defmethod cob_srvs::SetOperationModeRequest
  (:response () (instance cob_srvs::SetOperationModeResponse :init)))

(setf (get cob_srvs::SetOperationModeRequest :md5sum-) "588a6152f0ed57d725118cc82be8479a")
(setf (get cob_srvs::SetOperationModeRequest :datatype-) "cob_srvs/SetOperationModeRequest")
(setf (get cob_srvs::SetOperationModeRequest :definition-)
      "std_msgs/String operation_mode

================================================================================
MSG: std_msgs/String
string data
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

(setf (get cob_srvs::SetOperationModeResponse :md5sum-) "588a6152f0ed57d725118cc82be8479a")
(setf (get cob_srvs::SetOperationModeResponse :datatype-) "cob_srvs/SetOperationModeResponse")
(setf (get cob_srvs::SetOperationModeResponse :definition-)
      "std_msgs/String operation_mode

================================================================================
MSG: std_msgs/String
string data
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



(provide :cob_srvs/SetOperationMode "588a6152f0ed57d725118cc82be8479a")


