;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetInt)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetInt (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETINT")
  (make-package "COB_SRVS::SETINT"))
(unless (find-package "COB_SRVS::SETINTREQUEST")
  (make-package "COB_SRVS::SETINTREQUEST"))
(unless (find-package "COB_SRVS::SETINTRESPONSE")
  (make-package "COB_SRVS::SETINTRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetIntRequest
  :super ros::object
  :slots (_data ))

(defmethod cob_srvs::SetIntRequest
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cob_srvs::SetIntResponse
  :super ros::object
  :slots (_success _errorMessage ))

(defmethod cob_srvs::SetIntResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:errorMessage __errorMessage) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _errorMessage __errorMessage)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:errorMessage
   (&rest __errorMessage)
   (if (keywordp (car __errorMessage))
       (send* _errorMessage __errorMessage)
     (progn
       (if __errorMessage (setq _errorMessage (car __errorMessage)))
       _errorMessage)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; std_msgs/String _errorMessage
    (send _errorMessage :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; std_msgs/String _errorMessage
       (send _errorMessage :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; std_msgs/String _errorMessage
     (send _errorMessage :deserialize buf ptr-) (incf ptr- (send _errorMessage :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::SetInt
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetInt :md5sum-) "6454c2b313b10b705d6cd215332fc952")
(setf (get cob_srvs::SetInt :datatype-) "cob_srvs/SetInt")
(setf (get cob_srvs::SetInt :request) cob_srvs::SetIntRequest)
(setf (get cob_srvs::SetInt :response) cob_srvs::SetIntResponse)

(defmethod cob_srvs::SetIntRequest
  (:response () (instance cob_srvs::SetIntResponse :init)))

(setf (get cob_srvs::SetIntRequest :md5sum-) "6454c2b313b10b705d6cd215332fc952")
(setf (get cob_srvs::SetIntRequest :datatype-) "cob_srvs/SetIntRequest")
(setf (get cob_srvs::SetIntRequest :definition-)
      "int32 data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetIntResponse :md5sum-) "6454c2b313b10b705d6cd215332fc952")
(setf (get cob_srvs::SetIntResponse :datatype-) "cob_srvs/SetIntResponse")
(setf (get cob_srvs::SetIntResponse :definition-)
      "int32 data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetInt "6454c2b313b10b705d6cd215332fc952")


