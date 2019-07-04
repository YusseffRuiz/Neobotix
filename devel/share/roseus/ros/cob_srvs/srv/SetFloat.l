;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetFloat)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetFloat (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETFLOAT")
  (make-package "COB_SRVS::SETFLOAT"))
(unless (find-package "COB_SRVS::SETFLOATREQUEST")
  (make-package "COB_SRVS::SETFLOATREQUEST"))
(unless (find-package "COB_SRVS::SETFLOATRESPONSE")
  (make-package "COB_SRVS::SETFLOATRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetFloatRequest
  :super ros::object
  :slots (_data ))

(defmethod cob_srvs::SetFloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cob_srvs::SetFloatResponse
  :super ros::object
  :slots (_success _errorMessage ))

(defmethod cob_srvs::SetFloatResponse
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

(defclass cob_srvs::SetFloat
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetFloat :md5sum-) "6d377670bf14df6212d27844842dc020")
(setf (get cob_srvs::SetFloat :datatype-) "cob_srvs/SetFloat")
(setf (get cob_srvs::SetFloat :request) cob_srvs::SetFloatRequest)
(setf (get cob_srvs::SetFloat :response) cob_srvs::SetFloatResponse)

(defmethod cob_srvs::SetFloatRequest
  (:response () (instance cob_srvs::SetFloatResponse :init)))

(setf (get cob_srvs::SetFloatRequest :md5sum-) "6d377670bf14df6212d27844842dc020")
(setf (get cob_srvs::SetFloatRequest :datatype-) "cob_srvs/SetFloatRequest")
(setf (get cob_srvs::SetFloatRequest :definition-)
      "float32 data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetFloatResponse :md5sum-) "6d377670bf14df6212d27844842dc020")
(setf (get cob_srvs::SetFloatResponse :datatype-) "cob_srvs/SetFloatResponse")
(setf (get cob_srvs::SetFloatResponse :definition-)
      "float32 data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetFloat "6d377670bf14df6212d27844842dc020")


