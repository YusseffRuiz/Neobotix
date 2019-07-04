;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetString)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetString (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETSTRING")
  (make-package "COB_SRVS::SETSTRING"))
(unless (find-package "COB_SRVS::SETSTRINGREQUEST")
  (make-package "COB_SRVS::SETSTRINGREQUEST"))
(unless (find-package "COB_SRVS::SETSTRINGRESPONSE")
  (make-package "COB_SRVS::SETSTRINGRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetStringRequest
  :super ros::object
  :slots (_data ))

(defmethod cob_srvs::SetStringRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cob_srvs::SetStringResponse
  :super ros::object
  :slots (_success _errorMessage ))

(defmethod cob_srvs::SetStringResponse
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

(defclass cob_srvs::SetString
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetString :md5sum-) "e1d8c050aed05957d504a68117eb0528")
(setf (get cob_srvs::SetString :datatype-) "cob_srvs/SetString")
(setf (get cob_srvs::SetString :request) cob_srvs::SetStringRequest)
(setf (get cob_srvs::SetString :response) cob_srvs::SetStringResponse)

(defmethod cob_srvs::SetStringRequest
  (:response () (instance cob_srvs::SetStringResponse :init)))

(setf (get cob_srvs::SetStringRequest :md5sum-) "e1d8c050aed05957d504a68117eb0528")
(setf (get cob_srvs::SetStringRequest :datatype-) "cob_srvs/SetStringRequest")
(setf (get cob_srvs::SetStringRequest :definition-)
      "string data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetStringResponse :md5sum-) "e1d8c050aed05957d504a68117eb0528")
(setf (get cob_srvs::SetStringResponse :datatype-) "cob_srvs/SetStringResponse")
(setf (get cob_srvs::SetStringResponse :definition-)
      "string data
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetString "e1d8c050aed05957d504a68117eb0528")


