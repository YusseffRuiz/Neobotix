;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetMaxVel)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetMaxVel (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETMAXVEL")
  (make-package "COB_SRVS::SETMAXVEL"))
(unless (find-package "COB_SRVS::SETMAXVELREQUEST")
  (make-package "COB_SRVS::SETMAXVELREQUEST"))
(unless (find-package "COB_SRVS::SETMAXVELRESPONSE")
  (make-package "COB_SRVS::SETMAXVELRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetMaxVelRequest
  :super ros::object
  :slots (_max_vel ))

(defmethod cob_srvs::SetMaxVelRequest
  (:init
   (&key
    ((:max_vel __max_vel) 0.0)
    )
   (send-super :init)
   (setq _max_vel (float __max_vel))
   self)
  (:max_vel
   (&optional __max_vel)
   (if __max_vel (setq _max_vel __max_vel)) _max_vel)
  (:serialization-length
   ()
   (+
    ;; float32 _max_vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _max_vel
       (sys::poke _max_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _max_vel
     (setq _max_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cob_srvs::SetMaxVelResponse
  :super ros::object
  :slots (_success _errorMessage ))

(defmethod cob_srvs::SetMaxVelResponse
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

(defclass cob_srvs::SetMaxVel
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetMaxVel :md5sum-) "5212ea5f4e683819f4e395ad0eb8ae8c")
(setf (get cob_srvs::SetMaxVel :datatype-) "cob_srvs/SetMaxVel")
(setf (get cob_srvs::SetMaxVel :request) cob_srvs::SetMaxVelRequest)
(setf (get cob_srvs::SetMaxVel :response) cob_srvs::SetMaxVelResponse)

(defmethod cob_srvs::SetMaxVelRequest
  (:response () (instance cob_srvs::SetMaxVelResponse :init)))

(setf (get cob_srvs::SetMaxVelRequest :md5sum-) "5212ea5f4e683819f4e395ad0eb8ae8c")
(setf (get cob_srvs::SetMaxVelRequest :datatype-) "cob_srvs/SetMaxVelRequest")
(setf (get cob_srvs::SetMaxVelRequest :definition-)
      "float32 max_vel
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetMaxVelResponse :md5sum-) "5212ea5f4e683819f4e395ad0eb8ae8c")
(setf (get cob_srvs::SetMaxVelResponse :datatype-) "cob_srvs/SetMaxVelResponse")
(setf (get cob_srvs::SetMaxVelResponse :definition-)
      "float32 max_vel
---
bool success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetMaxVel "5212ea5f4e683819f4e395ad0eb8ae8c")


