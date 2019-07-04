;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetJointStiffness)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetJointStiffness (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETJOINTSTIFFNESS")
  (make-package "COB_SRVS::SETJOINTSTIFFNESS"))
(unless (find-package "COB_SRVS::SETJOINTSTIFFNESSREQUEST")
  (make-package "COB_SRVS::SETJOINTSTIFFNESSREQUEST"))
(unless (find-package "COB_SRVS::SETJOINTSTIFFNESSRESPONSE")
  (make-package "COB_SRVS::SETJOINTSTIFFNESSRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetJointStiffnessRequest
  :super ros::object
  :slots (_joint_stiffness ))

(defmethod cob_srvs::SetJointStiffnessRequest
  (:init
   (&key
    ((:joint_stiffness __joint_stiffness) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_stiffness __joint_stiffness)
   self)
  (:joint_stiffness
   (&optional __joint_stiffness)
   (if __joint_stiffness (setq _joint_stiffness __joint_stiffness)) _joint_stiffness)
  (:serialization-length
   ()
   (+
    ;; float32[] _joint_stiffness
    (* 4    (length _joint_stiffness)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _joint_stiffness
     (write-long (length _joint_stiffness) s)
     (dotimes (i (length _joint_stiffness))
       (sys::poke (elt _joint_stiffness i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _joint_stiffness
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_stiffness (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_stiffness i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass cob_srvs::SetJointStiffnessResponse
  :super ros::object
  :slots (_success _error_message ))

(defmethod cob_srvs::SetJointStiffnessResponse
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

(defclass cob_srvs::SetJointStiffness
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetJointStiffness :md5sum-) "2ae9ddd5d3294ccf3c876d966e4e11a3")
(setf (get cob_srvs::SetJointStiffness :datatype-) "cob_srvs/SetJointStiffness")
(setf (get cob_srvs::SetJointStiffness :request) cob_srvs::SetJointStiffnessRequest)
(setf (get cob_srvs::SetJointStiffness :response) cob_srvs::SetJointStiffnessResponse)

(defmethod cob_srvs::SetJointStiffnessRequest
  (:response () (instance cob_srvs::SetJointStiffnessResponse :init)))

(setf (get cob_srvs::SetJointStiffnessRequest :md5sum-) "2ae9ddd5d3294ccf3c876d966e4e11a3")
(setf (get cob_srvs::SetJointStiffnessRequest :datatype-) "cob_srvs/SetJointStiffnessRequest")
(setf (get cob_srvs::SetJointStiffnessRequest :definition-)
      "float32[] joint_stiffness
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

(setf (get cob_srvs::SetJointStiffnessResponse :md5sum-) "2ae9ddd5d3294ccf3c876d966e4e11a3")
(setf (get cob_srvs::SetJointStiffnessResponse :datatype-) "cob_srvs/SetJointStiffnessResponse")
(setf (get cob_srvs::SetJointStiffnessResponse :definition-)
      "float32[] joint_stiffness
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



(provide :cob_srvs/SetJointStiffness "2ae9ddd5d3294ccf3c876d966e4e11a3")


