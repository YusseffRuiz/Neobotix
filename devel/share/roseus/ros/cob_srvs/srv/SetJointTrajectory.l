;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::SetJointTrajectory)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'SetJointTrajectory (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::SETJOINTTRAJECTORY")
  (make-package "COB_SRVS::SETJOINTTRAJECTORY"))
(unless (find-package "COB_SRVS::SETJOINTTRAJECTORYREQUEST")
  (make-package "COB_SRVS::SETJOINTTRAJECTORYREQUEST"))
(unless (find-package "COB_SRVS::SETJOINTTRAJECTORYRESPONSE")
  (make-package "COB_SRVS::SETJOINTTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_srvs::SetJointTrajectoryRequest
  :super ros::object
  :slots (_trajectory ))

(defmethod cob_srvs::SetJointTrajectoryRequest
  (:init
   (&key
    ((:trajectory __trajectory) (instance trajectory_msgs::JointTrajectory :init))
    )
   (send-super :init)
   (setq _trajectory __trajectory)
   self)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::SetJointTrajectoryResponse
  :super ros::object
  :slots (_success _errorMessage ))

(defmethod cob_srvs::SetJointTrajectoryResponse
  (:init
   (&key
    ((:success __success) 0)
    ((:errorMessage __errorMessage) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _success (round __success))
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
    ;; int64 _success
    8
    ;; std_msgs/String _errorMessage
    (send _errorMessage :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _success
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _success (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _success) (= (length (_success . bv)) 2)) ;; bignum
              (write-long (ash (elt (_success . bv) 0) 0) s)
              (write-long (ash (elt (_success . bv) 1) -1) s))
             ((and (class _success) (= (length (_success . bv)) 1)) ;; big1
              (write-long (elt (_success . bv) 0) s)
              (write-long (if (>= _success 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _success s)(write-long (if (>= _success 0) 0 #xffffffff) s)))
     ;; std_msgs/String _errorMessage
       (send _errorMessage :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _success
#+(or :alpha :irix6 :x86_64)
      (setf _success (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _success (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; std_msgs/String _errorMessage
     (send _errorMessage :deserialize buf ptr-) (incf ptr- (send _errorMessage :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::SetJointTrajectory
  :super ros::object
  :slots ())

(setf (get cob_srvs::SetJointTrajectory :md5sum-) "098b55597ea8f8573e4c0bab09cb9bce")
(setf (get cob_srvs::SetJointTrajectory :datatype-) "cob_srvs/SetJointTrajectory")
(setf (get cob_srvs::SetJointTrajectory :request) cob_srvs::SetJointTrajectoryRequest)
(setf (get cob_srvs::SetJointTrajectory :response) cob_srvs::SetJointTrajectoryResponse)

(defmethod cob_srvs::SetJointTrajectoryRequest
  (:response () (instance cob_srvs::SetJointTrajectoryResponse :init)))

(setf (get cob_srvs::SetJointTrajectoryRequest :md5sum-) "098b55597ea8f8573e4c0bab09cb9bce")
(setf (get cob_srvs::SetJointTrajectoryRequest :datatype-) "cob_srvs/SetJointTrajectoryRequest")
(setf (get cob_srvs::SetJointTrajectoryRequest :definition-)
      "trajectory_msgs/JointTrajectory trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
int64 success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")

(setf (get cob_srvs::SetJointTrajectoryResponse :md5sum-) "098b55597ea8f8573e4c0bab09cb9bce")
(setf (get cob_srvs::SetJointTrajectoryResponse :datatype-) "cob_srvs/SetJointTrajectoryResponse")
(setf (get cob_srvs::SetJointTrajectoryResponse :definition-)
      "trajectory_msgs/JointTrajectory trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
int64 success
std_msgs/String errorMessage


================================================================================
MSG: std_msgs/String
string data
")



(provide :cob_srvs/SetJointTrajectory "098b55597ea8f8573e4c0bab09cb9bce")


