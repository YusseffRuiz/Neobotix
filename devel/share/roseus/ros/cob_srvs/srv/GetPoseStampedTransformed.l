;; Auto-generated. Do not edit!


(when (boundp 'cob_srvs::GetPoseStampedTransformed)
  (if (not (find-package "COB_SRVS"))
    (make-package "COB_SRVS"))
  (shadow 'GetPoseStampedTransformed (find-package "COB_SRVS")))
(unless (find-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMED")
  (make-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMED"))
(unless (find-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMEDREQUEST")
  (make-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMEDREQUEST"))
(unless (find-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMEDRESPONSE")
  (make-package "COB_SRVS::GETPOSESTAMPEDTRANSFORMEDRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cob_srvs::GetPoseStampedTransformedRequest
  :super ros::object
  :slots (_target _origin _orientation_override _root_name _tip_name ))

(defmethod cob_srvs::GetPoseStampedTransformedRequest
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::PoseStamped :init))
    ((:origin __origin) (instance geometry_msgs::PoseStamped :init))
    ((:orientation_override __orientation_override) (instance geometry_msgs::Quaternion :init))
    ((:root_name __root_name) "")
    ((:tip_name __tip_name) "")
    )
   (send-super :init)
   (setq _target __target)
   (setq _origin __origin)
   (setq _orientation_override __orientation_override)
   (setq _root_name (string __root_name))
   (setq _tip_name (string __tip_name))
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:orientation_override
   (&rest __orientation_override)
   (if (keywordp (car __orientation_override))
       (send* _orientation_override __orientation_override)
     (progn
       (if __orientation_override (setq _orientation_override (car __orientation_override)))
       _orientation_override)))
  (:root_name
   (&optional __root_name)
   (if __root_name (setq _root_name __root_name)) _root_name)
  (:tip_name
   (&optional __tip_name)
   (if __tip_name (setq _tip_name __tip_name)) _tip_name)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _target
    (send _target :serialization-length)
    ;; geometry_msgs/PoseStamped _origin
    (send _origin :serialization-length)
    ;; geometry_msgs/Quaternion _orientation_override
    (send _orientation_override :serialization-length)
    ;; string _root_name
    4 (length _root_name)
    ;; string _tip_name
    4 (length _tip_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _target
       (send _target :serialize s)
     ;; geometry_msgs/PoseStamped _origin
       (send _origin :serialize s)
     ;; geometry_msgs/Quaternion _orientation_override
       (send _orientation_override :serialize s)
     ;; string _root_name
       (write-long (length _root_name) s) (princ _root_name s)
     ;; string _tip_name
       (write-long (length _tip_name) s) (princ _tip_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; geometry_msgs/PoseStamped _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; geometry_msgs/Quaternion _orientation_override
     (send _orientation_override :deserialize buf ptr-) (incf ptr- (send _orientation_override :serialization-length))
   ;; string _root_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _root_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _tip_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tip_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cob_srvs::GetPoseStampedTransformedResponse
  :super ros::object
  :slots (_success _result ))

(defmethod cob_srvs::GetPoseStampedTransformedResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:result __result) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _result __result)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:result
   (&rest __result)
   (if (keywordp (car __result))
       (send* _result __result)
     (progn
       (if __result (setq _result (car __result)))
       _result)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; geometry_msgs/PoseStamped _result
    (send _result :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/PoseStamped _result
       (send _result :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/PoseStamped _result
     (send _result :deserialize buf ptr-) (incf ptr- (send _result :serialization-length))
   ;;
   self)
  )

(defclass cob_srvs::GetPoseStampedTransformed
  :super ros::object
  :slots ())

(setf (get cob_srvs::GetPoseStampedTransformed :md5sum-) "ed0069a0a6ce5102fc39c3328eef052f")
(setf (get cob_srvs::GetPoseStampedTransformed :datatype-) "cob_srvs/GetPoseStampedTransformed")
(setf (get cob_srvs::GetPoseStampedTransformed :request) cob_srvs::GetPoseStampedTransformedRequest)
(setf (get cob_srvs::GetPoseStampedTransformed :response) cob_srvs::GetPoseStampedTransformedResponse)

(defmethod cob_srvs::GetPoseStampedTransformedRequest
  (:response () (instance cob_srvs::GetPoseStampedTransformedResponse :init)))

(setf (get cob_srvs::GetPoseStampedTransformedRequest :md5sum-) "ed0069a0a6ce5102fc39c3328eef052f")
(setf (get cob_srvs::GetPoseStampedTransformedRequest :datatype-) "cob_srvs/GetPoseStampedTransformedRequest")
(setf (get cob_srvs::GetPoseStampedTransformedRequest :definition-)
      "geometry_msgs/PoseStamped target
geometry_msgs/PoseStamped origin
geometry_msgs/Quaternion  orientation_override
string root_name
string tip_name

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
geometry_msgs/PoseStamped result


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get cob_srvs::GetPoseStampedTransformedResponse :md5sum-) "ed0069a0a6ce5102fc39c3328eef052f")
(setf (get cob_srvs::GetPoseStampedTransformedResponse :datatype-) "cob_srvs/GetPoseStampedTransformedResponse")
(setf (get cob_srvs::GetPoseStampedTransformedResponse :definition-)
      "geometry_msgs/PoseStamped target
geometry_msgs/PoseStamped origin
geometry_msgs/Quaternion  orientation_override
string root_name
string tip_name

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
geometry_msgs/PoseStamped result


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :cob_srvs/GetPoseStampedTransformed "ed0069a0a6ce5102fc39c3328eef052f")


