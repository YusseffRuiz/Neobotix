; Auto-generated. Do not edit!


(cl:in-package neo_msgs-msg)


;//! \htmlinclude IOBoard.msg.html

(cl:defclass <IOBoard> (roslisp-msg-protocol:ros-message)
  ((digital_inputs
    :reader digital_inputs
    :initarg :digital_inputs
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 16 :element-type 'cl:boolean :initial-element cl:nil))
   (digital_outputs
    :reader digital_outputs
    :initarg :digital_outputs
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 16 :element-type 'cl:boolean :initial-element cl:nil))
   (analog_inputs
    :reader analog_inputs
    :initarg :analog_inputs
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass IOBoard (<IOBoard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IOBoard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IOBoard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neo_msgs-msg:<IOBoard> is deprecated: use neo_msgs-msg:IOBoard instead.")))

(cl:ensure-generic-function 'digital_inputs-val :lambda-list '(m))
(cl:defmethod digital_inputs-val ((m <IOBoard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neo_msgs-msg:digital_inputs-val is deprecated.  Use neo_msgs-msg:digital_inputs instead.")
  (digital_inputs m))

(cl:ensure-generic-function 'digital_outputs-val :lambda-list '(m))
(cl:defmethod digital_outputs-val ((m <IOBoard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neo_msgs-msg:digital_outputs-val is deprecated.  Use neo_msgs-msg:digital_outputs instead.")
  (digital_outputs m))

(cl:ensure-generic-function 'analog_inputs-val :lambda-list '(m))
(cl:defmethod analog_inputs-val ((m <IOBoard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neo_msgs-msg:analog_inputs-val is deprecated.  Use neo_msgs-msg:analog_inputs instead.")
  (analog_inputs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IOBoard>) ostream)
  "Serializes a message object of type '<IOBoard>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'digital_inputs))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'digital_outputs))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'analog_inputs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IOBoard>) istream)
  "Deserializes a message object of type '<IOBoard>"
  (cl:setf (cl:slot-value msg 'digital_inputs) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'digital_inputs)))
    (cl:dotimes (i 16)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'digital_outputs) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'digital_outputs)))
    (cl:dotimes (i 16)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'analog_inputs) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'analog_inputs)))
    (cl:dotimes (i 8)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IOBoard>)))
  "Returns string type for a message object of type '<IOBoard>"
  "neo_msgs/IOBoard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IOBoard)))
  "Returns string type for a message object of type 'IOBoard"
  "neo_msgs/IOBoard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IOBoard>)))
  "Returns md5sum for a message object of type '<IOBoard>"
  "bfa16d6202dd05ce5b3fdebf8a7d44db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IOBoard)))
  "Returns md5sum for a message object of type 'IOBoard"
  "bfa16d6202dd05ce5b3fdebf8a7d44db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IOBoard>)))
  "Returns full string definition for message of type '<IOBoard>"
  (cl:format cl:nil "# This message holds all data available from Neobotix IOBoard~% ~%bool[16] digital_inputs			# state for all digital inputs~%bool[16] digital_outputs		# state for all digital outputs~%~%uint8[8] analog_inputs			# analog input values~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IOBoard)))
  "Returns full string definition for message of type 'IOBoard"
  (cl:format cl:nil "# This message holds all data available from Neobotix IOBoard~% ~%bool[16] digital_inputs			# state for all digital inputs~%bool[16] digital_outputs		# state for all digital outputs~%~%uint8[8] analog_inputs			# analog input values~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IOBoard>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_outputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'analog_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IOBoard>))
  "Converts a ROS message object to a list"
  (cl:list 'IOBoard
    (cl:cons ':digital_inputs (digital_inputs msg))
    (cl:cons ':digital_outputs (digital_outputs msg))
    (cl:cons ':analog_inputs (analog_inputs msg))
))
