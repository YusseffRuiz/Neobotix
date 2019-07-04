;; Auto-generated. Do not edit!


(when (boundp 'neo_msgs::IOBoard)
  (if (not (find-package "NEO_MSGS"))
    (make-package "NEO_MSGS"))
  (shadow 'IOBoard (find-package "NEO_MSGS")))
(unless (find-package "NEO_MSGS::IOBOARD")
  (make-package "NEO_MSGS::IOBOARD"))

(in-package "ROS")
;;//! \htmlinclude IOBoard.msg.html


(defclass neo_msgs::IOBoard
  :super ros::object
  :slots (_digital_inputs _digital_outputs _analog_inputs ))

(defmethod neo_msgs::IOBoard
  (:init
   (&key
    ((:digital_inputs __digital_inputs) (let (r) (dotimes (i 16) (push nil r)) r))
    ((:digital_outputs __digital_outputs) (let (r) (dotimes (i 16) (push nil r)) r))
    ((:analog_inputs __analog_inputs) (make-array 8 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _digital_inputs __digital_inputs)
   (setq _digital_outputs __digital_outputs)
   (setq _analog_inputs __analog_inputs)
   self)
  (:digital_inputs
   (&optional __digital_inputs)
   (if __digital_inputs (setq _digital_inputs __digital_inputs)) _digital_inputs)
  (:digital_outputs
   (&optional __digital_outputs)
   (if __digital_outputs (setq _digital_outputs __digital_outputs)) _digital_outputs)
  (:analog_inputs
   (&optional __analog_inputs)
   (if __analog_inputs (setq _analog_inputs __analog_inputs)) _analog_inputs)
  (:serialization-length
   ()
   (+
    ;; bool[16] _digital_inputs
    (* 1    16)
    ;; bool[16] _digital_outputs
    (* 1    16)
    ;; uint8[8] _analog_inputs
    (* 1    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool[16] _digital_inputs
     (dotimes (i 16)
       (if (elt _digital_inputs i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[16] _digital_outputs
     (dotimes (i 16)
       (if (elt _digital_outputs i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint8[8] _analog_inputs
     (princ _analog_inputs s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool[16] _digital_inputs
   (dotimes (i (length _digital_inputs))
     (setf (elt _digital_inputs i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[16] _digital_outputs
   (dotimes (i (length _digital_outputs))
     (setf (elt _digital_outputs i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; uint8[8] _analog_inputs
   (setq _analog_inputs (make-array 8 :element-type :char))
   (replace _analog_inputs buf :start2 ptr-) (incf ptr- 8)
   ;;
   self)
  )

(setf (get neo_msgs::IOBoard :md5sum-) "bfa16d6202dd05ce5b3fdebf8a7d44db")
(setf (get neo_msgs::IOBoard :datatype-) "neo_msgs/IOBoard")
(setf (get neo_msgs::IOBoard :definition-)
      "# This message holds all data available from Neobotix IOBoard
 
bool[16] digital_inputs			# state for all digital inputs
bool[16] digital_outputs		# state for all digital outputs

uint8[8] analog_inputs			# analog input values



")



(provide :neo_msgs/IOBoard "bfa16d6202dd05ce5b3fdebf8a7d44db")


