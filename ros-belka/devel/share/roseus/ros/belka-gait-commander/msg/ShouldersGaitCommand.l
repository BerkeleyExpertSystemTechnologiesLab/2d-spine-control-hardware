;; Auto-generated. Do not edit!


(when (boundp 'belka-gait-commander::ShouldersGaitCommand)
  (if (not (find-package "BELKA-GAIT-COMMANDER"))
    (make-package "BELKA-GAIT-COMMANDER"))
  (shadow 'ShouldersGaitCommand (find-package "BELKA-GAIT-COMMANDER")))
(unless (find-package "BELKA-GAIT-COMMANDER::SHOULDERSGAITCOMMAND")
  (make-package "BELKA-GAIT-COMMANDER::SHOULDERSGAITCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ShouldersGaitCommand.msg.html


(defclass belka-gait-commander::ShouldersGaitCommand
  :super ros::object
  :slots (_shoulders_command ))

(defmethod belka-gait-commander::ShouldersGaitCommand
  (:init
   (&key
    ((:shoulders_command __shoulders_command) "")
    )
   (send-super :init)
   (setq _shoulders_command (string __shoulders_command))
   self)
  (:shoulders_command
   (&optional __shoulders_command)
   (if __shoulders_command (setq _shoulders_command __shoulders_command)) _shoulders_command)
  (:serialization-length
   ()
   (+
    ;; string _shoulders_command
    4 (length _shoulders_command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _shoulders_command
       (write-long (length _shoulders_command) s) (princ _shoulders_command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _shoulders_command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _shoulders_command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get belka-gait-commander::ShouldersGaitCommand :md5sum-) "ba9308f196c49d06e4333c12eec0f361")
(setf (get belka-gait-commander::ShouldersGaitCommand :datatype-) "belka-gait-commander/ShouldersGaitCommand")
(setf (get belka-gait-commander::ShouldersGaitCommand :definition-)
      "string shoulders_command
")



(provide :belka-gait-commander/ShouldersGaitCommand "ba9308f196c49d06e4333c12eec0f361")


