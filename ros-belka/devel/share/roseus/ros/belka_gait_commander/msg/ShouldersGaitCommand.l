;; Auto-generated. Do not edit!


(when (boundp 'belka_gait_commander::ShouldersGaitCommand)
  (if (not (find-package "BELKA_GAIT_COMMANDER"))
    (make-package "BELKA_GAIT_COMMANDER"))
  (shadow 'ShouldersGaitCommand (find-package "BELKA_GAIT_COMMANDER")))
(unless (find-package "BELKA_GAIT_COMMANDER::SHOULDERSGAITCOMMAND")
  (make-package "BELKA_GAIT_COMMANDER::SHOULDERSGAITCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ShouldersGaitCommand.msg.html


(defclass belka_gait_commander::ShouldersGaitCommand
  :super ros::object
  :slots (_shoulders_command ))

(defmethod belka_gait_commander::ShouldersGaitCommand
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

(setf (get belka_gait_commander::ShouldersGaitCommand :md5sum-) "ba9308f196c49d06e4333c12eec0f361")
(setf (get belka_gait_commander::ShouldersGaitCommand :datatype-) "belka_gait_commander/ShouldersGaitCommand")
(setf (get belka_gait_commander::ShouldersGaitCommand :definition-)
      "string shoulders_command
")



(provide :belka_gait_commander/ShouldersGaitCommand "ba9308f196c49d06e4333c12eec0f361")


