;; Auto-generated. Do not edit!


(when (boundp 'belka-gait-commander::HipsGaitCommand)
  (if (not (find-package "BELKA-GAIT-COMMANDER"))
    (make-package "BELKA-GAIT-COMMANDER"))
  (shadow 'HipsGaitCommand (find-package "BELKA-GAIT-COMMANDER")))
(unless (find-package "BELKA-GAIT-COMMANDER::HIPSGAITCOMMAND")
  (make-package "BELKA-GAIT-COMMANDER::HIPSGAITCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude HipsGaitCommand.msg.html


(defclass belka-gait-commander::HipsGaitCommand
  :super ros::object
  :slots (_hips_command ))

(defmethod belka-gait-commander::HipsGaitCommand
  (:init
   (&key
    ((:hips_command __hips_command) "")
    )
   (send-super :init)
   (setq _hips_command (string __hips_command))
   self)
  (:hips_command
   (&optional __hips_command)
   (if __hips_command (setq _hips_command __hips_command)) _hips_command)
  (:serialization-length
   ()
   (+
    ;; string _hips_command
    4 (length _hips_command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _hips_command
       (write-long (length _hips_command) s) (princ _hips_command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _hips_command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hips_command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get belka-gait-commander::HipsGaitCommand :md5sum-) "3e35c60667a75681d37226fc0b36d0b6")
(setf (get belka-gait-commander::HipsGaitCommand :datatype-) "belka-gait-commander/HipsGaitCommand")
(setf (get belka-gait-commander::HipsGaitCommand :definition-)
      "string hips_command
")



(provide :belka-gait-commander/HipsGaitCommand "3e35c60667a75681d37226fc0b36d0b6")


