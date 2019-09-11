; Auto-generated. Do not edit!


(cl:in-package belka-gait-commander-msg)


;//! \htmlinclude HipsGaitCommand.msg.html

(cl:defclass <HipsGaitCommand> (roslisp-msg-protocol:ros-message)
  ((hips_command
    :reader hips_command
    :initarg :hips_command
    :type cl:string
    :initform ""))
)

(cl:defclass HipsGaitCommand (<HipsGaitCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HipsGaitCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HipsGaitCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name belka-gait-commander-msg:<HipsGaitCommand> is deprecated: use belka-gait-commander-msg:HipsGaitCommand instead.")))

(cl:ensure-generic-function 'hips_command-val :lambda-list '(m))
(cl:defmethod hips_command-val ((m <HipsGaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader belka-gait-commander-msg:hips_command-val is deprecated.  Use belka-gait-commander-msg:hips_command instead.")
  (hips_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HipsGaitCommand>) ostream)
  "Serializes a message object of type '<HipsGaitCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hips_command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hips_command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HipsGaitCommand>) istream)
  "Deserializes a message object of type '<HipsGaitCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hips_command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hips_command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HipsGaitCommand>)))
  "Returns string type for a message object of type '<HipsGaitCommand>"
  "belka-gait-commander/HipsGaitCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HipsGaitCommand)))
  "Returns string type for a message object of type 'HipsGaitCommand"
  "belka-gait-commander/HipsGaitCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HipsGaitCommand>)))
  "Returns md5sum for a message object of type '<HipsGaitCommand>"
  "3e35c60667a75681d37226fc0b36d0b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HipsGaitCommand)))
  "Returns md5sum for a message object of type 'HipsGaitCommand"
  "3e35c60667a75681d37226fc0b36d0b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HipsGaitCommand>)))
  "Returns full string definition for message of type '<HipsGaitCommand>"
  (cl:format cl:nil "string hips_command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HipsGaitCommand)))
  "Returns full string definition for message of type 'HipsGaitCommand"
  (cl:format cl:nil "string hips_command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HipsGaitCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'hips_command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HipsGaitCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'HipsGaitCommand
    (cl:cons ':hips_command (hips_command msg))
))
