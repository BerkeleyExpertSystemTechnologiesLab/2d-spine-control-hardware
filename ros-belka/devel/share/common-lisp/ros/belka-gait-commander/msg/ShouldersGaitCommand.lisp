; Auto-generated. Do not edit!


(cl:in-package belka-gait-commander-msg)


;//! \htmlinclude ShouldersGaitCommand.msg.html

(cl:defclass <ShouldersGaitCommand> (roslisp-msg-protocol:ros-message)
  ((shoulders_command
    :reader shoulders_command
    :initarg :shoulders_command
    :type cl:string
    :initform ""))
)

(cl:defclass ShouldersGaitCommand (<ShouldersGaitCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShouldersGaitCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShouldersGaitCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name belka-gait-commander-msg:<ShouldersGaitCommand> is deprecated: use belka-gait-commander-msg:ShouldersGaitCommand instead.")))

(cl:ensure-generic-function 'shoulders_command-val :lambda-list '(m))
(cl:defmethod shoulders_command-val ((m <ShouldersGaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader belka-gait-commander-msg:shoulders_command-val is deprecated.  Use belka-gait-commander-msg:shoulders_command instead.")
  (shoulders_command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShouldersGaitCommand>) ostream)
  "Serializes a message object of type '<ShouldersGaitCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'shoulders_command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'shoulders_command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShouldersGaitCommand>) istream)
  "Deserializes a message object of type '<ShouldersGaitCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shoulders_command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'shoulders_command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShouldersGaitCommand>)))
  "Returns string type for a message object of type '<ShouldersGaitCommand>"
  "belka-gait-commander/ShouldersGaitCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShouldersGaitCommand)))
  "Returns string type for a message object of type 'ShouldersGaitCommand"
  "belka-gait-commander/ShouldersGaitCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShouldersGaitCommand>)))
  "Returns md5sum for a message object of type '<ShouldersGaitCommand>"
  "ba9308f196c49d06e4333c12eec0f361")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShouldersGaitCommand)))
  "Returns md5sum for a message object of type 'ShouldersGaitCommand"
  "ba9308f196c49d06e4333c12eec0f361")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShouldersGaitCommand>)))
  "Returns full string definition for message of type '<ShouldersGaitCommand>"
  (cl:format cl:nil "string shoulders_command~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShouldersGaitCommand)))
  "Returns full string definition for message of type 'ShouldersGaitCommand"
  (cl:format cl:nil "string shoulders_command~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShouldersGaitCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'shoulders_command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShouldersGaitCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ShouldersGaitCommand
    (cl:cons ':shoulders_command (shoulders_command msg))
))
