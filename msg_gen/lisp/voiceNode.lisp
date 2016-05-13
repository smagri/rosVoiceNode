; Auto-generated. Do not edit!


(cl:in-package rosVoiceNode-msg)


;//! \htmlinclude voiceNode.msg.html

(cl:defclass <voiceNode> (roslisp-msg-protocol:ros-message)
  ((txt4TTS
    :reader txt4TTS
    :initarg :txt4TTS
    :type cl:string
    :initform "")
   (voiceName
    :reader voiceName
    :initarg :voiceName
    :type cl:string
    :initform ""))
)

(cl:defclass voiceNode (<voiceNode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <voiceNode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'voiceNode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosVoiceNode-msg:<voiceNode> is deprecated: use rosVoiceNode-msg:voiceNode instead.")))

(cl:ensure-generic-function 'txt4TTS-val :lambda-list '(m))
(cl:defmethod txt4TTS-val ((m <voiceNode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosVoiceNode-msg:txt4TTS-val is deprecated.  Use rosVoiceNode-msg:txt4TTS instead.")
  (txt4TTS m))

(cl:ensure-generic-function 'voiceName-val :lambda-list '(m))
(cl:defmethod voiceName-val ((m <voiceNode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosVoiceNode-msg:voiceName-val is deprecated.  Use rosVoiceNode-msg:voiceName instead.")
  (voiceName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <voiceNode>) ostream)
  "Serializes a message object of type '<voiceNode>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'txt4TTS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'txt4TTS))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'voiceName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'voiceName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <voiceNode>) istream)
  "Deserializes a message object of type '<voiceNode>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'txt4TTS) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'txt4TTS) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'voiceName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'voiceName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<voiceNode>)))
  "Returns string type for a message object of type '<voiceNode>"
  "rosVoiceNode/voiceNode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'voiceNode)))
  "Returns string type for a message object of type 'voiceNode"
  "rosVoiceNode/voiceNode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<voiceNode>)))
  "Returns md5sum for a message object of type '<voiceNode>"
  "0da82377af97bdc608894b2abce81d56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'voiceNode)))
  "Returns md5sum for a message object of type 'voiceNode"
  "0da82377af97bdc608894b2abce81d56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<voiceNode>)))
  "Returns full string definition for message of type '<voiceNode>"
  (cl:format cl:nil "string txt4TTS~%string voiceName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'voiceNode)))
  "Returns full string definition for message of type 'voiceNode"
  (cl:format cl:nil "string txt4TTS~%string voiceName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <voiceNode>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'txt4TTS))
     4 (cl:length (cl:slot-value msg 'voiceName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <voiceNode>))
  "Converts a ROS message object to a list"
  (cl:list 'voiceNode
    (cl:cons ':txt4TTS (txt4TTS msg))
    (cl:cons ':voiceName (voiceName msg))
))
