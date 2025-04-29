; Auto-generated. Do not edit!


(cl:in-package gazebo_ros_link_attacher-srv)


;//! \htmlinclude SetStatic-request.msg.html

(cl:defclass <SetStatic-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform "")
   (set_static
    :reader set_static
    :initarg :set_static
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetStatic-request (<SetStatic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStatic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStatic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_ros_link_attacher-srv:<SetStatic-request> is deprecated: use gazebo_ros_link_attacher-srv:SetStatic-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <SetStatic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:model_name-val is deprecated.  Use gazebo_ros_link_attacher-srv:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <SetStatic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:link_name-val is deprecated.  Use gazebo_ros_link_attacher-srv:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'set_static-val :lambda-list '(m))
(cl:defmethod set_static-val ((m <SetStatic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:set_static-val is deprecated.  Use gazebo_ros_link_attacher-srv:set_static instead.")
  (set_static m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStatic-request>) ostream)
  "Serializes a message object of type '<SetStatic-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_static) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStatic-request>) istream)
  "Deserializes a message object of type '<SetStatic-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'set_static) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStatic-request>)))
  "Returns string type for a service object of type '<SetStatic-request>"
  "gazebo_ros_link_attacher/SetStaticRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStatic-request)))
  "Returns string type for a service object of type 'SetStatic-request"
  "gazebo_ros_link_attacher/SetStaticRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStatic-request>)))
  "Returns md5sum for a message object of type '<SetStatic-request>"
  "45a69e9f7b70c39951d40d23f04ad0c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStatic-request)))
  "Returns md5sum for a message object of type 'SetStatic-request"
  "45a69e9f7b70c39951d40d23f04ad0c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStatic-request>)))
  "Returns full string definition for message of type '<SetStatic-request>"
  (cl:format cl:nil "string model_name~%string link_name~%bool set_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStatic-request)))
  "Returns full string definition for message of type 'SetStatic-request"
  (cl:format cl:nil "string model_name~%string link_name~%bool set_static~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStatic-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'link_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStatic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStatic-request
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':set_static (set_static msg))
))
;//! \htmlinclude SetStatic-response.msg.html

(cl:defclass <SetStatic-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetStatic-response (<SetStatic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStatic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStatic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_ros_link_attacher-srv:<SetStatic-response> is deprecated: use gazebo_ros_link_attacher-srv:SetStatic-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetStatic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:ok-val is deprecated.  Use gazebo_ros_link_attacher-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStatic-response>) ostream)
  "Serializes a message object of type '<SetStatic-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStatic-response>) istream)
  "Deserializes a message object of type '<SetStatic-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStatic-response>)))
  "Returns string type for a service object of type '<SetStatic-response>"
  "gazebo_ros_link_attacher/SetStaticResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStatic-response)))
  "Returns string type for a service object of type 'SetStatic-response"
  "gazebo_ros_link_attacher/SetStaticResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStatic-response>)))
  "Returns md5sum for a message object of type '<SetStatic-response>"
  "45a69e9f7b70c39951d40d23f04ad0c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStatic-response)))
  "Returns md5sum for a message object of type 'SetStatic-response"
  "45a69e9f7b70c39951d40d23f04ad0c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStatic-response>)))
  "Returns full string definition for message of type '<SetStatic-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStatic-response)))
  "Returns full string definition for message of type 'SetStatic-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStatic-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStatic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStatic-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStatic)))
  'SetStatic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStatic)))
  'SetStatic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStatic)))
  "Returns string type for a service object of type '<SetStatic>"
  "gazebo_ros_link_attacher/SetStatic")