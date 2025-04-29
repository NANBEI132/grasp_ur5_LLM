; Auto-generated. Do not edit!


(cl:in-package gazebo_ros_link_attacher-srv)


;//! \htmlinclude Attach-request.msg.html

(cl:defclass <Attach-request> (roslisp-msg-protocol:ros-message)
  ((model_name_1
    :reader model_name_1
    :initarg :model_name_1
    :type cl:string
    :initform "")
   (link_name_1
    :reader link_name_1
    :initarg :link_name_1
    :type cl:string
    :initform "")
   (model_name_2
    :reader model_name_2
    :initarg :model_name_2
    :type cl:string
    :initform "")
   (link_name_2
    :reader link_name_2
    :initarg :link_name_2
    :type cl:string
    :initform ""))
)

(cl:defclass Attach-request (<Attach-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Attach-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Attach-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_ros_link_attacher-srv:<Attach-request> is deprecated: use gazebo_ros_link_attacher-srv:Attach-request instead.")))

(cl:ensure-generic-function 'model_name_1-val :lambda-list '(m))
(cl:defmethod model_name_1-val ((m <Attach-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:model_name_1-val is deprecated.  Use gazebo_ros_link_attacher-srv:model_name_1 instead.")
  (model_name_1 m))

(cl:ensure-generic-function 'link_name_1-val :lambda-list '(m))
(cl:defmethod link_name_1-val ((m <Attach-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:link_name_1-val is deprecated.  Use gazebo_ros_link_attacher-srv:link_name_1 instead.")
  (link_name_1 m))

(cl:ensure-generic-function 'model_name_2-val :lambda-list '(m))
(cl:defmethod model_name_2-val ((m <Attach-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:model_name_2-val is deprecated.  Use gazebo_ros_link_attacher-srv:model_name_2 instead.")
  (model_name_2 m))

(cl:ensure-generic-function 'link_name_2-val :lambda-list '(m))
(cl:defmethod link_name_2-val ((m <Attach-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:link_name_2-val is deprecated.  Use gazebo_ros_link_attacher-srv:link_name_2 instead.")
  (link_name_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Attach-request>) ostream)
  "Serializes a message object of type '<Attach-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name_1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name_1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name_2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name_2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Attach-request>) istream)
  "Deserializes a message object of type '<Attach-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name_1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name_1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name_1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name_1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name_2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name_2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name_2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name_2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Attach-request>)))
  "Returns string type for a service object of type '<Attach-request>"
  "gazebo_ros_link_attacher/AttachRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Attach-request)))
  "Returns string type for a service object of type 'Attach-request"
  "gazebo_ros_link_attacher/AttachRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Attach-request>)))
  "Returns md5sum for a message object of type '<Attach-request>"
  "c91fb3be70ce66d19130d40294cf4bd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Attach-request)))
  "Returns md5sum for a message object of type 'Attach-request"
  "c91fb3be70ce66d19130d40294cf4bd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Attach-request>)))
  "Returns full string definition for message of type '<Attach-request>"
  (cl:format cl:nil "string model_name_1~%string link_name_1~%string model_name_2~%string link_name_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Attach-request)))
  "Returns full string definition for message of type 'Attach-request"
  (cl:format cl:nil "string model_name_1~%string link_name_1~%string model_name_2~%string link_name_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Attach-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name_1))
     4 (cl:length (cl:slot-value msg 'link_name_1))
     4 (cl:length (cl:slot-value msg 'model_name_2))
     4 (cl:length (cl:slot-value msg 'link_name_2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Attach-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Attach-request
    (cl:cons ':model_name_1 (model_name_1 msg))
    (cl:cons ':link_name_1 (link_name_1 msg))
    (cl:cons ':model_name_2 (model_name_2 msg))
    (cl:cons ':link_name_2 (link_name_2 msg))
))
;//! \htmlinclude Attach-response.msg.html

(cl:defclass <Attach-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Attach-response (<Attach-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Attach-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Attach-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_ros_link_attacher-srv:<Attach-response> is deprecated: use gazebo_ros_link_attacher-srv:Attach-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <Attach-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_ros_link_attacher-srv:ok-val is deprecated.  Use gazebo_ros_link_attacher-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Attach-response>) ostream)
  "Serializes a message object of type '<Attach-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Attach-response>) istream)
  "Deserializes a message object of type '<Attach-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Attach-response>)))
  "Returns string type for a service object of type '<Attach-response>"
  "gazebo_ros_link_attacher/AttachResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Attach-response)))
  "Returns string type for a service object of type 'Attach-response"
  "gazebo_ros_link_attacher/AttachResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Attach-response>)))
  "Returns md5sum for a message object of type '<Attach-response>"
  "c91fb3be70ce66d19130d40294cf4bd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Attach-response)))
  "Returns md5sum for a message object of type 'Attach-response"
  "c91fb3be70ce66d19130d40294cf4bd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Attach-response>)))
  "Returns full string definition for message of type '<Attach-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Attach-response)))
  "Returns full string definition for message of type 'Attach-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Attach-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Attach-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Attach-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Attach)))
  'Attach-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Attach)))
  'Attach-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Attach)))
  "Returns string type for a service object of type '<Attach>"
  "gazebo_ros_link_attacher/Attach")