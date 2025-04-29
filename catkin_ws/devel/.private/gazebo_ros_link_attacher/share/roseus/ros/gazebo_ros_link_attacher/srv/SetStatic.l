;; Auto-generated. Do not edit!


(when (boundp 'gazebo_ros_link_attacher::SetStatic)
  (if (not (find-package "GAZEBO_ROS_LINK_ATTACHER"))
    (make-package "GAZEBO_ROS_LINK_ATTACHER"))
  (shadow 'SetStatic (find-package "GAZEBO_ROS_LINK_ATTACHER")))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATIC")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATIC"))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATICREQUEST")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATICREQUEST"))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATICRESPONSE")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::SETSTATICRESPONSE"))

(in-package "ROS")





(defclass gazebo_ros_link_attacher::SetStaticRequest
  :super ros::object
  :slots (_model_name _link_name _set_static ))

(defmethod gazebo_ros_link_attacher::SetStaticRequest
  (:init
   (&key
    ((:model_name __model_name) "")
    ((:link_name __link_name) "")
    ((:set_static __set_static) nil)
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   (setq _link_name (string __link_name))
   (setq _set_static __set_static)
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:set_static
   (&optional (__set_static :null))
   (if (not (eq __set_static :null)) (setq _set_static __set_static)) _set_static)
  (:serialization-length
   ()
   (+
    ;; string _model_name
    4 (length _model_name)
    ;; string _link_name
    4 (length _link_name)
    ;; bool _set_static
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;; bool _set_static
       (if _set_static (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _set_static
     (setq _set_static (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass gazebo_ros_link_attacher::SetStaticResponse
  :super ros::object
  :slots (_ok ))

(defmethod gazebo_ros_link_attacher::SetStaticResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass gazebo_ros_link_attacher::SetStatic
  :super ros::object
  :slots ())

(setf (get gazebo_ros_link_attacher::SetStatic :md5sum-) "45a69e9f7b70c39951d40d23f04ad0c9")
(setf (get gazebo_ros_link_attacher::SetStatic :datatype-) "gazebo_ros_link_attacher/SetStatic")
(setf (get gazebo_ros_link_attacher::SetStatic :request) gazebo_ros_link_attacher::SetStaticRequest)
(setf (get gazebo_ros_link_attacher::SetStatic :response) gazebo_ros_link_attacher::SetStaticResponse)

(defmethod gazebo_ros_link_attacher::SetStaticRequest
  (:response () (instance gazebo_ros_link_attacher::SetStaticResponse :init)))

(setf (get gazebo_ros_link_attacher::SetStaticRequest :md5sum-) "45a69e9f7b70c39951d40d23f04ad0c9")
(setf (get gazebo_ros_link_attacher::SetStaticRequest :datatype-) "gazebo_ros_link_attacher/SetStaticRequest")
(setf (get gazebo_ros_link_attacher::SetStaticRequest :definition-)
      "string model_name
string link_name
bool set_static
---
bool ok

")

(setf (get gazebo_ros_link_attacher::SetStaticResponse :md5sum-) "45a69e9f7b70c39951d40d23f04ad0c9")
(setf (get gazebo_ros_link_attacher::SetStaticResponse :datatype-) "gazebo_ros_link_attacher/SetStaticResponse")
(setf (get gazebo_ros_link_attacher::SetStaticResponse :definition-)
      "string model_name
string link_name
bool set_static
---
bool ok

")



(provide :gazebo_ros_link_attacher/SetStatic "45a69e9f7b70c39951d40d23f04ad0c9")


