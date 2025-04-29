;; Auto-generated. Do not edit!


(when (boundp 'gazebo_ros_link_attacher::Attach)
  (if (not (find-package "GAZEBO_ROS_LINK_ATTACHER"))
    (make-package "GAZEBO_ROS_LINK_ATTACHER"))
  (shadow 'Attach (find-package "GAZEBO_ROS_LINK_ATTACHER")))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::ATTACH")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::ATTACH"))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::ATTACHREQUEST")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::ATTACHREQUEST"))
(unless (find-package "GAZEBO_ROS_LINK_ATTACHER::ATTACHRESPONSE")
  (make-package "GAZEBO_ROS_LINK_ATTACHER::ATTACHRESPONSE"))

(in-package "ROS")





(defclass gazebo_ros_link_attacher::AttachRequest
  :super ros::object
  :slots (_model_name_1 _link_name_1 _model_name_2 _link_name_2 ))

(defmethod gazebo_ros_link_attacher::AttachRequest
  (:init
   (&key
    ((:model_name_1 __model_name_1) "")
    ((:link_name_1 __link_name_1) "")
    ((:model_name_2 __model_name_2) "")
    ((:link_name_2 __link_name_2) "")
    )
   (send-super :init)
   (setq _model_name_1 (string __model_name_1))
   (setq _link_name_1 (string __link_name_1))
   (setq _model_name_2 (string __model_name_2))
   (setq _link_name_2 (string __link_name_2))
   self)
  (:model_name_1
   (&optional __model_name_1)
   (if __model_name_1 (setq _model_name_1 __model_name_1)) _model_name_1)
  (:link_name_1
   (&optional __link_name_1)
   (if __link_name_1 (setq _link_name_1 __link_name_1)) _link_name_1)
  (:model_name_2
   (&optional __model_name_2)
   (if __model_name_2 (setq _model_name_2 __model_name_2)) _model_name_2)
  (:link_name_2
   (&optional __link_name_2)
   (if __link_name_2 (setq _link_name_2 __link_name_2)) _link_name_2)
  (:serialization-length
   ()
   (+
    ;; string _model_name_1
    4 (length _model_name_1)
    ;; string _link_name_1
    4 (length _link_name_1)
    ;; string _model_name_2
    4 (length _model_name_2)
    ;; string _link_name_2
    4 (length _link_name_2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name_1
       (write-long (length _model_name_1) s) (princ _model_name_1 s)
     ;; string _link_name_1
       (write-long (length _link_name_1) s) (princ _link_name_1 s)
     ;; string _model_name_2
       (write-long (length _model_name_2) s) (princ _model_name_2 s)
     ;; string _link_name_2
       (write-long (length _link_name_2) s) (princ _link_name_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name_1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name_1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _link_name_1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name_1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _model_name_2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name_2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _link_name_2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name_2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gazebo_ros_link_attacher::AttachResponse
  :super ros::object
  :slots (_ok ))

(defmethod gazebo_ros_link_attacher::AttachResponse
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

(defclass gazebo_ros_link_attacher::Attach
  :super ros::object
  :slots ())

(setf (get gazebo_ros_link_attacher::Attach :md5sum-) "c91fb3be70ce66d19130d40294cf4bd5")
(setf (get gazebo_ros_link_attacher::Attach :datatype-) "gazebo_ros_link_attacher/Attach")
(setf (get gazebo_ros_link_attacher::Attach :request) gazebo_ros_link_attacher::AttachRequest)
(setf (get gazebo_ros_link_attacher::Attach :response) gazebo_ros_link_attacher::AttachResponse)

(defmethod gazebo_ros_link_attacher::AttachRequest
  (:response () (instance gazebo_ros_link_attacher::AttachResponse :init)))

(setf (get gazebo_ros_link_attacher::AttachRequest :md5sum-) "c91fb3be70ce66d19130d40294cf4bd5")
(setf (get gazebo_ros_link_attacher::AttachRequest :datatype-) "gazebo_ros_link_attacher/AttachRequest")
(setf (get gazebo_ros_link_attacher::AttachRequest :definition-)
      "string model_name_1
string link_name_1
string model_name_2
string link_name_2
---
bool ok

")

(setf (get gazebo_ros_link_attacher::AttachResponse :md5sum-) "c91fb3be70ce66d19130d40294cf4bd5")
(setf (get gazebo_ros_link_attacher::AttachResponse :datatype-) "gazebo_ros_link_attacher/AttachResponse")
(setf (get gazebo_ros_link_attacher::AttachResponse :definition-)
      "string model_name_1
string link_name_1
string model_name_2
string link_name_2
---
bool ok

")



(provide :gazebo_ros_link_attacher/Attach "c91fb3be70ce66d19130d40294cf4bd5")


