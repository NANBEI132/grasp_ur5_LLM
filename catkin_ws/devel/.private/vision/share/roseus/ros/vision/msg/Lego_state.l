;; Auto-generated. Do not edit!


(when (boundp 'vision::Lego_state)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Lego_state (find-package "VISION")))
(unless (find-package "VISION::LEGO_STATE")
  (make-package "VISION::LEGO_STATE"))

(in-package "ROS")
;;//! \htmlinclude Lego_state.msg.html


(defclass vision::Lego_state
  :super ros::object
  :slots (_posx _posy _posz _rotx _roty _rotz _class_id ))

(defmethod vision::Lego_state
  (:init
   (&key
    ((:posx __posx) 0.0)
    ((:posy __posy) 0.0)
    ((:posz __posz) 0.0)
    ((:rotx __rotx) 0.0)
    ((:roty __roty) 0.0)
    ((:rotz __rotz) 0.0)
    ((:class_id __class_id) 0)
    )
   (send-super :init)
   (setq _posx (float __posx))
   (setq _posy (float __posy))
   (setq _posz (float __posz))
   (setq _rotx (float __rotx))
   (setq _roty (float __roty))
   (setq _rotz (float __rotz))
   (setq _class_id (round __class_id))
   self)
  (:posx
   (&optional __posx)
   (if __posx (setq _posx __posx)) _posx)
  (:posy
   (&optional __posy)
   (if __posy (setq _posy __posy)) _posy)
  (:posz
   (&optional __posz)
   (if __posz (setq _posz __posz)) _posz)
  (:rotx
   (&optional __rotx)
   (if __rotx (setq _rotx __rotx)) _rotx)
  (:roty
   (&optional __roty)
   (if __roty (setq _roty __roty)) _roty)
  (:rotz
   (&optional __rotz)
   (if __rotz (setq _rotz __rotz)) _rotz)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:serialization-length
   ()
   (+
    ;; float64 _posx
    8
    ;; float64 _posy
    8
    ;; float64 _posz
    8
    ;; float64 _rotx
    8
    ;; float64 _roty
    8
    ;; float64 _rotz
    8
    ;; int64 _class_id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _posx
       (sys::poke _posx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _posy
       (sys::poke _posy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _posz
       (sys::poke _posz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotx
       (sys::poke _rotx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roty
       (sys::poke _roty (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotz
       (sys::poke _rotz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _class_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _class_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _class_id) (= (length (_class_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_class_id . bv) 0) 0) s)
              (write-long (ash (elt (_class_id . bv) 1) -1) s))
             ((and (class _class_id) (= (length (_class_id . bv)) 1)) ;; big1
              (write-long (elt (_class_id . bv) 0) s)
              (write-long (if (>= _class_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _class_id s)(write-long (if (>= _class_id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _posx
     (setq _posx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _posy
     (setq _posy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _posz
     (setq _posz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotx
     (setq _rotx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roty
     (setq _roty (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotz
     (setq _rotz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _class_id
#+(or :alpha :irix6 :x86_64)
      (setf _class_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _class_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get vision::Lego_state :md5sum-) "b52ed836370681e6ac7dd0d073d7baf5")
(setf (get vision::Lego_state :datatype-) "vision/Lego_state")
(setf (get vision::Lego_state :definition-)
      "float64 posx
float64 posy
float64 posz
float64 rotx
float64 roty
float64 rotz
int64 class_id
")



(provide :vision/Lego_state "b52ed836370681e6ac7dd0d073d7baf5")


