;; Auto-generated. Do not edit!


(when (boundp 'vision::SetTrackingColours)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'SetTrackingColours (find-package "VISION")))
(unless (find-package "VISION::SETTRACKINGCOLOURS")
  (make-package "VISION::SETTRACKINGCOLOURS"))

(in-package "ROS")
;;//! \htmlinclude SetTrackingColours.msg.html


(defclass vision::SetTrackingColours
  :super ros::object
  :slots (_hue_low _hue_high _sat_low _sat_high _val_low _val_high ))

(defmethod vision::SetTrackingColours
  (:init
   (&key
    ((:hue_low __hue_low) 0)
    ((:hue_high __hue_high) 0)
    ((:sat_low __sat_low) 0)
    ((:sat_high __sat_high) 0)
    ((:val_low __val_low) 0)
    ((:val_high __val_high) 0)
    )
   (send-super :init)
   (setq _hue_low (round __hue_low))
   (setq _hue_high (round __hue_high))
   (setq _sat_low (round __sat_low))
   (setq _sat_high (round __sat_high))
   (setq _val_low (round __val_low))
   (setq _val_high (round __val_high))
   self)
  (:hue_low
   (&optional __hue_low)
   (if __hue_low (setq _hue_low __hue_low)) _hue_low)
  (:hue_high
   (&optional __hue_high)
   (if __hue_high (setq _hue_high __hue_high)) _hue_high)
  (:sat_low
   (&optional __sat_low)
   (if __sat_low (setq _sat_low __sat_low)) _sat_low)
  (:sat_high
   (&optional __sat_high)
   (if __sat_high (setq _sat_high __sat_high)) _sat_high)
  (:val_low
   (&optional __val_low)
   (if __val_low (setq _val_low __val_low)) _val_low)
  (:val_high
   (&optional __val_high)
   (if __val_high (setq _val_high __val_high)) _val_high)
  (:serialization-length
   ()
   (+
    ;; uint8 _hue_low
    1
    ;; uint8 _hue_high
    1
    ;; uint8 _sat_low
    1
    ;; uint8 _sat_high
    1
    ;; uint8 _val_low
    1
    ;; uint8 _val_high
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _hue_low
       (write-byte _hue_low s)
     ;; uint8 _hue_high
       (write-byte _hue_high s)
     ;; uint8 _sat_low
       (write-byte _sat_low s)
     ;; uint8 _sat_high
       (write-byte _sat_high s)
     ;; uint8 _val_low
       (write-byte _val_low s)
     ;; uint8 _val_high
       (write-byte _val_high s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _hue_low
     (setq _hue_low (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hue_high
     (setq _hue_high (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_low
     (setq _sat_low (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_high
     (setq _sat_high (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _val_low
     (setq _val_low (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _val_high
     (setq _val_high (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get vision::SetTrackingColours :md5sum-) "2c86efe874e740a21108c4cdc260ae8a")
(setf (get vision::SetTrackingColours :datatype-) "vision/SetTrackingColours")
(setf (get vision::SetTrackingColours :definition-)
      "uint8 hue_low
uint8 hue_high
uint8 sat_low
uint8 sat_high
uint8 val_low
uint8 val_high


")



(provide :vision/SetTrackingColours "2c86efe874e740a21108c4cdc260ae8a")


