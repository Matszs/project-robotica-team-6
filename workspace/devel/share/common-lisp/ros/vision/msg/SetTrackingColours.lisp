; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude SetTrackingColours.msg.html

(cl:defclass <SetTrackingColours> (roslisp-msg-protocol:ros-message)
  ((hue_low
    :reader hue_low
    :initarg :hue_low
    :type cl:fixnum
    :initform 0)
   (hue_high
    :reader hue_high
    :initarg :hue_high
    :type cl:fixnum
    :initform 0)
   (sat_low
    :reader sat_low
    :initarg :sat_low
    :type cl:fixnum
    :initform 0)
   (sat_high
    :reader sat_high
    :initarg :sat_high
    :type cl:fixnum
    :initform 0)
   (val_low
    :reader val_low
    :initarg :val_low
    :type cl:fixnum
    :initform 0)
   (val_high
    :reader val_high
    :initarg :val_high
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetTrackingColours (<SetTrackingColours>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTrackingColours>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTrackingColours)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<SetTrackingColours> is deprecated: use vision-msg:SetTrackingColours instead.")))

(cl:ensure-generic-function 'hue_low-val :lambda-list '(m))
(cl:defmethod hue_low-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:hue_low-val is deprecated.  Use vision-msg:hue_low instead.")
  (hue_low m))

(cl:ensure-generic-function 'hue_high-val :lambda-list '(m))
(cl:defmethod hue_high-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:hue_high-val is deprecated.  Use vision-msg:hue_high instead.")
  (hue_high m))

(cl:ensure-generic-function 'sat_low-val :lambda-list '(m))
(cl:defmethod sat_low-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:sat_low-val is deprecated.  Use vision-msg:sat_low instead.")
  (sat_low m))

(cl:ensure-generic-function 'sat_high-val :lambda-list '(m))
(cl:defmethod sat_high-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:sat_high-val is deprecated.  Use vision-msg:sat_high instead.")
  (sat_high m))

(cl:ensure-generic-function 'val_low-val :lambda-list '(m))
(cl:defmethod val_low-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:val_low-val is deprecated.  Use vision-msg:val_low instead.")
  (val_low m))

(cl:ensure-generic-function 'val_high-val :lambda-list '(m))
(cl:defmethod val_high-val ((m <SetTrackingColours>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:val_high-val is deprecated.  Use vision-msg:val_high instead.")
  (val_high m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTrackingColours>) ostream)
  "Serializes a message object of type '<SetTrackingColours>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue_low)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue_high)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_low)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_high)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_low)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_high)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTrackingColours>) istream)
  "Deserializes a message object of type '<SetTrackingColours>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue_low)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hue_high)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_low)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_high)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_low)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_high)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTrackingColours>)))
  "Returns string type for a message object of type '<SetTrackingColours>"
  "vision/SetTrackingColours")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTrackingColours)))
  "Returns string type for a message object of type 'SetTrackingColours"
  "vision/SetTrackingColours")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTrackingColours>)))
  "Returns md5sum for a message object of type '<SetTrackingColours>"
  "2c86efe874e740a21108c4cdc260ae8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTrackingColours)))
  "Returns md5sum for a message object of type 'SetTrackingColours"
  "2c86efe874e740a21108c4cdc260ae8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTrackingColours>)))
  "Returns full string definition for message of type '<SetTrackingColours>"
  (cl:format cl:nil "uint8 hue_low~%uint8 hue_high~%uint8 sat_low~%uint8 sat_high~%uint8 val_low~%uint8 val_high~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTrackingColours)))
  "Returns full string definition for message of type 'SetTrackingColours"
  (cl:format cl:nil "uint8 hue_low~%uint8 hue_high~%uint8 sat_low~%uint8 sat_high~%uint8 val_low~%uint8 val_high~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTrackingColours>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTrackingColours>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTrackingColours
    (cl:cons ':hue_low (hue_low msg))
    (cl:cons ':hue_high (hue_high msg))
    (cl:cons ':sat_low (sat_low msg))
    (cl:cons ':sat_high (sat_high msg))
    (cl:cons ':val_low (val_low msg))
    (cl:cons ':val_high (val_high msg))
))
