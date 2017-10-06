
(cl:in-package :asdf)

(defsystem "vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetTrackingColours" :depends-on ("_package_SetTrackingColours"))
    (:file "_package_SetTrackingColours" :depends-on ("_package"))
    (:file "TrackedPosition" :depends-on ("_package_TrackedPosition"))
    (:file "_package_TrackedPosition" :depends-on ("_package"))
  ))