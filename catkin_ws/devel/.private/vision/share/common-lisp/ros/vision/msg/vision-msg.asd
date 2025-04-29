
(cl:in-package :asdf)

(defsystem "vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Lego_state" :depends-on ("_package_Lego_state"))
    (:file "_package_Lego_state" :depends-on ("_package"))
  ))