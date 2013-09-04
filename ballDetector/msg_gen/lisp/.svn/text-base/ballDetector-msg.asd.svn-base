
(cl:in-package :asdf)

(defsystem "ballDetector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ballLocation" :depends-on ("_package_ballLocation"))
    (:file "_package_ballLocation" :depends-on ("_package"))
  ))