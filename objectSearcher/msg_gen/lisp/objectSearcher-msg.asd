
(cl:in-package :asdf)

(defsystem "objectSearcher-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hoverStatus" :depends-on ("_package_hoverStatus"))
    (:file "_package_hoverStatus" :depends-on ("_package"))
  ))