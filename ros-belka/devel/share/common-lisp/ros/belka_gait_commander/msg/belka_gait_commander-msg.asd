
(cl:in-package :asdf)

(defsystem "belka_gait_commander-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HipsGaitCommand" :depends-on ("_package_HipsGaitCommand"))
    (:file "_package_HipsGaitCommand" :depends-on ("_package"))
    (:file "ShouldersGaitCommand" :depends-on ("_package_ShouldersGaitCommand"))
    (:file "_package_ShouldersGaitCommand" :depends-on ("_package"))
  ))