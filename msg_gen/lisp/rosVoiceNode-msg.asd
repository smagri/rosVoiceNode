
(cl:in-package :asdf)

(defsystem "rosVoiceNode-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "voiceNode" :depends-on ("_package_voiceNode"))
    (:file "_package_voiceNode" :depends-on ("_package"))
  ))