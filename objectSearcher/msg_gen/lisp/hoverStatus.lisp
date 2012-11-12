; Auto-generated. Do not edit!


(cl:in-package objectSearcher-msg)


;//! \htmlinclude hoverStatus.msg.html

(cl:defclass <hoverStatus> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass hoverStatus (<hoverStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hoverStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hoverStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objectSearcher-msg:<hoverStatus> is deprecated: use objectSearcher-msg:hoverStatus instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <hoverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objectSearcher-msg:status-val is deprecated.  Use objectSearcher-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hoverStatus>) ostream)
  "Serializes a message object of type '<hoverStatus>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hoverStatus>) istream)
  "Deserializes a message object of type '<hoverStatus>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hoverStatus>)))
  "Returns string type for a message object of type '<hoverStatus>"
  "objectSearcher/hoverStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hoverStatus)))
  "Returns string type for a message object of type 'hoverStatus"
  "objectSearcher/hoverStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hoverStatus>)))
  "Returns md5sum for a message object of type '<hoverStatus>"
  "86791dcf1de997ec7de5a0de7e4dcfcc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hoverStatus)))
  "Returns md5sum for a message object of type 'hoverStatus"
  "86791dcf1de997ec7de5a0de7e4dcfcc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hoverStatus>)))
  "Returns full string definition for message of type '<hoverStatus>"
  (cl:format cl:nil "int32 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hoverStatus)))
  "Returns full string definition for message of type 'hoverStatus"
  (cl:format cl:nil "int32 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hoverStatus>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hoverStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'hoverStatus
    (cl:cons ':status (status msg))
))
