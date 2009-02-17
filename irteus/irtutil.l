;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
;;;
;;; This software is a collection of EusLisp code for robot applications,
;;; which has been developed by the JSK Laboratory for the IRT project.
;;; For more information on EusLisp and its application to the robotics,
;;; please refer to the following papers.
;;;
;;; Toshihiro Matsui
;;; Multithread object-oriented language euslisp for parallel and
;;;  asynchronous programming in robotics
;;; Workshop on Concurrent Object-based Systems,
;;;  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
;;;
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;

(in-package "USER")

(export '(forward-message-to forward-message-to-all))

(defun forward-message-to (to args)
  (if args (send* to args) to))

(defun forward-message-to-all (to args)
  (if args (apply #'send-all to args) to))

(defmacro send-message* (&rest msgs)
  `(apply #'send-message . ,msgs)
  )

(defmacro do-until-key (&rest forms)
  `(while (null (select-stream (list *standard-input*) 0.0000001))
     ,@forms
     )
  )

(defclass mtimer
  :super object
  :slots (buf))
(defmethod mtimer
  (:init () (send self :start) self)
  (:start () (setq buf (unix:gettimeofday)))
  (:stop ()
	 (let ((tmp nil))
	    (setq tmp (unix:gettimeofday))
	    (setq tmp (map cons #'- tmp buf))
	    (+ (car tmp) (/ (cadr tmp) 1000000f))))
  )

(provide :irtutil "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.3  2009-02-17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.2  2008/10/09 15:09:46  k-okada
;;; add send-message
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
