;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
;;;
;;; This software is a collisoin of EusLisp code for robot applications,
;;; which has been developed by the JSK Laboratory for the IRT project.
;;; For more information on EusLisp and it's application to the robotics,
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


(setq *objdir*
  (format nil "~A/~A/obj/" *eusdir* (unix:getenv "ARCHDIR"))
  )

(in-package "IMAGE")
(comp:compile-file-if-src-newer (format nil "~A/lisp/image/jpeg/eusjpeg.l" *eusdir*) user::*objdir*)
(in-package "X")
(comp:compile-file-if-src-newer "irtx.l" user::*objdir*)

(exit 0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.1  2008-09-18 18:11:00  k-okada
;;; add irteus
;;;
;;;