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


(unless (boundp '*objdir*)
  (setq *objdir*
        (format nil "~A/~A/obj/" *eusdir* (unix:getenv "ARCHDIR"))
  ))

(in-package "IMAGE")
(if (eq (unix:access (format nil "~A/lisp/image/jpeg/" *eusdir*) unix::O_RDWR) t)
    (comp:compile-file-if-src-newer (format nil "~A/lisp/image/jpeg/eusjpeg.l" *eusdir*) user::*objdir*)
  (progn
    (unix::mkdir (format nil "/tmp/~A" (unix::getpid)))
    (unix::system (format nil "cp ~A/lisp/image/jpeg/eusjpeg.l /tmp/~A/eusjpeg.l" user::*eusdir* (unix::getpid)))
    (unix::link (format nil "~A/lisp/image/jpeg/eusjpeg.l" *eusdir*) (format nil "/tmp/~A/eusjpeg.l" (unix::getpid)))
    (comp:compile-file-if-src-newer (format nil "/tmp/~A/eusjpeg.l" (unix::getpid)) user::*objdir*)
    (unix::unlink (format nil "/tmp/~A" (unix::getpid)))
    ))
(comp:compile-file-if-src-newer (format nil "png.l" *eusdir*) user::*objdir*)
(comp:compile-file-if-src-newer (format nil "irtimage.l" *eusdir*) user::*objdir*)

(exit 0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;;
