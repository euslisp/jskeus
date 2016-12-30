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

(unless (boundp '*objdir*)
  (setq *objdir*
        (format nil "~A/~A/obj/" *eusdir* (unix:getenv "ARCHDIR"))
  ))
(print *objdir*)
(comp:compile-file-if-src-newer "irtmath.l" *objdir*)
(comp:compile-file-if-src-newer "irtutil.l" *objdir*)
(comp:compile-file-if-src-newer "irtgraph.l" *objdir*)

(defun comp::compile-llib-file-if-src-newer (fname)
  (print (list 'compile fname))
  (if (eq (unix:access (format nil "~A/lib/llib/" *eusdir*) unix::O_RDWR) t)
      (comp:compile-file-if-src-newer (format nil "~A/lib/llib/~A" *eusdir* fname) *objdir*)
    (progn
      (unix::mkdir (format nil "/tmp/~A" (unix::getpid)))
      (unix::system (format nil "cp ~A/lib/llib/~A /tmp/~A/~A" *eusdir* fname (unix::getpid) fname))
      (unix::link (format nil "~A/lib/llib/~A" *eusdir* fname) (format nil "/tmp/~A/pgsql.l" (unix::getpid)))
      (comp:compile-file-if-src-newer (format nil "/tmp/~A/~A" (unix::getpid) fname) *objdir*)
      (unix::unlink (format nil "/tmp/~A" (unix::getpid)))
      ))
  )
(comp::compile-llib-file-if-src-newer "time.l")
(comp::compile-llib-file-if-src-newer "pgsql.l")

(exit 0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.3  2009-11-09 02:43:00  k-okada
;;; add pgsql
;;;
;;; Revision 1.2  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:00  k-okada
;;; add irteus
;;;
;;;
