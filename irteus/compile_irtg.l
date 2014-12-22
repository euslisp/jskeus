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

(in-package "GEOMETRY")

(comp:compile-file-if-src-newer "irtgeo.l" user::*objdir*)
(comp:compile-file-if-src-newer "pqp.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtscene.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtmodel.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtsensor.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtdyna.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtrobot.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtbvh.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtcollada.l" user::*objdir*)
(comp:compile-file-if-src-newer "irtpointcloud.l" user::*objdir*)

(exit 0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.2  2009-02-17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:00  k-okada
;;; add irteus
;;;
;;;
