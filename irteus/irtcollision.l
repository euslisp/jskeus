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

(require :pqp)
(require :bullet)

(defconstant *collision-algorithm-pqp* :pqp)
(defconstant *collision-algorithm-bullet* :bullet)
(defvar *collision-algorithm* *collision-algorithm-pqp*)

(defmethod cascaded-coords
  (:make-collisionmodel
   (&rest args &key &allow-other-keys)
   "Make collision model and save pointer."
   (cond ((eq *collision-algorithm* *collision-algorithm-pqp*)
          (send* self :make-pqpmodel args))
         ((eq *collision-algorithm* *collision-algorithm-bullet*)
          (send* self :make-btmodel args))
         (t
          (error "invalid collision algorithm: ~a~%" *collision-algorithm*)))
   )
  )

(defun collision-distance
    (model1 model2 &rest args &key &allow-other-keys)
  "Calculate collision distance between model1 and model2.
   Return value is (list [distance] [nearest point on model1] [nearest point on model2])."
  (cond ((eq *collision-algorithm* *collision-algorithm-pqp*)
         (apply #'pqp-collision-distance model1 model2 args))
        ((eq *collision-algorithm* *collision-algorithm-bullet*)
         (apply #'bt-collision-distance model1 model2 args))
        (t
         (error "invalid collision algorithm: ~a~%" *collision-algorithm*)))
  )

(defun collision-check
    (model1 model2 &rest args &key &allow-other-keys)
  "Check collision between model1 and model2.
   If return value is 0, no collision.
   Otherwise (return value is 1), collision."
  (cond ((eq *collision-algorithm* *collision-algorithm-pqp*)
         (apply #'pqp-collision-check model1 model2 args))
        ((eq *collision-algorithm* *collision-algorithm-bullet*)
         (apply #'bt-collision-check model1 model2 args))
        (t
         (error "invalid collision algorithm: ~a~%" *collision-algorithm*)))
  )

(defun collision-check-objects
    (obj1 obj2 &rest args &key &allow-other-keys)
  "Check collision between obj1 and obj2.
   obj1 and obj2 should be list of models.
   If return value is nil, no collision.
   Otherwise (return value is t), collision."
  (dolist (o1 obj1)
    (dolist (o2 obj2)
      (if (> (collision-check o1 o2) 0)
          (return-from collision-check-objects t))))
  nil)

(defun select-collision-algorithm
    (alg)
  "Select collision algorithm.
   :pqp and :bullet are supported."
  (setq *collision-algorithm* alg)
  )

(in-package "GEOMETRY")

(provide :irtcollision "$Id$")
