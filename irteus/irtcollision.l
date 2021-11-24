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
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright notice,
;;;   this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright notice,
;;;   this list of conditions and the following disclaimer in the documentation
;;;   and/or other materials provided with the distribution.
;;; * Neither the name of JSK Robotics Laboratory, The University of Tokyo
;;;   (JSK) nor the names of its contributors may be used to endorse or promote
;;;   products derived from this software without specific prior written
;;;   permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
;;; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
;;; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
;;; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
;;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
;;; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
;;; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;;; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
;;; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package "USER")

(require :pqp)
(require :bullet)

;; set *collision-algorithm* from available libraries.
;; need to cehck if *collision-algorithm-pqp*/*collision-algorithm-bullet* exists, becuase it does not defined during compile time
(if (and (boundp '*collision-algorithm-pqp*) (boundp '*collision-algorithm-bullet*))
    (defvar *collision-algorithm* (or *collision-algorithm-pqp* *collision-algorithm-bullet*)))

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
    (model1 model2 &rest args)
  "Check collision between model1 and model2.
   If return value is 0, no collision.
   Otherwise (return value is 1), collision."
  (cond ((eq *collision-algorithm* *collision-algorithm-pqp*)
         (if (symbolp (car args)) (setq args (append (list geo::PQP_FIRST_CONTACT) args)))
         (apply #'pqp-collision-check model1 model2 args))
        ((eq *collision-algorithm* *collision-algorithm-bullet*)
         (if (numberp (car args)) (setq args (cdr args)))
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
