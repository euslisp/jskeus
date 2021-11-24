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

(require :irtmath)
(require :irtgeo)

(in-package "GEOMETRY")

(export '(bt-collision-distance bt-collision-check))

(defun bt-make-model-from-body
    (b &key (csg (send b :csg)) (margin 0))
  "Make bullet model from body."
  (let (m)
    (cond ((assoc :sphere csg)
           (setq m
                 (btmakespheremodel
                  (* 1e-3 (+ (user::radius-of-sphere b) margin)))
                 ))
          ((assoc :cube csg)
           (setq m
                 (btmakeboxmodel
                  (* 1e-3 (+ (user::x-of-cube b) (* margin 2)))
                  (* 1e-3 (+ (user::y-of-cube b) (* margin 2)))
                  (* 1e-3 (+ (user::z-of-cube b) (* margin 2)))
                  )))
          ((assoc :cylinder csg)
           (setq m
                 (btmakecylindermodel
                  (* 1e-3 (+ (user::radius-of-cylinder b) margin))
                  (* 1e-3 (+ (user::height-of-cylinder b) margin))
                  )))
          (t
           (setq m
                 (btmakemeshmodel
                  (scale 1e-3 ;; [m]
                         (apply #'float-vector (apply #'concatenate cons
                                (mapcar #'(lambda (v) (send b :inverse-transform-vector v)) (send b :vertices)))))
                  (length (send b :vertices))
                  ))
           (if (> margin 0) (btsetmargin m (* 1e-3 margin)))
           ))
    m)
  )

(defmethod cascaded-coords
  (:make-btmodel
   (&key (fat 0) ((:faces fs)))
    "Make bullet model and save pointer of the bullet model."
   (let (vs m v v1 v2 v3)
     (cond (fs
            (setq vs (flatten (send-all fs :vertices)))
            (setq m
                  (btmakemeshmodel
                   (scale 1e-3 ;; [m]
                          (apply #'float-vector (apply #'concatenate cons
                                 (mapcar #'(lambda (v) (send self :inverse-transform-vector v)) vs))))
                   (length vs)
                   ))
            (btsetmargin m fat)
            )
           ((derivedp self body)
            (setq m
                  (bt-make-model-from-body self :margin fat))
            )
           (t
            (setq vs (flatten (send-all (send self :bodies) :vertices)))
            (setq m
                  (btmakemeshmodel
                   (scale 1e-3 ;; [m]
                          (apply #'float-vector (apply #'concatenate cons
                                 (mapcar #'(lambda (v) (send self :inverse-transform-vector v)) vs))))
                   (length vs)
                   ))
            (btsetmargin m fat)
            ))
     (setf (get self :btmodel) m)
     m)
    )
  )

(defun bt-collision-distance
    (model1 model2 &key (fat 0) (fat2 nil) (qsize))
  "Calculate collision distance between model1 and model2 using Bullet.
   Return value is (list [distance] [nearest point on model1] [nearest point on model2]).
   qsize argument is not used, just for compatibility with pqp-collision-distance."
  (let ((m1 (get model1 :btmodel))
        (m2 (get model2 :btmodel))
        (r1 (user::matrix2quaternion (send model1 :worldrot)))
        (t1 (scale 1e-3 (send model1 :worldpos))) ;; [m]
        (r2 (user::matrix2quaternion (send model2 :worldrot)))
        (t2 (scale 1e-3 (send model2 :worldpos))) ;; [m]
        (dist (float-vector 0))
        (dir (float-vector 0 0 0))
        (p1 (float-vector 0 0 0))
        (p2 (float-vector 0 0 0))
        r)
    (if (null fat2) (setq fat2 fat))
    (if (null m1) (setq m1 (send model1 :make-btmodel :fat fat)))
    (if (null m2) (setq m2 (send model2 :make-btmodel :fat fat2)))
    (btcalccollisiondistance
     m1 m2 t1 r1 t2 r2
     dist dir p1 p2)
    (list (* 1e3 (elt dist 0)) (scale 1e3 p1) (scale 1e3 p2))
    ))

(defun bt-collision-check
    (model1 model2 &key (fat 0) (fat2 nil))
  "Check collision between model1 and model2 using Bullet.
   If return value is 0, no collision.
   Otherwise (return value is 1), collision."
  (if (> (elt (bt-collision-distance model1 model2 :fat fat :fat2 fat2) 0) 0) 0 1)
  )

(provide :bullet "$Id$")
