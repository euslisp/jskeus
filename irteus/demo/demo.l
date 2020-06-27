#!/usr/bin/env irteusgl
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

;;(load "irteus/irtmodel.l")
;;(load "irteus/irtdyna.l")

(load "full-body-ik.l")
(load "function-ik.l")
(load "null-space-ik.l")
(load "dual-arm-ik.l")
(load "hand-grasp-ik.l")
(load "dual-manip-ik.l")
(load "crank-motion.l")
(load "walk-motion.l")
(load "hanoi-arm.l")
(load "particle.l")
(load "closed-loop.l")
(load "special-joints.l")
(load "virtual-joints.l")
(load "look-at-ik.l")

(unless (boundp '*irtviewer*) (make-irtviewer))
#+:jsk
(defun objects (&rest args) (apply #'irt-objects args))

#+:jsk
(defun test-j1 nil
  (let (r0 b0)
    ;; demo program for using jsk/hand model in irteus
    ;;
    ;; import jsk/rbrain library
    (provide :rbrain-basic nil)
    (defclass robot-link :super cascaded-link)
    (defclass articulated-link :super cascaded-link
      :slots (analysis-level))
    (defmethod articulated-link
      (:vertices () (list (send self :worldpos))))
    (defclass robot-frame :super cascaded-link)
    (defclass robot-limb :super cascaded-link)
    (unless (find-package "WIN32") (make-package "WIN32"))
    (load "~/prog/euslib/rbrain/jskmapclass.l")
    (defmethod bodyset-with-knowledge
      (:vertices () (list (send self :worldpos))))
    (setq r0 (instance sample-robot :init))
    (send r0 :transform (send (apply #'midcoords 0.5 (send r0 :legs :end-coords)) :transformation (send r0 :worldcoords)))
    (send r0 :update-descendants)
    (setq b0 (instance broom :init :length 700 :bar-color :brown :sweep-color :red))
    (send b0 :locate #f(200 0 0))

    (send r0 :lleg :move-end-pos #f(0 0 50) :local :rotation-axis t)
    (send r0 :rleg :move-end-pos #f(0 0 50) :local :rotation-axis t)
    (objects (list  r0 b0))

    (send r0 :larm :inverse-kinematics (car (send b0 :handle))
          :stop 1000 :thre 10
          :rotation-axis nil :debug-view t)
    (send r0 :rarm :inverse-kinematics (cadr (send b0 :handle))
          :stop 1000 :thre 10
          :rotation-axis nil :debug-view t)
    ))

