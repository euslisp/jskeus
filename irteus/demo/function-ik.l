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

(load "sample-robot-model.l")

(defun shake-cocktail ()
  (send *irtviewer* :title "shake-cocktail")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))

  (setq *obj* (make-cylinder 20 100))
  (send *obj* :set-color #f(1 1 0))
  (send *robot* :reset-pose)
  (objects (list *robot* *obj*))

  (send *robot* :inverse-kinematics
        (list (make-coords :pos #f(400 0 0)))
        :move-target
        (list (send *robot* :larm :end-coords))
        :link-list
        (list (send *robot* :link-list
                    (send (send *robot* :larm :end-coords) :parent)
                    (car (send *robot* :larm :links))))
        :translation-axis (list t)
        :rotation-axis (list nil))

  (let* ((cnt 0.0))
    (do-until-key
     (incf cnt 0.1)
     (send *robot* :inverse-kinematics
           (list (make-coords :pos (float-vector (+ 400 (* 100 (sin cnt))) (* 50 (cos cnt)) 0))
                 #'(lambda ()
                     (send (send (send *robot* :larm :end-coords) :copy-worldcoords)
                           :translate #f(0 0 100) :local)))
           :move-target
           (list (send *robot* :larm :end-coords)
                 (send *robot* :rarm :end-coords))
           :link-list
           (list (send *robot* :link-list
                       (send (send *robot* :larm :end-coords) :parent)
                       (car (send *robot* :larm :links)))
                 (send *robot* :link-list
                       (send (send *robot* :rarm :end-coords) :parent)
                       (car (send *robot* :rarm :links))))
           :translation-axis (list :z t)
           :rotation-axis (list nil :z))
     (send *obj* :newcoords (send (send *robot* :larm :end-coords) :copy-worldcoords))
     (send *irtviewer* :draw-objects))))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(shake-cocktail) ;; generate shaking cocktail motion using functional target-coords.~%")
