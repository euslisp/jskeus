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

(defun full-body-ik
  (&key (target (make-coords :pos (float-vector 0 0 1500)))
        (use-torso t) (use-leg) (debug-view :no-flush))
  "full body ik"
    (send *irtviewer* :title "full-body-ik")
    (unless (boundp '*robot*)
      (setq *robot* (instance sample-robot :init)))
    (send *robot* :reset-pose)
    (if (= (length (car (send *robot* :legs))) 6)
	(send *robot* :legs :angle-vector #f(0 0 -10 20 0 -10)))
    (if (some #'null (send *robot* :legs))
	(send *robot* :newcoords (make-coords))
      (send *robot* :fix-leg-to-coords (make-coords) :lleg))
    (send *robot* :update-descendants)
    (let* ((move-target (send *robot* :larm :end-coords))
           (link-list (send *robot* :link-list
                            (send move-target :parent)
                            (cond
                             (use-leg (send *robot* :lleg :end-coords :parent))
                             (use-torso (send *robot* :torso :root-link))
                             (t (send *robot* :larm :root-link))))))
      (objects (list *robot*))
      ;;
      (do-until-key
       (let ((dif-pos (send move-target :difference-position target
                            :translation-axis t))
             (dif-rot (send move-target :difference-rotation target
                            :rotation-axis nil)))
         (send* *robot* :inverse-kinematics-loop dif-pos dif-rot
               :link-list link-list
               :target-coords target
               :move-target (send *robot* :larm :end-coords)
               :rotation-axis nil :translation-axis t
               :manipulability-gain 0.05
               :debug-view debug-view
               (if use-leg '(:manipulability-limit 0.5)))
       (send *robot* :head :look-at
             (send *robot* :larm :end-coords :worldpos))
       (if (some #'null (send *robot* :legs))
           (send *robot* :newcoords (make-coords))
         (send *robot* :fix-leg-to-coords (make-coords) :lleg))
       (if use-leg (send *irtviewer* :draw-objects :flush nil))
       (send *irtviewer* :viewer :viewsurface :color #f(1 1 1))
       (send *irtviewer* :viewer :viewsurface :line-width 2)
       (send *irtviewer* :viewer :viewsurface :3d-line
             (send *robot* :larm :end-coords :worldpos)
             (send *robot* :rarm :end-coords :worldpos))
       (send *irtviewer* :flush)
       )
      ))
    )

;;
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(full-body-ik) ;; for humanoid~%")

