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

(defclass sample-broom
  :super cascaded-link
  :slots (handle)
  )

(defmethod sample-broom
  (:init
    (&rest args
     &key (sweep-height 250) (sweep-width 125) (sweep-thickness 20)
          (bar-length 700) (bar-width 20)
     &allow-other-keys)
    (send-super* :init args)
    (setq handle nil)
    ;; 1. make links links and assoc all links
    (let ((rl (send self :make-broom-link
                    sweep-height sweep-width sweep-thickness
                    bar-length bar-width)))
      ;; 2. assoc links
      ;;    Root link should be associated with "self".
      (send self :assoc rl)
      ;; 3. make all joints
      ;;    Before making joints, you should :assoc all links.

      ;; 4. define slots for robot class
      ;;    links and joint-list for cascaded-link.
      (setq links (list rl))
      (setq joint-list nil)
      ;; 5. call :init-ending after defining links and joint-list and return "self"
      (send self :init-ending)
      self))
  ;; Methods to define robot links
  (:make-broom-link
    (sh ;; sw = Sweep Height
     sw ;; sw = Sweep Width
     st ;; st = Sweep Thickness
     bl ;; bl = Bar Length
     bw) ;; bw = Bar Width
   (let* ((bar (make-cylinder (/ bw 2.0) bl))
	  (sweep (make-prism (list (float-vector sw (- sh) 0)
				   (float-vector (- sw) (- sh) 0)
				   (float-vector (* -0.5 bw) 0 0)
				   (float-vector (* 0.5 bw) 0 0))
                             st)))
     (send bar :locate (float-vector 0 0 sh) :world)
     (send sweep :rotate pi/2 :x)
     (send sweep :locate (float-vector 0 10 sh) :world)
     (send bar :set-color :brown)
     (send sweep :set-color :red)
     (send bar :assoc sweep)
     (let ((br (instance bodyset-link :init (make-cascoords)
			 :bodies (list bar sweep))))
       (dolist (rate (list 0.8 0.6))
	 (push (make-cascoords :pos (float-vector 0 0 (+ (* rate bl) sh))) handle))
       (dolist (hc handle) (send br :assoc hc))
       br)))
  (:handle () handle)
  )

(defun dual-arm-ik nil
  "dual arm ik"
  (send *irtviewer* :title "dual-arm-ik")
  (let ((i 0) link-list move-target target-coords b0)
    ;;
    (unless (boundp '*robot*)
      (setq *robot* (instance sample-robot :init)))
    (send *robot* :reset-pose)
    (if (some #'null (send *robot* :legs))
	(send *robot* :newcoords (make-coords))
      (send *robot* :fix-leg-to-coords (make-coords)))
    (send *robot* :update-descendants)
    ;;
    ;; make broom model
    (setq b0 (instance sample-broom :init))
    (send b0 :locate #f(250 0 0))
    ;;
    ;; setup move-target and link-list
    (setq move-target (send *robot* :arms :end-coords)
	  link-list (mapcar #'(lambda (mt) (send *robot* :link-list mt)) (send-all move-target :parent)))
    ;;
    ;; look-at
    (send *robot* :head :look-at (apply #'midpoint 0.5 (send-all (send b0 :handle) :worldpos)))
    (objects (list *robot* b0))
    ;;
    ;; do sweep
    (do-until-key
     (send *robot* :inverse-kinematics (send b0 :handle)
	   :link-list link-list :move-target move-target
	   :stop 500 :thre '(10 10)
	   :rotation-axis '(nil nil) :debug-view nil :dump-command nil)
     (send *robot* :head :look-at
	   (apply #'midpoint 0.5 (send-all (send b0 :handle) :worldpos)))
     (send b0 :orient (* 0.2 (sin (/ i 10.0))) :x :world)
     (send b0 :locate (float-vector 250 (* 250 (sin (/ (incf i) 10.0))) 0) :world)
     (send *irtviewer* :draw-objects)
     (x::window-main-one)
     (incf i)
     )
    ))
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(dual-arm-ik) ;; for tool usage~%")
