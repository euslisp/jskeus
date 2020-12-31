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

(defun null-space-ik
  ()
  "null space ik"
  (let ((left-target-coords (make-coords :pos #f(300 200 800)))
	(right-target-coords (make-coords :pos #f(400 -200 800)))
	(sign 1)
	(vel 0.05)
	(tm 0))
    (send *irtviewer* :title "null-space-ik")
    (unless (boundp '*robot*)
      (setq *robot* (instance sample-robot :init)))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :update-descendants)
    (setq *box* (make-cube 100 100 100 :pos #f(200 -200 500)))
    (send *box* :set-color :red)
    (objects (list *robot* *box*))
    ;;
    (do-until-key
#|
     you can use :null-space, or :additional-nspace-list
       :weight or :null-space argument (recommended for developper)

         If users want to specify weight or null-space as float-vector, use :weight or :null-space.

       :additional-weight-list or :additional-nspace-list (recommended for users)

         If users want to specify the joint to set weight or nspace, use :additional-weight-list or :additional-nspace-list.
|#

     (send *robot* :larm :inverse-kinematics left-target-coords
	   ;; :null-space (float-vector 0 (* sign vel) 0 0 0 0 0) ;; joint angle speed, in null space
	   :additional-nspace-list
	   (list (list (send *robot* :larm :shoulder-r :child-link) (* sign vel)))
	   :rotation-axis :z;; t
	   ;;:debug-view t
	   :look-at-target t)
     (format t "shoulder-r ~A~%" (send *robot* :larm :shoulder-r :joint-angle))
     (cond ((> (elt (send *robot* :larm :angle-vector) 1) 50)
	    (setq sign -1))
	   ((< (elt (send *robot* :larm :angle-vector) 1) 10)
	    (setq sign  1)))

     (send *robot* :rarm :inverse-kinematics right-target-coords
	   :collision-avoidance-link-pair (list (list (send *robot* :rarm :elbow-p :child-link) *box*))
	   :avoid-collision-joint-gain 1.0 ;; 1.0
	   :avoid-collision-null-gain  200 ;; 1.0
	   :rotation-axis :z;; t
	   :debug-view :no-message
           ;; you can set optional ik convergence check by additional-check function
           :additional-check #'(lambda ()
                                 ;; calculate :inverse-kinematics-loop until collision distance becomes large enough. (> 80mm, in this case)
                                 (let ((dist (car (collision-distance (send *robot* :rarm :elbow-p :child-link) *box*))))
                                   (> dist 80)))
	   )
     (send *box* :translate (float-vector 0 0 (* 7 (sin tm))))
     (incf tm 0.1)
     
     (x::window-main-one) ;; to process x event
     )
    ))

;;
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(null-space-ik) ;; for humanoid~%")

