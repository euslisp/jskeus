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

(defun dual-manip-ik
  (&rest args)
  "dual-armed object manipulation"
  ;; initialize robot
  (send *irtviewer* :title "dual-manip-ik")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  ;; fix leg
  (if (some #'null (send *robot* :legs))
      (send *robot* :newcoords (make-coords))
    (send *robot* :fix-leg-to-coords (make-coords)))
  (send *robot* :update-descendants)
  ;; generate object model
  (setq *obj* (make-cube 30 160 100))
  (dolist (l '(:rarm :larm))
    (send *obj* :put l ;; generate target coords
          (make-cascoords :coords
                          (send (send (send *obj* :copy-worldcoords) :rotate (case l (:rarm pi/2) (:larm -pi/2)) :z)
                                :translate (float-vector -75 0 0))))
    (send *obj* :assoc (send *obj* :get l)))
  (send *obj* :newcoords (make-coords :pos #f(300 0 700)))
  (send *obj* :set-color :blue)
  (send *robot* :head :look-at (send *obj* :worldpos))
  (objects (list *obj* *robot*))
  ;; ik codes
  (let ((i 0))
    (do-until-key
     ;; rotate target coords
     (when (/= i 0)
       (let ((l (cond
                 ((or (= (mod i 4) 1) (= (mod i 4) 2)) :rarm)
                 ((or (= (mod i 4) 3) (= (mod i 4) 0)) :larm)))
             (rot (if (oddp (mod i 4)) (* (case (mod i 4) (1 1) (3 -1)) (deg2rad 90))))
             (trs (float-vector (* (expt -1 (mod i 2)) 50) 0 0)))
         (send *obj* :dissoc (send *obj* :get l))
         (send *obj* :assoc (send *obj* :get l))
         (if (oddp (mod i 4))
             (send (send *obj* :get l) :rotate rot :x))
         (send (send *obj* :get l) :translate trs)))
     ;; solve inverse kinematics
     (with-move-target-link-list
      (mt robot-ll *robot* '(:rarm :larm))
      (let* ((ret
              (append-obj-virtual-joint
               robot-ll (mapcar #'(lambda (l) (send *obj* :get l)) '(:rarm :larm))
               :joint-class 6dof-joint))
             (ll (car ret)) (ot (cadr ret))
             (rotation-axis (cond
                             ((= (mod i 4) 1) '(:x t))
                             ((= (mod i 4) 3) '(t :x))
                             (t '(t t))))
             (weight (fill (instantiate float-vector (send *robot* :calc-target-joint-dimension ll)) 1.0)))
        (dotimes (i 3)
          (setf (elt weight (+ (- (send *robot* :calc-target-joint-dimension ll) 6) i)) 0.001))
        (send (send (car ot) :parent) :assoc *obj*)
        (send* *robot* :inverse-kinematics ot
               :link-list ll :move-target mt
               :rotation-axis rotation-axis
               :weight weight
               :union-link-list #'(lambda (tmp-ll)
                                    (apply #'append
                                           (mapcar
                                            #'(lambda (f)
                                                (send *robot* :calc-union-link-list
                                                      (mapcar #'(lambda (l)
                                                                  (funcall f #'(lambda (x) (member x (send *robot* :links))) l))
                                                              tmp-ll)))
                                            (list #'remove-if-not #'remove-if))))
               :jacobi #'(lambda (tmp-ll tmp-mt tmp-ta tmp-ra)
                           (calc-jacobian-from-link-list-including-robot-and-obj-virtual-joint
                            tmp-ll tmp-mt ot *robot* :rotation-axis tmp-ra))
               ;;:debug-view t
               :debug-view :no-message
	       :dump-command nil
               (append (if (= i 0) '(:stop 100)) args))
        (send (send (car ot) :parent) :dissoc *obj*)
        ))
     (incf i)
     (send *robot* :head :look-at (send *obj* :worldpos))
     )
    ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(dual-manip-ik) ;; for dual-armed object manipulation~%")


