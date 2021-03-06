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

(defun look-at-ik-common
  (&key (camera-limb :head) (camera-axis :z))
  ;; initialize robot
  (send *irtviewer* :title "look-at-ik")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  ;; fix leg
  (if (some #'null (send *robot* :legs))
      (send *robot* :newcoords (make-coords))
    (send *robot* :fix-leg-to-coords (make-coords)))
  (send *robot* :update-descendants)
  ;; generate object model
  (setq *obj* (make-cube 100 100 100))
  (send *obj* :locate (float-vector 250 -300 700) :world)
  (send *obj* :set-color :blue)
  (objects (list *obj* *robot*))
  ;; ik codes
  (let* ((look-at-face (elt (send *obj* :faces) 2))
         (result
          (send *robot* :fullbody-inverse-kinematics
                (list #'(lambda ()
                          (orient-coords-to-axis
                           (make-coords :pos (send *obj* :worldpos)
                                        :rot (copy-object (send *robot* camera-limb :end-coords :worldrot)))
                           (v- (send look-at-face :normal))
                           camera-axis))
                      (send *robot* :rleg :end-coords :copy-worldcoords)
                      (send *robot* :lleg :end-coords :copy-worldcoords))
                :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) (list camera-limb :rleg :lleg))
                :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) (list camera-limb :rleg :lleg))
                :rotation-axis (list camera-axis t t)
                :translation-axis (list camera-axis t t)
                :debug-view :no-message
                :centroid-thre 20
                :stop 1000)))
    (send *irtviewer* :draw-objects)
    (send look-at-face :draw-on :flush t :color #f(1 0 0) :width 2)
    result))

(defun head-look-at-ik
  ()
  "look target by directing head camera"
  (look-at-ik-common :camera-limb :head :camera-axis :z)
  )

(defun hand-look-at-ik
  ()
  "look target by directing hand camera"
  (look-at-ik-common :camera-limb :larm :camera-axis :x)
  )

(defun look-at-hand-object-ik
  ()
  "look object in hand by head camera"
  ;; initialize robot
  (send *irtviewer* :title "look-at-ik")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  ;; fix leg
  (if (some #'null (send *robot* :legs))
      (send *robot* :newcoords (make-coords))
    (send *robot* :fix-leg-to-coords (make-coords)))
  (send *robot* :update-descendants)
  ;; generate object model
  (setq *obj* (make-cube 27 128 195))
  (send *obj* :newcoords
        (send (send *robot* :larm :end-coords :copy-worldcoords) :transform
              (make-coords :pos (float-vector 100 0 0) :rpy (mapcar #'deg2rad '(-90 0 90)))))
  (send (send *robot* :larm :end-coords :parent) :assoc *obj*)
  (send *obj* :set-color :blue)
  (objects (list *obj* *robot*))
  ;; ik codes
  (let* ((look-at-face (elt (send *obj* :faces) 2))
         (view-coords
          (make-cascoords
           :coords
           (orient-coords-to-axis
            (send *obj* :copy-worldcoords)
            (v- (cadr (send look-at-face :centroid)) (send *obj* :worldpos)))
           :parent (send *robot* :larm :end-coords :parent)))
         (result
          (send *robot* :fullbody-inverse-kinematics
                (list #'(lambda ()
                          (orient-coords-to-axis
                           (send *robot* :head :end-coords :copy-worldcoords)
                           (v- (send view-coords :worldpos) (send *robot* :head :end-coords :worldpos))))
                      (send *robot* :rleg :end-coords :copy-worldcoords)
                      (send *robot* :lleg :end-coords :copy-worldcoords)
                      #'(lambda ()
                          (orient-coords-to-axis
                           (send view-coords :copy-worldcoords)
                           (v- (send *robot* :head :end-coords :worldpos) (send view-coords :worldpos)))))
                :move-target (append (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:head :rleg :lleg)) (list view-coords))
                :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:head :rleg :lleg :larm))
                :rotation-axis '(:z t t :z)
                :translation-axis '(:z t t :z)
                :debug-view :no-message)))
    (send (send *robot* :larm :end-coords :parent) :dissoc view-coords)
    (send (send *robot* :larm :end-coords :parent) :dissoc *obj*)
    (send *irtviewer* :draw-objects)
    (send look-at-face :draw-on :flush t :color #f(1 0 0) :width 2)
    result))

(defun look-at-ik-all
  ()
  (every #'identity
         (list (head-look-at-ik)
               (hand-look-at-ik)
               (look-at-hand-object-ik)))
  )

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(head-look-at-ik) ;; ik to look target by directing head camera~%")
(warn "(hand-look-at-ik) ;; ik to look target by directing hand camera~%")
(warn "(look-at-hand-object-ik) ;; ik to look object in hand by head camera~%")
(warn "(look-at-ik-all) ;; all ik to look target~%")
