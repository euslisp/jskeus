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

(defun robot-camera-settings ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :arms :shoulder-p :joint-angle -30)
  (send *robot* :arms :shoulder-r :joint-angle 30)
  (send *robot* :arms :shoulder-y :joint-angle 30)
  (send *robot* :arms :elbow-p :joint-angle -60)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :look-at-hand :larm)
  (send *robot* :larm :gripper :joint-angle 60)
  (setq *table* (make-cube 800 1200 5 :pos #f(800 0 500))) (send *table* :set-color :white)
  (setq *ray-target* (make-sphere 20)) (send *ray-target* :set-color :blue)
  (setq *tabletop-target* (make-cylinder 15 200)) (send *tabletop-target* :set-color :green)
  (objects (list *robot* *ray-target* *tabletop-target* *table*))
  (setq *camera* (send *robot* :camera "left-camera"))
  (if (not (boundp '*camera-view*))
      (setq *camera-view* (send *camera* :create-viewer)))
  )

(defun robot-camera-demo ()
  (let  ((uv-list nil)
         (images))
    (dotimes (i 30)
      ;; move robot head
      (send *robot* :head :neck-y :joint-angle -2 :relative t)
      (send *robot* :head :neck-p :joint-angle 0.4 :relative t)
      ;; camera/screen operations
      ;;  : screen-point  ;; "Returns point in screen corresponds to the given pos."
      (setq uv (send *camera* :screen-point (send *robot* :larm :end-coords :worldpos)))
      ;;  : "Returns ray vector of given x and y."
      (setq ray (send *camera* :ray 240 120)) ;; "Returns ray vector of given x and y."
      (setq ray-line (make-line (send *camera* :worldpos) (v+ (send *camera* :worldpos) (scale 1000 ray))))
      (send *ray-target* :locate (v+ (send *camera* :worldpos) (scale 1000 ray)) :world)
      ;;  calc intersection of table and ray-line
      (setq face-top (send *table* :face 1))
      (setq intersect-param (send face-top :intersection (send ray-line :pvertex) (send ray-line :nvertex)))
      (send *tabletop-target* :locate (send ray-line :point intersect-param) :world)
      ;; draw 3d info
      (send *camera* :draw-objects (list *robot* *ray-target* *tabletop-target*))
      (send *irtviewer* :draw-objects :flush nil)
      (send *camera-view* :viewsurface :makecurrent) ;; point-size requre this
      ;; draw camera/screen operations
      ;;  :screen-point
      (send *camera-view* :viewsurface :point-size 10)
      (send *camera-view* :viewsurface :color #f(1 0 0))
      (send *camera-view* :viewsurface :draw-point uv)
      ;;  draw history of :screen-point
      (send *camera-view* :viewsurface :point-size 2)
      (dolist (tmp-uv uv-list) (send *camera-view* :viewsurface :draw-point tmp-uv))
      (send *camera-view* :viewsurface :color #f(1 1 1))
      (send *camera-view* :viewsurface :string (elt uv 0) (elt uv 1) (format nil "larm ~A" uv))
      (push uv uv-list)
      ;;  :ray
      (send *camera-view* :viewsurface :point-size 10)
      (send *camera-view* :viewsurface :color #f(1 1 1))
      (send *camera-view* :viewsurface :draw-point #f(240 120)) ; see *camera* :ray for #f(240 120)
      (send *camera-view* :viewsurface :string 480 120 "ray point")
      ;;  :3d-point
      ;; draw sensor/kinematics info
      (send *irtviewer* :viewer :viewsurface :makecurrent)
      (send ray-line :draw-on :flush nil :color #f(0 0 1) :width 5)
      (send *camera* :draw-on :flush nil)
      (send *robot* :larm :end-coords :draw-on :flush t)
      ;; flush
      (send *camera-view* :viewsurface :flush)
      (push (send *camera* :get-image) images)
      (send *irtviewer* :viewer :viewsurface :flush)
      (x::window-main-one)
      )
    (x::make-animgif-from-images "sample-robot-camera-gif" (nreverse images))
    ))

(robot-camera-settings)
(send *irtviewer* :logging :clear)
(send *irtviewer* :logging :start)
(robot-camera-demo)
(send *irtviewer* :logging :stop)
(send *irtviewer* :save-animgif :fname "robot-camera-demo.gif")

;;
(format t ";; demo to manipulatie camera model (:ray, :screen-point)~%")

