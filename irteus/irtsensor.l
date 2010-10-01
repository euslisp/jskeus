;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: irtsensor.l $
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
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;

(in-package "USER")

(defclass sensor-model
  :super body
  :slots (data profile))
(defmethod sensor-model
  (:profile (&optional p) (if p (setq profile p)) profile)
  (:signal (rawinfo) (warn ":signal should be defined in lower class~%"))
  (:simulate (model) (warn ":simulate should be defined in lower class~%"))
  (:read () data)
  (:draw-sensor (v) (warn ":draw-sensor should be defined in lower class~%"));; virtual method
  (:init
   (shape &key name)
   (replace-object self shape)
   (send self :name name)
   shape))

(defclass bumper
  :super sensor-model
  :slots (bumper-threshold))
(defmethod bumper
  (:init
   (&rest args &key ((:bumper-threshold bt) 20))
   (setq data 0)
   (setq bumper-threshold bt)
   (send-super* :init args))
  (:simulate
   (objs)
   (let (r)
     (if (atom objs) (setq objs (list objs)))
     (setq
      data
      (if (some #'(lambda (obj)
                    (setq r (car (pqp-collision-distance self obj)))
                    (< r bumper-threshold)) objs) 1 0))))
  (:draw (vwer) (send self :draw-sensor vwer))
  (:draw-sensor
   (vwer)
   (setf (get self :backup-color) (get self :face-color))
   (if (= data 1)
       (send self :set-color :red)
     (send self :set-color (get self :backup-color)))
   (gl::draw-glbody vwer self)
   (setf (get self :face-color) (get self :backup-color))
   )
  )

(defclass camera
  :super sensor-model
  :slots ((viewing :forward
                   (:projection :newprojection
                    :view :viewpoint :view-direction :viewdistance
                    :yon :hither))
          pwidth pheight))
(defmethod camera
  (:init
   (b &rest args
          &key
          ((:width pw) 320) ((:height ph) 240)
          (view-up #f(-1 0 0))
          (viewdistance 5.0)
          (hither 100.0)
          (yon 10000.0)
          &allow-other-keys)
   (let ()
     (send-super :init b)
     (setq pwidth pw pheight ph)
     (setq viewing
           (instance* perspective-viewing :init
                      :pos (send self :worldpos)
                      :target (send self :transform-vector
                                    (matrix-column (send self :worldrot) 2))
                      :view-up view-up
                      :viewdistance viewdistance
                      :hither hither :yon yon
                      :screen 1.0
                      args
                      ))
     (send self :assoc viewing)
     self))
  (:width () pwidth)
  (:width () pheight)
  (:screen-point
   (pos)
   (let (v w cx cy)
     (setq v (send self :view (transform #2f((1 0 0)(0 -1 0)(0 0 1)) pos)))
     (setq w (- (aref v 3)))
     (setf (aref v 0) (/ (aref v 0) w))
     (setf (aref v 1) (/ (aref v 1) w))
     (return-from :screen-point (subseq v 0 2))))
  (:ray
   (x y)
   (let* (xx yy
             (proj (send viewing :projection))
             (cx (aref proj 0 2)) (cy (aref proj 1 2))
             (tx (aref proj 0 3)) (ty (aref proj 1 3)))
     (setq xx (- x cx tx) yy (- y cy ty))
     (send viewing :ray (/ xx pwdith) (/ (- yy) pheight))))
  (:draw-sensor
   (vwer)
   (let* ((yon (send self :yon)) (hither (send self :hither))
          (viewdistance (send self :viewdistance))
          (pwidth (send vwer :viewsurface :line-width))
          (pcolor (send vwer :viewsurface :color))
          (color (float-vector 1 1 1))
          (vp (send self :viewpoint))
          (r (/ hither viewdistance))
          (rr (/ yon viewdistance))
          (screenx 1.0) (screeny 1.0))
     (send vwer :viewsurface :line-width 1)
     (send vwer :viewsurface :color color)
     (gl::glDisable gl::GL_DEPTH_TEST)
     (send vwer :viewsurface :3d-line
           (send self :viewpoint)
           (v+ (send self :viewpoint) (scale 200 (send self :view-direction))))
     (dolist (s (list (cons screenx screeny)
                      (cons screenx (- screeny))
                      (cons (- screenx) screeny)
                      (cons (- screenx) (- screeny))))
       (send vwer :viewsurface :3d-line
             (send self :transform-vector
                   (float-vector (* rr (car s)) (* rr (cdr s)) yon))
             (send self :transform-vector
                   (float-vector (* r  (car s)) (* r (cdr s)) hither))))
     (gl::glEnable gl::GL_DEPTH_TEST)
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (gl::draw-glbody vwer self)))
  )

(in-package "GEOMETRY")

(provide :irtsensor "$Id: $")


