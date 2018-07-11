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
   (shape &key name &allow-other-keys)
   (replace-object self shape)
   (send self :name name)
   shape))

(defclass bumper-model
  :super sensor-model
  :slots (bumper-threshold))
(defmethod bumper-model
  (:init
   (b &rest args &key ((:bumper-threshold bt) 20) name)
   "Create bumper model, b is the shape of an object and bt is the threshold in distance[mm]."
   (setq data 0)
   (setq bumper-threshold bt)
   (send-super* :init b :name name args))
  (:simulate
   (objs)
   "Simulate bumper, with given objects, return 1 if the sensor detects an object and 0 if not."
   (let (r)
     (unless objs (return-from :simulate nil))
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

(defclass camera-model
  :super sensor-model
  :slots ((vwing :forward
                   (:projection :newprojection :screen
                    :view :viewpoint :view-direction :viewdistance
                    :yon :hither))
          img-viewer pwidth pheight))
(defmethod camera-model
  (:init
   (b &rest args
          &key
          ((:width pw) 320) ((:height ph) 240)
          (view-up #f(0 1 0))
          (viewdistance 100.0)
          (hither 100.0)
          (yon 10000.0)
          &allow-other-keys)
   "Create camera model. b is the shape of an object"
   (let ()
     (send-super* :init b args)
     (setq pwidth pw pheight ph)
     (setq vwing
           (instance* perspective-viewing :init
                      :pos (send self :worldpos)
                      :target (send self :transform-vector
                                    (v- (matrix-column (send self :worldrot) 2)))
                      :view-up view-up
                      :viewdistance viewdistance
                      :hither hither :yon yon
                      :screen 1.0
                      args
                      ))
     (send vwing :screen (/ (* viewdistance pw 0.5) viewdistance) (/ (* viewdistance ph 0.5) viewdistance))
     (send self :assoc vwing)
     (send self :newprojection
           (make-matrix 4 4
                        (list (float-vector  viewdistance 0  (/ pw 2) 0) ;; f = 1, t  =0
                              (float-vector  0  viewdistance (/ ph 2) 0) ;;
                              (float-vector  0  0  1  0)
                              (float-vector  0  0  0  1))))
     self))
  (:create-viewer (&optional cv)
   "Create camera viewer, or set viewer"
   (unless (boundp '*irtviewer*) (make-irtviewer))
   (unless cv
     (setq cv
          (view
           :x pwidth :y pheight
           :viewing (send self :viewing)
           :viewsurface
           (instance gl::glviewsurface :create
                     :glcon ((send *irtviewer* :viewer :viewsurface) . gl::glcon)
                     :title (format nil "~A_view" (send self :name)) :width pwidth :height pheight)
           :title (format nil "~A_view" (send self :name))))
     (send cv :viewsurface :makecurrent)
     (let ((default-ambient (float-vector 0.1 0.1 0.1 1.0))
           (default-diffuse (float-vector 1.0 1.0 1.0 1.0))
           (default-specular (float-vector 0.1 0.1 0.1 1.0))
           (scale-light0 1.00)
           (scale-light1 0.70)
           (scale-light2 0.40)
           (scale-light3 0.25)
           light0 light1 light2 light3)
       (setq light0 ;; from back
             (instance gl::lightsource :create 0
                       :ambient (scale scale-light0 default-ambient)
                       :diffuse (scale scale-light0 default-diffuse)
                       :specular (scale scale-light0 default-specular)
                       :position (float-vector 4000.0 3000.0 0.0 0.0)))
       (setq light1 ;; from front-left
             (instance gl::lightsource :create 1
                       :ambient (scale scale-light1 default-ambient)
                       :diffuse (scale scale-light1 default-diffuse)
                       :specular (scale scale-light1 default-specular)
                       :position (float-vector -4000.0 -2000.0 -2000.0 0.0)))
       (setq light2 ;; from front-left
             (instance gl::lightsource :create 2
                       :ambient (scale scale-light2 default-ambient)
                       :diffuse (scale scale-light2 default-diffuse)
                       :specular (scale scale-light2 default-specular)
                       :position (float-vector -2000.0 -2000.0 2500.0 0.0)))
       (setq light3
             (instance gl::lightsource :create 3
                       :ambient (scale scale-light3 default-ambient)
                       :diffuse (scale scale-light3 default-diffuse)
                       :specular (scale scale-light3 default-specular)
                       :position (float-vector 0.0 0.0 0.0 1.0)))
       (send light0 :opengl)
       (send light1 :opengl)
       (send light2 :opengl)
       (send light3 :opengl)
       )
     )
   (setq img-viewer cv))
  (:width () "Returns width of the camera in pixel." pwidth)
  (:height () "Returns height of the camera in pixel." pheight)
  (:viewing (&rest args) (forward-message-to vwing args))
  (:image-viewer (&rest args) (forward-message-to img-viewer args))
  (:fovy () "Returns field of view in degree"
   (let ((proj (send vwing :projection)))
     (* 2 (atan2 (/ pwidth 2.0) (aref proj 0 0)))))
  (:cx () "Returns center x." (let ((proj (send vwing :projection))) (aref proj 0 2)))
  (:cy () "Returns center y." (let ((proj (send vwing :projection))) (aref proj 1 2)))
  (:fx () "Returns focal length of x." (let ((proj (send vwing :projection))) (aref proj 0 0)))
  (:fy () "Returns focal length of y." (let ((proj (send vwing :projection))) (aref proj 1 1)))
  (:screen-point
   (pos)
   "Returns point in screen corresponds to the given pos."
   (let (v w)
     (setq v (send vwing :project3 (send vwing :inverse-transform-vector pos)))
     (setq w (aref v 2))
     (setf (aref v 0) (/ (aref v 0) w))
     (setf (aref v 1) (/ (aref v 1) w))
     (return-from :screen-point (subseq v 0 2))))
  (:3d-point
   (x y d)
   "Returns 3d position"
   (let* ((proj (send vwing :projection))
          (cx (aref proj 0 2)) (cy (aref proj 1 2))
          (f  (aref proj 0 0))
          (tx (/ (aref proj 0 3) f))
          (q (make-matrix 4 4
                          (list
                           (list 1 0           0 (- cx))
                           (list 0 1           0 (- cy))
                           (list 0 0           0 f)
                           (list 0 0 (/ -1.0 tx) 0))))
          r w)
     (setq r (transform q (float-vector x y d 1)))
     (setq w (elt r 3))
     (scale (/ 1.0 w) (subseq r 0 3))))
  (:ray
   (x y)
   "Returns ray vector of given x and y."
   (let* (xx yy u v
             (proj (send vwing :projection))
             (cx (aref proj 0 2)) (cy (aref proj 1 2))
             (tx (aref proj 0 3)) (ty (aref proj 1 3))
             (viewdistance (send vwing :viewdistance))
             (screen (send vwing :screen)))
     (setq xx (- x cx) yy (- y cy))
     (setq u (/ xx ( / pwidth 2.0)) v (/ yy (/ pheight 2.0)))
     (normalize-vector
	(send self :rotate-vector
		(float-vector (* (elt screen 0) u) (* (elt screen 1) v)
                              viewdistance)))))
  (:draw-on
   (&rest args &key ((:viewer vwer) user::*viewer*) &allow-other-keys)
   "Draw camera raw in irtviewer, ex (send cam :draw-on :flush t)"
   (send* self :draw-sensor vwer args))
  (:draw-sensor
   (vwer &key flush (width 1) (color (float-vector 1 1 1)))
   (let* ((yon (send self :yon)) (hither (send self :hither))
          (viewdistance (send self :viewdistance))
          (pwidth (send vwer :viewsurface :line-width))
          (pcolor (send vwer :viewsurface :color))
          (vp (send self :viewpoint))
          (r (/ hither viewdistance))
          (rr (/ yon viewdistance))
          (screen (send self :screen))
          (screenx (car screen)) (screeny (cadr screen)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (gl::glDisable gl::GL_DEPTH_TEST)
     (send vwer :viewsurface :3d-line
           (send self :viewpoint)
           (v+ (send self :viewpoint) (scale -200 (send self :view-direction))))
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
     (gl::draw-glbody vwer self)
     (if flush (send vwer :viewsurface :flush))))
  (:draw-objects (objs)
   "Draw objects in camera viewer, expected type of objs is list of objects"
   (unless img-viewer (send self :create-viewer))
   (send self :draw-objects-raw img-viewer objs))
  (:draw-objects-raw
   (vwr objs)
   (let* (pcurrent pcolor (draw-things (x::draw-things objs))
                   viewpoint viewtarget
                   (proj (send vwing :projection))
                   (cx (aref proj 0 2)) (cy (aref proj 1 2))
                   (f  (aref proj 0 0)))
     (send vwr :viewsurface :makecurrent)
     ;;(resetperspective (send vwr :viewing) (send vwr :viewsurface))
     (if (> pwidth pheight)
         (gl::glviewport 0 (- (/ (- pwidth pheight) 2)) pwidth pwidth)
       (gl::glviewport (- (/ (- pheight pwidth) 2)) 0 pheight pheight))
     (gl::glMatrixMode gl::GL_PROJECTION)
     (gl::glLoadIdentity)
     (let ((fv (instantiate float-vector 16)))
       ;; projection matrix on OpenGL
       ;; f 0  A 0
       ;; 0 f  B 0
       ;; 0 0  C D
       ;; 0 0 -1 0
       (setf (elt fv 0) (/ 1.0 (tan (/ (send self :fovy) 2.0)))) ;; f
       (setf (elt fv 5) (elt fv 0)) ;; f
       (setf (elt fv 8)
             (/ (* 2 (- (/ (1- pwidth) 2.0) cx))
                (if (> pwidth pheight) pwidth pheight))) ;; A
       (setf (elt fv 9)
             (/ (* -2 (- (/ (1- pheight) 2.0) cy))
                (if (> pwidth pheight) pwidth pheight))) ;; B
       (setf (elt fv 10)
             (/ (+ (send self :viewing :yon)
                   (send self :viewing :hither))
                (- (send self :viewing :hither)
                   (send self :viewing :yon)))) ;; C
       (setf (elt fv 14)
             (/ (* 2 (send self :viewing :yon)
                   (send self :viewing :hither))
                (- (send self :viewing :hither)
                   (send self :viewing :yon)))) ;; D
       (setf (elt fv 11) -1)
       (gl::glMultMatrixd fv))

     (setq viewpoint (send self :worldpos))
     ;; glview define view-directoin to oppsite direction
     (setq viewtarget
           (v- viewpoint (send self :viewing :view-direction)))
     (gl::gluLookAtfv (concatenate vector viewpoint viewtarget
                                   (v- (send self :viewing :view-up))))
     (gl::glMatrixMode gl::GL_MODELVIEW)

     (send vwr :viewsurface :clear)
     (setq pcolor (send vwr :viewsurface :color))
     (send vwr :viewsurface :color pcolor)

     (gl::glEnable gl::GL_LIGHTING)
     (dolist (abody draw-things)
      ;; draw body
      (cond
       ((find-method abody :draw)
	(send abody :draw vwr))
       ((derivedp abody faceset)
	(gl::draw-glbody vwr abody))
       ((find-method abody :draw-on)
	(send abody :draw-on :viewer vwr))
       (t (warn "Unknown body to draw ~A~%" abody)))
       )
     (send vwr :viewsurface :flush)
     ))
  (:get-image
   (&key (with-points) (with-colors))
   "Get image objects you need to call :draw-objects before calling this function"
   (let (points colors img pc)
     (if with-points
         (setq points (make-matrix (* pwidth pheight) 3)))
     (if with-colors
         (setq colors (make-matrix (* pwidth pheight) 3)))
     (setq img (send self :get-image-raw img-viewer :points points :colors colors))
     (cond
      ((and with-points with-colors)
       (setq pc (instance pointcloud :init
                          :height pheight :width pwidth
                          :points points :colors colors)))
      (with-points
       (setq pc (instance pointcloud :init
                          :height pheight :width pwidth
                          :points points))))
     (if with-points
         (cons img pc)
       img)))
  (:get-image-raw (vwr &key (points) (colors))
   (let* ((sf (send vwr :viewsurface))
          (width (send sf :width))
          (height (send sf :height))
          (cx (send self :cx))
          (cy (send self :cy))
          (num (* width height))
          (fovy (send vwing :fovy))
          (np (send vwing :hither))
          (fp (send vwing :yon))
          (focus (/ (/ height 2.0) (tan (/ fovy 2.0))))
          (mptr 0)
          (vptr 0)
          (pos (float-vector 0 0 0))
          (col (float-vector 0 0 0))
          mat-ent img-ent col-ent fv
          img)
     (when points
       (unless (and (= width pwidth) (= height pheight))
         (warn ";; width: %d /= %d or height: %d /= %d~%" width pwidth height pheight)
         (return-from :get-image-raw))
       (setq fv (make-array num :element-type :float))
       (setq mat-ent (array-entity points))
       (fill mat-ent 0.0)
       (when colors
         (setq col-ent (array-entity colors))
         (fill col-ent 0.0)))
     ;; get image
     (setq img (send sf :getglimage :depthbuf fv))
     (when points
       (setq img-ent (send img :entity))
       (dotimes (y height)
         (dotimes (x width)
           (if (< (elt fv vptr) 1.0)
               (let ((zpos (/ (* fp np) (- (* (elt fv vptr) (- fp np)) fp))))
                 (setq mptr (* 3 (+ (* (- height y 1) width) x)))
                 (setf (elt pos 0) (* (- cx x) (/ zpos focus)))
                 (setf (elt pos 1) (* (- cy (- height y 1)) (/ zpos focus)))
                 (setf (elt pos 2) (- zpos)) ;; ok
                 (sys::vector-replace mat-ent pos mptr)
                 (when colors
                   (setf (elt col 0)
                         (/ (sys::peek img-ent mptr :byte) 255.0))
                   (setf (elt col 1)
                         (/ (sys::peek img-ent (+ mptr 1) :byte) 255.0))
                   (setf (elt col 2)
                         (/ (sys::peek img-ent (+ mptr 2) :byte) 255.0))
                   (sys::vector-replace col-ent col mptr))))
           (incf vptr 1)
           )))
     img
     ))
  (:select-drawmode (mode objs)
   "Change drawmode for drawing with :draw-objects methods. mode is symbol of mode, 'hid is symbol for hidden line mode, the other symbols indicate default mode. objs is the same objects using :draw-objects."
   (let ((glcon ((send img-viewer :viewsurface) . gl::glcon)))
     (dolist (obj (x::draw-things objs))
       (let ((hid (get obj :gl-hiddenline)))
         (gl::delete-displaylist-id (get obj :GL-DISPLAYLIST-ID))
         (setf (get obj :GL-DISPLAYLIST-ID) nil)
         (case mode
          (user::hid
           (unless (assq glcon hid)
             (setf (get obj :gl-hiddenline)
                   (append hid (list (cons glcon t))))))
          (t
           (setf (get obj :gl-hiddenline) nil))
          )))
     ))
  )

;; utility functions
(defun make-camera-from-param (&key pwidth pheight fx fy cx cy (tx 0) (ty 0) parent-coords name
                                    create-viewer)
  "Create camera object from given parameters."
  (let* ((b (body+ (make-cube 40 30 30)
                   (send (make-cylinder 2 30) :rotate pi/2 :x)
                   (make-cylinder 13 25)))
	 c (vd 1.0))
    (setq c (instance camera-model :init b :width pwidth :height pheight
                      :viewdistance vd :name name))
    (send (c . vwing) :screen (/ (* vd pwidth 0.5) fx) (/ (* vd pheight 0.5) fy))
    (send c :newprojection
          (make-matrix 4 4
                       (list (float-vector fx  0 cx (* fx tx))
                             (float-vector  0 fy cy (* fy ty))
                             (float-vector  0  0  1  0)
                             (float-vector  0  0  0  1))))
    (if parent-coords (send c :newcoords (send parent-coords :copy-worldcoords)))
    (send c :translate (float-vector (- tx) (- ty) 0))
    (send (c . vwing) :translate (float-vector tx ty 0))
    (if parent-coords (send parent-coords :assoc c))
    (if create-viewer (send c :create-viewer))
    c))

(in-package "GEOMETRY")

(provide :irtsensor "$Id: $")


