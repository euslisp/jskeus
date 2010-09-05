;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
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

(in-package "X")

(defun make-lr-ud-coords (lra uda)
  (let ((c (geo::make-coords)))
    (send c :rotate (deg2rad lra) :z)
    (send c :rotate (- (deg2rad uda)) :y)
    c))

(defmethod geo::viewer
  (:draw-objects
    (&rest args)
    (send* (get self :pickviewer) :draw-objects args)))

(defclass irtviewer
  :super x::panel
  :slots (viewer
	  objects draw-things
	  previous-cursor-pos
	  left-right-angle
	  up-down-angle
	  viewpoint
	  viewtarget
	  drawmode))
(defmethod irtviewer
  (:create
   (&rest args
	  &key (title "IRT viewer")
	  (hither 200.0) (yon 50000.0)
	  (width 500) (height 500)
	  &allow-other-keys)
   (let ()
     (setq left-right-angle 60
	   up-down-angle 20
	   viewpoint (float-vector 700 400 250)
	   viewtarget (float-vector 0 0 0))
     (send-super* :create :width width :height height :title title
		  :event-mask '(:configure) args)
     (setq gl::*perspective-far* yon)
     (setq gl::*perspective-near* hither)
     (setq viewer (apply #'user::view :parent self
			 :x 0 :y 0
			 :width width :height height 
			 :title title
			 :name (gensym "title")
			 args))
     (setf (get viewer :pickviewer) self)
     (setq user::*viewer* viewer)

     (setq gl::*light0* 
       (instance gl::lightsource :create 0
		 :ambient (float-vector 0.2 0.2 0.2 1.0)
		 :diffuse (float-vector 0.6 0.6 0.6 1.0)
		 :specular (float-vector 0.4 0.4 0.4 1.0)
		 :position (float-vector 1.0 2.0 1.5 0.0)))
     (setq gl::*light1*
       (instance gl::lightsource :create 1
		 :ambient (float-vector 0.1 0.1 0.1 1.0)
		 :diffuse (float-vector 0.2 0.2 0.2 1.0)
		 :specular (float-vector 0.4 0.4 0.4 1.0)
		 :position (float-vector -1.0 -2.0 0.0 0.0)))
     (setq  gl::*light2*
       (instance gl::lightsource :create 2
		 :ambient (float-vector 0.1 0.1 0.1 1.0)
		 :diffuse (float-vector 0.2 0.2 0.2 1.0)
		 :specular (float-vector 0.4 0.4 0.4 1.0)
		 :position (float-vector -1.0 -2.0 -1.0 0.0)))
     (send gl::*light0* :opengl)
     (send gl::*light1* :opengl)
     (send gl::*light2* :opengl)
     ;;
     (send viewer :viewsurface :set-event-proc
	   :buttonrelease-left :draw-event self)
     (send viewer :viewsurface :set-event-proc
	   :buttonpress :set-cursor-pos-event self)
     (send viewer :viewsurface :set-event-proc
	   :motionnotify-left :move-coords-event self)

     (send self :look1)
     (send self :draw-objects)
     self))
  (:viewer (&rest args) (user::forward-message-to viewer args))
  (:redraw () (send self :draw-objects))
  ;;
  (:expose (event) )
  (:resize (newwidth newheight)
   (setq x::width newwidth x::height newheight)
   (send-super  :resize newwidth newheight)
   (send viewer :viewsurface :resize newwidth newheight)
   (send viewer :viewsurface :makecurrent)
   (send viewer :viewsurface :3d-mode)
   (send self :redraw))
  (:configureNotify
   (event)
   (if *debug* (warn "ConfigureNotify came to ~S~%" self))
   (let ((newwidth (send-message self x::xwindow :width))
	 (newheight (send-message self x::xwindow :height)))
     (when (or (/= newwidth x::width) (/= newheight x::height))
       (send self :resize newwidth newheight))))
  ;;
  (:viewtarget (&optional p) (if p (setq viewtarget p)) viewtarget)
  (:viewpoint  (&optional p) (if p (setq viewpoint p))  viewpoint)
  (:look1
   (&optional (vt viewtarget)
       (lra left-right-angle) (uda up-down-angle))
   (let* ((c (make-lr-ud-coords lra uda))
	  (vp
	   (v+ vt (send c :transform-vector
			(float-vector (norm (v- viewpoint vt)) 0 0))))
	  (view-up (send c :rotate-vector (float-vector 0 0 1))))
     (send self :viewpoint vp)
     (if (send self :viewer :viewing)
	 (send self :viewer :viewing :look viewpoint vt view-up))
     ))
  (:look-all
   (&optional (bbox (geo::make-bounding-box (flatten (send-all x::draw-things :vertices)))))
   (when bbox
     (let (vt vp)
       (setq vt (send bbox :center))
       (setq vp
	 (scale
	  (/ (apply #'max (coerce (send bbox :diagonal) cons))
	     (* 0.5 (tan (send self :viewer :viewing :view-angle))))
	  (normalize-vector (v- viewpoint vt))))
       (send self :viewtarget vt)
       (send self :viewpoint vp)
       ))
   (send self :look1)
   (send self :draw-objects))
  (:move-viewing-around-viewtarget
   (event x y dx dy vwr)
   (cond
       ((or (event-right event)       
	    (and (< 60 x (- (send vwr :viewsurface :width) 60))
		 (< 60 y (- (send vwr :viewsurface :height) 60))))
	(setq left-right-angle (- left-right-angle dx))
	(setq up-down-angle (+ up-down-angle dy)))
       ((or (> x (- (send vwr :viewsurface :width) 60)) (< y 60))
	(send self :viewpoint
	      (v+ viewpoint
		  (scale (- (* 10 (if (< y 60) dx dy)))
			 (normalize-vector (v- viewtarget viewpoint))))))
       ((or (> y (- (send vwr :viewsurface :height) 60)) (< x 60))
	(let* ((v (send vwr :viewing :transform-vector
			 (if (< x 60)
			     (float-vector 0 (* 2 dy) 0)
			   (float-vector (* -2 dx) 0 0))))
	       (tp (send self :viewtarget))
	       (dv (v- v viewpoint)))
	  (send self :viewpoint v)
	  (send self :viewtarget (v+ tp dv))
	  ))))
  (:set-cursor-pos-event
   (event)
   (setq previous-cursor-pos (event-pos event)))
  (:move-coords-event
   (event)
   (if (null previous-cursor-pos)
       (setq previous-cursor-pos (event-pos event)))
   (let* ((current-pos (event-pos event))
	  (x (elt current-pos 0))
	  (y (elt current-pos 1))
	  (dx (- x (elt previous-cursor-pos 0)))
	  (dy (- y (elt previous-cursor-pos 1)))
	  )
     (send self :move-viewing-around-viewtarget event x y dx dy viewer)
     (send self :look1)
     (send self :draw-objects)
     (setq previous-cursor-pos current-pos)))
  (:draw-event
   (event)
   (send self :draw-objects))
  (:draw-objects
   (&rest args)
   (send viewer :viewsurface :makecurrent)
   (apply #'gl::draw-globjects viewer draw-things args))
  (:objects
   (&rest args)
   (when
       args
     (cond
	((null (car args))
	 (setq objects nil))
	((consp (car args))
	 (setq objects (car args)))
	((atom (car args))
	 (setq objects (append args objects)))
	 ))
   (send-all objects :worldcoords)
   (setq draw-things (draw-things objects))
   objects)
  (:select-drawmode
   (mode)
   (when (not (eq mode drawmode))
     (dolist (obj draw-things)
       (setf (get obj :GL-DISPLAYLIST-ID) nil)
       (case mode
         (user::hid
          (setf (get obj :gl-hiddenline) (list (cons nil t))))
         (t
          (setf (get obj :gl-hiddenline) nil)))))
   (setq drawmode mode)
   drawmode)
  (:flush () (if viewer (send viewer :viewsurface :glflush)))
  )
   
(defun draw-things (objs)
  (cond
   ((atom objs)
    (cond
     ((null objs) nil)
     ((find-method objs :draw) (list objs))
     ((find-method objs :bodies) (send objs :bodies))
     (t (list objs))))
   (t
    (append (draw-things (car objs))
	    (draw-things (cdr objs))))
   ))


(in-package "GEOMETRY")

;;
;; re-definition
;;
(defmethod viewing
  (:look
   (from &optional (to (float-vector 0 0 0)) (view-up (float-vector 0 0 1)))
   (let* ((view-direction (normalize-vector (v- to from)))
	  (v1 (v* view-direction view-up))
	  view-right)
     (if (< (norm v1) *epsilon*)
	 (setq view-right (normalize-vector
			   (v* view-direction (float-vector -1 0 0))))
       (setq view-right (normalize-vector v1)))
     (setq view-up (normalize-vector (v* view-right view-direction)))
     (setq view-direction (scale -1.0 view-direction))
     (setf (array-entity rot)
	   (concatenate float-vector view-right view-up view-direction))
     (transpose rot rot)
     (send self :locate from :world)
     (send self :worldcoords))
   )
  )

(in-package "USER")

(defun objects (&optional (objs t) vw)
  (when
      (and objs (not (boundp '*viewer*)) (null vw))
    (warn "do (make-viewer) beforehand or use (objects objs {viewer})~%")
    (warn "or use (send your-viewer :objects your-objects)~%")
    (return-from objects nil))
  (if (null vw) (setq vw *irtviewer*))
  (let (objects)
  (if (eq objs t) (setq objects (send vw :objects))
    (setq objects (send vw :objects objs)))
  (send vw :look-all)
  objects))

;;
;;
(proclaim '(special *irtviewer*))
(defun make-irtviewer (&rest args)
  (setq *irtviewer* (send-lexpr (instantiate x::irtviewer) :create args)))

(in-package "GL")

(provide :irtviewer "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.8  2010-02-04 01:23:30  k-okada
;;; :make-current before gl::draw-globjects in irtviewer
;;;
;;; Revision 1.7  2009/11/08 10:36:17  k-okada
;;; set :name using gensym, for (find-viewer) in send irtviewer :create
;;;
;;; Revision 1.6  2009/11/08 09:43:12  k-okada
;;; make-irtviewer :title works
;;;
;;; Revision 1.5  2009/07/31 11:10:31  ueda
;;; add :draw-objects to geo::viewer for (pickview :separate t)
;;;
;;; Revision 1.4  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.3  2009/01/06 13:18:23  k-okada
;;; check event-type and window-id for skipping x event, use :expose for each item
;;;
;;; Revision 1.2  2008/09/22 06:02:10  k-okada
;;; fix to work with jskeusgl
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
