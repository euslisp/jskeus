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

(in-package "USER")

(require :irtgeo)
(require :irtutil)

(defclass joint
  :super propertied-object
  :slots (parent-link child-link joint-angle min-angle max-angle default-coords))

(defmethod joint
  (:init (&key (name :joint) ((:child-link clink)) ((:parent-link plink))
	       (min -90) (max 90) &allow-other-keys)
	 (send self :name name)
	 (setq parent-link plink child-link clink
	       min-angle min max-angle max)
	 (setq default-coords (send child-link :copy-coords))
	 self)
  (:min-angle (&optional v) (if v (setq min-angle v)) min-angle)
  (:max-angle (&optional v) (if v (setq max-angle v)) max-angle)
  (:parent-link (&rest args) (user::forward-message-to parent-link args))
  (:child-link (&rest args) (user::forward-message-to child-link args))
  )

(defclass rotational-joint
  :super joint
  :slots (axis))

(defmethod rotational-joint
  (:init (&rest args &key ((:axis ax) :z) &allow-other-keys)
	 (setq axis ax)
	 (setq joint-angle 0.0)
	 (send-super* :init args)
	 ;; set default value
	 (if (null min-angle) (setq min-angle -90.0))
	 (if (null max-angle) (setq max-angle (+ 180.0 min-angle)))
	 self)
  (:joint-angle
   (&optional v &key relative)
   (let ()
     (when v
       (if relative (setq v (+ v joint-angle)))
       (when (>= v max-angle)
	 (setq v max-angle))
       (when (<= v min-angle)
	 (setq v min-angle))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  )

(defclass linear-joint
  :super joint
  :slots (axis))

(defmethod linear-joint
  (:init (&rest args &key ((:axis ax) :z) &allow-other-keys)
	 (setq axis 
	   (if (float-vector-p ax) 
	       ax
	     (case ax (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
		   (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
		   (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1)))))
	 (setq joint-angle 0.0)
	 (send-super* :init args)
	 ;; set default value
	 (if (null min-angle) (setq min-angle -90.0))
	 (if (null max-angle) (setq max-angle  90.0))
	 self)
  (:joint-angle
   (&optional v &key relative)
   (let ()
     (when v
       (if relative (setq v (+ v joint-angle)))
       (when (>= v max-angle)
	 (setq v max-angle))
       (when (<= v min-angle)
	 (setq v min-angle))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :translate (scale joint-angle axis)))
     joint-angle))
  )

(defclass omniwheel-joint
  :super joint
  :slots (axis))

(defmethod omniwheel-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf*))
                &allow-other-keys)
       (setq joint-angle (float-vector 0 0 0))
       (setq axis (list #f(1 0 0) #f(0 1 0) :z))
       (send-super* :init :min min :max max args)
       ;; set default value
       self)
  (:joint-angle
   (&optional v &key relative)
   (let (relang relx rely)
     (unless relative
       (if v (warn "wheel-joint does not support non-relative mode??~%"))
       (return-from :joint-angle joint-angle))
     (when v
       (if relative (setq v (v+ v joint-angle)))
       (setq relx   (- (elt v 0) (elt joint-angle 0))
	     rely   (- (elt v 1) (elt joint-angle 1))
	     relang (- (elt v 2) (elt joint-angle 2))
	     joint-angle v)
       (send child-link :translate (float-vector relx rely 0))
       (send child-link :rotate (deg2rad relang) :z))
     joint-angle
     ))
  )

(defclass wheel-joint
  :super joint
  :slots (axis))

(defmethod wheel-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf*))
                &allow-other-keys)
	 (setq joint-angle (float-vector 0 0 0))
         (setq axis (list #f(1 0 0) :z))
	 (send-super* :init :min min :max max args)
	 ;; set default value
	 self)
  (:joint-angle
   (&optional v &key relative)
   (let (relvel relang)
     (unless relative
       (if v (warn "wheel-joint does not support non-relative mode??~%"))
       (return-from :joint-angle joint-angle))
     (when v
       (setq relvel (elt v 0) relang (elt v 1))
       (send child-link :translate (float-vector relvel 0 0))
       (send child-link :rotate (deg2rad relang) :z)
       )
     joint-angle))
  )

(defclass sphere-joint
  :super joint
  :slots (axis))

(defmethod sphere-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf*))
                &allow-other-keys)
	 (setq joint-angle (float-vector 0 0 0))
         (setq axis (list :z :y :x))
	 (send-super* :init :min min :max max args)
	 ;; set default value
	 self)
  (:joint-angle
   (&optional v &key relative)
   (let ()
     (when v
       (if relative (setq v (v+ v joint-angle)))
       (setq v (vmax v min-angle))
       (setq v (vmin v max-angle))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad (elt joint-angle 0)) :z)
       (send child-link :rotate (deg2rad (elt joint-angle 1)) :y)
       (send child-link :rotate (deg2rad (elt joint-angle 2)) :x))
     joint-angle))
  )

(defclass 6dof-joint
  :super joint
  :slots (axis))

(defmethod 6dof-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf* *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf* *inf* *inf* *inf*))
                &allow-other-keys)
	 (setq joint-angle (float-vector 0 0 0 0 0 0))
         (setq axis (list #f(1 0 0) #f(0 1 0) #f(0 0 1) :z :y :x))
	 (send-super* :init :min min :max max args)
	 ;; set default value
	 self)
  (:joint-angle
   (&optional v &key relative)
   (let ()
     (when v
       (if relative (setq v (v+ v joint-angle)))
       (setq v (vmax v min-angle))
       (setq v (vmin v max-angle))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :translate (subseq joint-angle 0 3))
       (send child-link :rotate (deg2rad (elt joint-angle 3)) :z)
       (send child-link :rotate (deg2rad (elt joint-angle 4)) :y)
       (send child-link :rotate (deg2rad (elt joint-angle 5)) :x))
     joint-angle))
  )

;;;
;;;

(defclass bodyset-link
  :super bodyset
  :slots (joint parent-link child-links analysis-level default-coords
                weight acentroid inertia-tensor))
(defmethod bodyset-link
  (:init 
   (coords
    &rest args
    &key
    ((:analysis-level level) :body)
    ((:weight w) 1)
    ((:centroid c) #f(0 0 0))
    ((:inertia-tensor i) (unit-matrix 3))
    &allow-other-keys)
   (setq analysis-level level)
   (setq weight w inertia-tensor i acentroid c)
   (send-super* :init coords args))
  (:worldcoords
   (&optional (level analysis-level))
   (case
    level
    (:coords (send-message self cascaded-coords :worldcoords))
    (t       (send-super :worldcoords)))
   )
  (:analysis-level
   (&optional v)
   (if v (setq analysis-level v)) analysis-level)
  (:weight
   (&optional w)
   (if w (setq weight w)) weight)
  (:centroid
   (&optional c)
   (if c (setq acentroid c))
   (send (send self :worldcoords) :transform-vector acentroid))
  (:inertia-tensor
   (&optional i)
   (if i (setq inertia-tensor i)) inertia-tensor)
  (:joint (&rest args) (user::forward-message-to joint args))
  (:add-joint (j) (setq joint j))
  (:del-joint () (setq joint nil))

  (:parent-link () parent-link)
  (:child-links () child-links)

  (:add-child-links (l) (unless (or (member l child-links) (not l))(push l child-links)))
  (:add-parent-link (l) (setq parent-link l))

  (:del-child-link  (l) (setq child-links (delete l child-links)))
  (:del-parent-link ()  (setq parent-link nil))

  (:default-coords (&optional c) (if c (setq default-coords c)) default-coords)
  )

(defclass cascaded-link
  :super cascaded-coords
  :slots (links joint-list bodies
		collision-avoidance-links))

(defmethod cascaded-link
  (:init (&rest args
		&key name
		&allow-other-keys)
	 (send-super-lexpr :init args)
	 self)
  (:init-ending
   ()
   (setq bodies (flatten (send-all links :bodies)))
   (dolist (j joint-list)
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (send (send j :parent-link) :add-child-links (send j :child-link))
     )
   (send self :update-descendants))
  (:links (&rest args) (user::forward-message-to-all links args))
  (:joint-list (&rest args) (user::forward-message-to-all joint-list args))

  (:bodies (&rest args) (user::forward-message-to-all bodies args))
  (:faces () (flatten (send-all bodies :faces)))

  (:update-descendants
   (&rest args)
   (send-all links :worldcoords))

  (:angle-vector
   (&optional vec 
	      (angle-vector (instantiate float-vector (length joint-list))))
   (dotimes (i (length angle-vector))
     (setf (elt angle-vector i)
	   (if vec
	       (send (elt joint-list i) :joint-angle (elt vec i))
	     (send (elt joint-list i) :joint-angle))))
   angle-vector)
  ;;
  (:find-link-route
   (to &optional from)
   (let ((pl (send to :parent-link)))
     (cond
      ((and pl (not (eq to from)))
       (append (send self :find-link-route pl from) (list to)))
      ((and pl (eq to from))
       (list from)))))
  (:link-list
   (to &optional from)
   (let (ret1 ret2)
     (setq ret1 (send self :find-link-route to from))
     (when (and from (not (eq from (car ret1))))
       (setq ret2 (send self :find-link-route from (car ret1)))
       (setq ret1 (nconc (nreverse ret2) ret1))
       )
     ret1))
  )

;;;
;;; for ik
;;; 
(defmethod cascaded-link
  (:calc-target-axis-dimension
   (rotation-axis translation-axis)
   (let ((dim 6))
     (dolist (axis (list translation-axis rotation-axis))
       (case axis
	     ((:x :y :z :xx :yy :zz) (decf dim 1))
	     ((:xy :yx :yz :zy :zx :xz) (decf dim 2))
	     (nil (decf dim 3))))
     dim))
  (:calc-target-joint-dimension
   (link-list)
   (calc-target-joint-dimension (send-all link-list :joint)))
  (:calc-inverse-jacobian
   (jacobi &rest args
    &key
    ((:manipulability-limit ml) 0.1)
    ((:manipulability-gain mg) 0.001)
    weight debug-view
    ret wmat tmat umat umat2 mat-tmp
    mat-tmp-rc tmp-mrr tmp-mrr2
    &allow-other-keys)
   (let (jacobi# m m2 (k 0))
     ;; m : manipulability
     (setq m (manipulability jacobi tmp-mrr tmat))
     (if (< m ml) (setq k (* mg (expt (- 1.0 (/ m  ml)) 2))))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "k     :~7,3f (manipulability:~7,3f, gain:~7,3f, limit:~7,3f, len:~d)~%" k m mg ml (cadr (array-dimensions jacobi))))

     ;; calc weighted SR-inverse
     (setq jacobi# (sr-inverse jacobi k weight 
			       ret wmat tmat umat umat2 mat-tmp
			       mat-tmp-rc tmp-mrr tmp-mrr2
			       ))
     jacobi#))
  (:calc-gradH-from-link-list
   (link-list &optional (res (instantiate float-vector (length link-list))))
   (let* ((j-l (send-all link-list :joint))
	  (angle-list (send-all j-l :joint-angle))
	  (min-angle-list (send-all j-l :min-angle))
	  (max-angle-list (send-all j-l :max-angle))
	  (angle-range-list (map cons #'- max-angle-list min-angle-list))
	  (mid-range-list (map cons #'(lambda (x y) (/ (+ x y) 2.0))
			       max-angle-list min-angle-list)))
     (dotimes (i (length link-list) res)
       (setf (elt res i)
	     (/ (- (elt mid-range-list i) (elt angle-list i))
		(elt angle-range-list i))))
     res))
  (:calc-jacobian-child-rotate-vector
   (paxis child-link move-target tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (setq tmp-v3 (normalize-vector (send child-link :rotate-vector paxis tmp-v3) tmp-v3))
   (setq tmp-v3
     (v- (send move-target :inverse-transform-vector tmp-v3 tmp-v3 tmp-v3b tmp-m33)
	 (send move-target :inverse-transform-vector #f(0 0 0) tmp-v3a tmp-v3b tmp-m33)  tmp-v3))
   tmp-v3)
  (:calc-jacobian-linear-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (setq tmp-v3 (send self :calc-jacobian-child-rotate-vector
			paxis child-link move-target tmp-v3 tmp-v3a tmp-v3b tmp-m33))

     (setq pos-r (calc-dif-with-axis tmp-v3 translation-axis tmp-v0 tmp-v1 tmp-v2))
     (setq pos-r (scale 0.001 pos-r pos-r));; scale [mm] -> [m]
     (dotimes (j (length pos-r)) (setf (aref fik j i) (elt pos-r j)))
     (setq pos-i (calc-dif-with-axis #f(0 0 0) rotation-axis tmp-v0 tmp-v1 tmp-v2))
     (dotimes (j (length pos-i)) (setf (aref fik (+ j (length pos-r)) i) (elt pos-i j)))
     ))
  (:calc-jacobian-rotational-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (setq tmp-v3 (send self :calc-jacobian-child-rotate-vector
			paxis child-link move-target tmp-v3 tmp-v3a tmp-v3b tmp-m33))

     (setq pos-i (scale (if child-reverse 1 -1)
			(send move-target :inverse-transform-vector
			      (send child-link :worldpos) tmp-v3a tmp-v3b tmp-m33) tmp-v3a))
     (setq pos-i (scale 0.001 pos-i pos-i));; scale [mm] -> [m]
     
     (setq pos-r (v* tmp-v3 pos-i tmp-v3b))
     (setq pos-r (calc-dif-with-axis pos-r translation-axis tmp-v0 tmp-v1 tmp-v2))
     (dotimes (j (length pos-r)) (setf (aref fik j i) (elt pos-r j)))
     (setq pos-i (calc-dif-with-axis tmp-v3 rotation-axis tmp-v0 tmp-v1 tmp-v2))
     (dotimes (j (length pos-i)) (setf (aref fik (+ j (length pos-r)) i) (elt pos-i j)))
     ))
  (:calc-jacobian-wheel-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik i joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 1) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ;;(setf (aref fik 2 1) 0)
     ))
  (:calc-jacobian-omniwheel-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik i joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik (+ i 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 2) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-sphere-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-rotational-joint
	   fik i joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 2) joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-6dof-joint
   (fik i joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik i joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik (+ i 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik (+ i 2) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 3) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 4) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik (+ i 5) joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-from-link-list
   (link-list &rest args 
	      &key move-target (rotation-axis nil) (translation-axis t)
	      (dim (send self :calc-target-axis-dimension rotation-axis translation-axis))
              (fik-len (send self :calc-target-joint-dimension link-list))
	      fik
	      (tmp-v0 (instantiate float-vector 0))
	      (tmp-v1 (instantiate float-vector 1))
	      (tmp-v2 (instantiate float-vector 2))
	      (tmp-v3 (instantiate float-vector 3))
	      (tmp-v3a (instantiate float-vector 3))
	      (tmp-v3b (instantiate float-vector 3))
	      (tmp-m33 (make-matrix 3 3))
	      debug-view
	      &allow-other-keys)
   (let* ((len (length link-list))
	  pos-i pos-r
	  paxis
	  child-link child-reverse
	  j)
     (unless fik (setq fik (make-matrix dim fik-len)))
     (do ((i 0 (1+ i))
	  (l 0 (1+ l)))
	 ((>= l len))
       (setq j (send (elt link-list l) :joint))
       (cond ((not (derivedp (send j :child-link) bodyset-link))
	      (setq child-reverse nil)
	      (setq child-link (send j :child-link)))
             ((and (< (+ l 1) len)
		   (not (eq (send j :child-link)
			    (send (elt link-list (+ l 1)) :parent-link))))
	      (setq child-reverse t)
	      (setq child-link (send j :parent-link)))
	     (t
	      (setq child-reverse nil)
	      (setq child-link (send j :child-link))))
	(setq paxis (case (j . axis)
		      (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
		      (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
		      (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
		      (t (j . axis))))

	(cond 
	 ((derivedp j linear-joint)
	  (send self :calc-jacobian-linear-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
	 ((derivedp j rotational-joint)
	  (send self :calc-jacobian-rotational-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
	 ((derivedp j wheel-joint)
	  (send self :calc-jacobian-wheel-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
	  (incf i 1))
	 ((derivedp j omniwheel-joint)
	  (send self :calc-jacobian-omniwheel-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
	  (incf i 2))
	 ((derivedp j sphere-joint)
	  (send self :calc-jacobian-sphere-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
	  (incf i 2))
	 ((derivedp j 6dof-joint)
	  (send self :calc-jacobian-6dof-joint
		fik i j paxis child-link child-reverse move-target
		rotation-axis translation-axis pos-i pos-r
		tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
	  (incf i 5))
	 (t
	  (warn "unknown joint type ~A~%" j)))
	;;
	)
     fik))
  ;;
  (:calc-joint-angle-speed
   (vel-pos vel-rot
    &rest args
    &key
    link-list
    move-target jacobi jacobi# null-space
    (rotation-axis nil) (translation-axis t)
    p-limit r-limit
    (weight (fill (instantiate float-vector (send self :calc-target-joint-dimension link-list)) 1))
    (wmat)
    (debug-view nil)
    (tmp-pos (float-vector 0 0 0))
    (tmp-rot (float-vector 0 0 0))
    (tmp-v0 (instantiate float-vector 0))
    (tmp-v1 (instantiate float-vector 1))
    (tmp-v2 (instantiate float-vector 2))
    (tmp-v3 (instantiate float-vector 3))
    (tmp-v3a (instantiate float-vector 3))
    (tmp-v3b (instantiate float-vector 3))
    (tmp-v3c (instantiate float-vector 3))
    (tmp-m33 (make-matrix 3 3))
    (tmp-dim)
    (tmp-len) (tmp-len2)
    (tmp-mcc) (tmp-mcc2)
    &allow-other-keys)
   (let (vel-p vel-r J#x I-J#J (len (send self :calc-target-joint-dimension link-list)))
     (when (or (null link-list) (null move-target))
       (warn ";; ERROR: :link-list or :move-target required~%")
       (return-from :calc-joint-angle-speed null-space))
     (if (and (null translation-axis) (null rotation-axis))
	 (return-from :calc-joint-angle-speed null-space))

     ;; limit
     (unless p-limit (setq p-limit 100.0))
     (unless r-limit (setq r-limit 0.5))
     (if (> (norm vel-pos) p-limit)
	 (setq vel-pos (scale p-limit (normalize-vector vel-pos tmp-pos) tmp-pos)))
     (if (> (norm vel-rot) r-limit)
	 (setq vel-rot (scale r-limit (normalize-vector vel-rot tmp-rot) tmp-rot)))

     (setq vel-pos (scale 0.001 vel-pos tmp-pos)) ;; scale [mm] -> [m]

     ;; dav = J#x + (I - J#J)y
     ;; particular : J#x
     (if (null jacobi)
	 (setq jacobi (send* self :calc-jacobian-from-link-list link-list args)))
     (if (null jacobi#)
	 (setq jacobi# (send* self :calc-inverse-jacobian jacobi args)))

     (if (null tmp-dim)
	 (setq tmp-dim (instantiate float-vector (send self :calc-target-axis-dimension rotation-axis translation-axis))))

     (setq vel-p (calc-dif-with-axis
		  (v- (send move-target :inverse-transform-vector vel-pos
			    tmp-v3 tmp-v3b tmp-m33)
		      (send move-target :inverse-transform-vector #f(0 0 0)
			    tmp-v3a tmp-v3b tmp-m33) tmp-v3c)
		  translation-axis
		  tmp-v0 tmp-v1 tmp-v2))
     (dotimes (i (length vel-p)) (setf (elt tmp-dim i) (elt vel-p i)))

     (setq vel-r (calc-dif-with-axis
		  (v- (send move-target :inverse-transform-vector vel-rot 
			    tmp-v3 tmp-v3b tmp-m33)
		      (send move-target :inverse-transform-vector #f(0 0 0)
			    tmp-v3a tmp-v3b tmp-m33) tmp-v3)
		  rotation-axis
		  tmp-v0 tmp-v1 tmp-v2))
     (dotimes (i (length vel-r)) (setf (elt tmp-dim (+ i (length vel-p))) (elt vel-r i)))
     
     (if (null tmp-len) (setq tmp-len (instantiate float-vector len)))
     (if (null tmp-len2) (setq tmp-len2 (instantiate float-vector len)))
     (setq J#x (transform jacobi# tmp-dim tmp-len))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "J#t   :")
       (dotimes (j (cadr (array-dimensions jacobi#)))
	 (if (/= j 0) (warn "       "))
	 (dotimes (i len) (warn "~7,3f " (aref jacobi# i j))) (warn "~%"))
       (warn "J#x   :")
       (dotimes (i len) (warn "~7,3f " (elt J#x i))) (warn "~%"))

     ;; if use null space
     (when (and (vectorp null-space)
		(= len (length null-space)))
       (if (null tmp-mcc) (setq tmp-mcc (make-matrix len len)))
       (if (null tmp-mcc2) (setq tmp-mcc2 (make-matrix len len)))
       ;; projection : I - J#J
       ;; (unit-matrix (car (array-dimensions jacobi#))))
       (fill (array-entity tmp-mcc) 0)
       (dotimes (i len) (setf (aref tmp-mcc i i) 1.0))
       (setq I-J#J (m- tmp-mcc
		       (m* jacobi# jacobi tmp-mcc2) tmp-mcc))
       ;; add null space
       (if (null wmat) (setq wmat (make-matrix len len )))
       (dotimes (i len) (setf (aref wmat i i) (aref weight i)))
       (v+ J#x (transform wmat (transform I-J#J null-space tmp-len2) tmp-len2) J#x))
     J#x))
  (:collision-avoidance-links
   (&optional l)
   (if l (setq collision-avoidance-links l)) collision-avoidance-links)
  (:collision-avoidance-link-pair-from-link-list
    (link-list &key obstacles ((:collision-avoidance-links collision-links) collision-avoidance-links))
   (let* ((parent-links
           (mapcar
            #'(lambda (ll)
                (let ((l ll))
                  (while (and l (send l :parent-link)
                              (eps= (distance (send ll :worldpos)
                                              (send (send l :parent-link) :worldpos)) 0))
                    (setq l (send l :parent-link)))
                  (send l :parent-link)))
            collision-links))
          (valid-link-list
           (subseq
            link-list
            (let ((i 1))
              (while (eps= (distance (send (elt link-list 0) :worldpos)
                                     (send (elt link-list i) :worldpos)) 0.0)
                (incf i)) i)))
          (index-links
           (mapcar #'(lambda (l) (position l valid-link-list)) collision-links))
          (len (length collision-links))
          i0 ret)
     (dotimes (i len)
       (do ((j (1+ i) (1+ j)))
           ((>= j len))
           (if (and (or (setq i0 (elt index-links i))
                        (elt index-links j))
                    (not (eq (elt parent-links i)
                             (elt collision-links j)))
                    (not (eq (elt parent-links j)
                             (elt collision-links i))))
               (push (list (elt collision-links (if i0 i j))
                           (elt collision-links (if i0 j i))) ret))))
     (when obstacles
       (if (atom obstacles) (setq obstacles (list obstacles)))
       (dotimes (i len)
         (if (elt index-links i)
             (dolist (o obstacles)
               (push (list (elt collision-links i) o) ret)))))
     ret))
  (:collision-avoidance-calc-distance
   (&rest args
	  &key link-list (warnp t)
          ((:collision-avoidance-link-pair pair-list))
	  &allow-other-keys)
   (let* ((pair-len (length pair-list))
	  (ret (make-list pair-len))
	  (col-list (send self :get :collision-distance))
	  (distance-limit 10)
	  (sc 0.0) np pair)
     (dotimes (i pair-len) (setf (elt ret i) (float-vector 0 0 0)))
     (when (or (null link-list) (= pair-len 0))
       (warn ";; ERROR: :link-list ~A or pair-len ~A required~%" link-list pair-len)
       (return-from :collision-avoidance-calc-distance ret))
     (if (or (null col-list) (/= (length col-list) pair-len))
	 (setq col-list (make-list pair-len)))
     ;; pair (part of this limb . part of another limb)
     (dotimes (i pair-len)
       (setq pair (elt pair-list i))
       (if (not (memq (car pair) link-list)) 
	   (progn
             (warn ";; ERROR : (car pair) is not included in link-list ~A~%" (send-all pair :name))
	     (setf (elt col-list i) nil)
	     )
	 (progn
	   (setq np (nconc (pqp-collision-distance (car pair) (cadr pair) :qsize 2) (list i)))
	   (if (<= (car np) distance-limit)
	       (progn
		 (if (and warnp (< (car np) 1.0))
		     (warn ";; !!WARNING!! collision detected~%;; (~a . ~a . ~a)~%"
			   (send (car pair) :name) (send (cadr pair) :name) np))
                 (setf (elt np 1) (send (car pair) :centroid))
                 (setf (elt col-list i) np)
                 (setf (elt (elt col-list i) 0) distance-limit))
	     (progn
	       (setf (elt col-list i) np)))
	   (setf (elt ret i)
		 (scale 0.001 (normalize-vector (v- (elt np 1) (elt np 2) (elt ret i)) (elt ret i)) (elt ret i)))
	   ))) ;; dotimes
     (sort col-list #'<= #'(lambda (p) (car p)))
     (send self :put :collision-distance col-list)
     ret))
  (:collision-avoidance-args
   (pair link-list)
   (let* ((col-args-list (send self :get :collision-avoidance-args-list))
          (col-link-list (send self :link-list (car pair) (car link-list)))
          (col-link-len (send self :calc-target-joint-dimension link-list))
          col-args)
     (if (> col-link-len (length col-args-list))
         (setq col-args-list
               (nconc col-args-list 
                      (make-list (- col-link-len
                                    (length col-args-list))))))
     (setq col-args (elt col-args-list (1- col-link-len)))
     (unless col-args
       (let* ((c col-link-len)
            (r 3) ;; :rotation-axis nil :translation-axis t
            (ret (make-matrix c r))
            (wmat (make-matrix c c))
            (tmat (make-matrix c r))
            (umat (make-matrix r r))
            (umat2 (make-matrix r r))
            (mat-tmp (make-matrix c r))
            (tmp-mcc (make-matrix c c))
            (tmp-mcc2 (make-matrix c c))
            (tmp-mrc (make-matrix r c))
            (tmp-mrr (make-matrix r r))
            (tmp-mrr2 (make-matrix r r))
            (fik (make-matrix r c))
            (tmp-v0 (instantiate float-vector 0))
            (tmp-v1 (instantiate float-vector 1))
            (tmp-v2 (instantiate float-vector 2))
            (tmp-v3 (instantiate float-vector 3))
            (tmp-v3a (instantiate float-vector 3))
            (tmp-v3b (instantiate float-vector 3))
            (tmp-v3c (instantiate float-vector 3))
            (tmp-m33 (make-matrix 3 3))
            (tmp-dim (instantiate float-vector r))
            (tmp-len (instantiate float-vector c))
            (tmp-len2 (instantiate float-vector c))
            (tmp-pos (instantiate float-vector 3))
            (tmp-rot (instantiate float-vector 3))
            )
       (setq col-args
             (list :ret ret :wmat wmat :tmat tmat :umat umat :umat2 umat2
                   :mat-tmp mat-tmp :tmp-mcc tmp-mcc :tmp-mcc2 tmp-mcc2
                   :tmp-mrc tmp-mrc :tmp-mrr tmp-mrr :tmp-mrr2 tmp-mrr2
                   :fik fik :weight nil
                   :tmp-v0 tmp-v0 :tmp-v1 tmp-v1
                   :tmp-v2 tmp-v2 :tmp-v3 tmp-v3
                   :tmp-v3a tmp-v3a :tmp-v3b tmp-v3b :tmp-v3c tmp-v3c
                   :tmp-m33 tmp-m33
                   :tmp-dim tmp-dim :tmp-len tmp-len :tmp-len2 tmp-len2
                   :tmp-pos tmp-pos :tmp-rot tmp-rot
                   ))
       (setf (elt col-args-list (1- col-link-len)) col-args)
       (send self :put :collision-avoidance-args-list col-args-list)))
     col-args))
  (:collision-avoidance
   (vel-pos &rest args
            &key
	    avoid-collision-distance
	    avoid-collision-joint-gain
	    avoid-collision-null-gain
            ((:collision-avoidance-link-pair pair-list))
            (link-list)
            tmp-mcc tmp-mcc2
            &allow-other-keys)
   (let* ((len (send self :calc-target-joint-dimension link-list))
          (ret (send* self :collision-avoidance-calc-distance args))
          (pair-len (length pair-list))
          (np (elt (send self :get :collision-distance) 0))
          (min-distance (car np))
          (pair (elt pair-list (car (last np))))
	  (dav-col (instantiate float-vector len))
          drag-coords col-link-list jacobi col-jacobi jacobi# I-J#J ddav-col col-args
          )
     (if (>= min-distance avoid-collision-distance)
         (return-from :collision-avoidance dav-col))
     (setq col-args (send self :collision-avoidance-args pair link-list))

     (setq drag-coords (make-cascoords :pos (elt np 1)))
     (send (car pair) :assoc drag-coords)
     (setq col-link-list (send self :link-list (car pair) (car link-list)))
     (setq col-jacobi (send* self :calc-jacobian-from-link-list
                             col-link-list :move-target drag-coords
                             col-args))
     (send (car pair) :dissoc drag-coords)
     (setq ddav-col (transform (transpose col-jacobi) (elt ret 0)))

     (dotimes (i (length ddav-col)) (setf (elt dav-col i) (elt ddav-col i)))
     (setq jacobi (send* self :calc-jacobian-from-link-list link-list args))
     (setq jacobi# (send* self :calc-inverse-jacobian jacobi args))
     ;;;
     (unless tmp-mcc  (setq tmp-mcc (make-matrix len len)))
     (unless tmp-mcc2 (setq tmp-mcc2 (make-matrix len len)))
     (fill (array-entity tmp-mcc) 0)
     (dotimes (i len) (setf (aref tmp-mcc i i) 1.0))

     (setq I-J#J (m- tmp-mcc (m* jacobi# jacobi tmp-mcc2) tmp-mcc))
     (setq ddav-col (transform I-J#J dav-col))
     (dotimes (i len) (setf (elt dav-col i)
                            (+ (* avoid-collision-joint-gain (elt dav-col i))
                               (* avoid-collision-null-gain (elt ddav-col i)))))
     (setq dav-col (scale (- (* avoid-collision-distance (/ 1.0 min-distance)) 1) dav-col))
     dav-col))
  (:move-joints
   (vel-pos vel-rot &rest args
    &key
    link-list
    move-target
    angle-speed
    (angle-speed-blending 0.5)
    (debug-view nil)
    (fix-targets)
    &allow-other-keys)
   (let (dav dtheta j fix-target fix-targets-coords (ret t))
     (if fix-targets
         (setq fix-targets-coords (send-all fix-targets :copy-worldcoords)))
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (if (and debug-view (not (equal debug-view :no-clear)) (boundp '*viewer*))
	 (send *viewer* :viewsurface :clear))

     (setq dav (send* self :calc-joint-angle-speed vel-pos vel-rot args))
     (if angle-speed (midpoint angle-speed-blending dav angle-speed dav))

     ;; update body
     (do ((i 0 (1+ i))
	  (l 0 (1+ l)))
	 ((>= l (length link-list)))
       (setq j (send (elt link-list l) :joint))
       (setq dtheta
	 (cond ((derivedp j linear-joint)
		(* 1000.0 (elt dav i)))
	       ((derivedp j rotational-joint)
		(rad2deg (elt dav i)))
	       ((derivedp j wheel-joint)
		(float-vector (* 1000 (elt dav i))
			      (rad2deg (elt dav (incf i)))))
	       ((derivedp j omniwheel-joint)
		(float-vector (* 1000 (elt dav i))
			      (* 1000 (elt dav (incf i)))
			      (rad2deg (elt dav (incf i)))))
	       ((derivedp j sphere-joint)
		(float-vector (rad2deg (elt dav i))
			      (rad2deg (elt dav (incf i)))
			      (rad2deg (elt dav (incf i)))))
	       ((derivedp j 6dof-joint)
		(float-vector (* 1000 (elt dav i))
			      (* 1000 (elt dav (incf i)))
                              (* 1000 (elt dav (incf i)))
			      (rad2deg (elt dav (incf i)))
                              (rad2deg (elt dav (incf i)))
                              (rad2deg (elt dav (incf i)))))
               ))
       (send (elt link-list l) :joint :joint-angle dtheta :relative t))
     (when fix-targets
       (let ((tmp-coords (pop fix-targets-coords)))
         (send tmp-coords :transform
               (send (pop fix-targets) :transformation self))
         (send self :newcoords tmp-coords)
         (while (setq fix-target (pop fix-targets))
           (unless
               (send self :inverse-kinematics (pop fix-targets-coords)
                     :move-target fix-target
                     :link-list (send self :link-list fix-target)
                     :avoid-collision-null-gain 0
                     :avoid-collision-joint-gain 0)
             (setq ret nil)))))
     ;;
     ;; done
     (when (and debug-view *viewer*)
       (send *viewer* :draw-objects :clear nil :flush nil)
       (if move-target
	   (send-message move-target cascaded-link :draw-on :flush nil :size 100)))
     ret))
  (:move-joints-avoidance
   (vel-pos vel-rot &rest args
	    &key
	    link-list
	    move-target
	    (weight (fill (instantiate float-vector (send self :calc-target-joint-dimension link-list)) 1))
	    (null-space)
	    (avoid-nspace-gain 0.01)
	    (avoid-weight-gain 1.0)
	    (avoid-collision-distance 200)
	    (avoid-collision-null-gain 1.0)
	    (avoid-collision-joint-gain 1.0)
            ((:collision-avoidance-link-pair pair-list)
             (send self :collision-avoidance-link-pair-from-link-list link-list))
	    (tmp-len) (tmp-len2) (debug-view)
	    &allow-other-keys)
   (let* ((len (send self :calc-target-joint-dimension link-list))
	  (wmat (make-matrix len len))
	  joint-angle-limit-nspace
	  joint-angle-limit-weight
	  joint-angle-limit-weight-old
	  dav dtheta 
	  (null-space-joint-limit (instantiate float-vector len))
          null-space-collision-avoidance
          angle-speed-collision-avoidance
          (angle-speed-collision-blending 0.0) min-distance
          (ret t))
     (when (or (null link-list) (null move-target))
       (warn ";; ERROR: :link-list or :move-target required~%")
       (return-from :move-joints-avoidance t))
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (unless (get self :joint-angle-limit-weight-old)
       (setf (get self :joint-angle-limit-weight-old) (make-hash-table)))
     (setq joint-angle-limit-weight-old
       (gethash link-list (get self :joint-angle-limit-weight-old)))
     (unless joint-angle-limit-weight-old
       (setq joint-angle-limit-weight-old (instantiate float-vector len))
       (fill joint-angle-limit-weight-old 1.0e+20)
       (setf (gethash link-list (get self :joint-angle-limit-weight-old))
	     joint-angle-limit-weight-old))
     (if (null tmp-len) (setq tmp-len (instantiate float-vector len)))
     (if (null tmp-len2) (setq tmp-len2 (instantiate float-vector len)))

     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "angle :")
       (dolist (j (send-all link-list :joint))
	 (let ((a (send j :joint-angle)))
	   (if (vectorp a)
	       (dotimes (i (length a)) (warn "~7,1f " (elt a i)))
	     (warn "~7,1f " a))))
	   (warn "~%"))
     ;;
     ;; wmat/weight: weighting joint angle weight 
     ;; 
     ;; w_i = 1 + | dH/dt |      if d|dH/dt| >= 0
     ;;     = 1                  if d|dH/dt| <  0
     ;; dH/dt = (t_max - t_min)^2 (2t - t_max - t_min)
     ;;         / 4 (t_max - t)^2 (t - t_min)^2
     ;;
     ;; T. F. Chang and R.-V. Dubey: "A weighted least-norm solution based
     ;; scheme for avoiding joint limits for redundant manipulators", in IEEE
     ;; Trans. On Robotics and Automation, 11((2):286-292, April 1995.
     ;;
     (when (> avoid-weight-gain 0.0)
       (setq joint-angle-limit-weight 
	 (scale avoid-weight-gain
		(joint-angle-limit-weight (send-all link-list :joint) tmp-len) tmp-len))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn "cost  :")
	 (dotimes (i len) (warn "~7,3f " (elt joint-angle-limit-weight i))) (warn "~%"))
       (dotimes (i len)
	 (setf (elt weight i)
	       (* (elt weight i) 
		  (if (> (- (elt joint-angle-limit-weight i)
                            (elt joint-angle-limit-weight-old i)) 0.0)
		      (/ 1.0 (+ 1.0 (elt joint-angle-limit-weight i)))
		    1.0))))
       (dotimes (i len) (setf (elt joint-angle-limit-weight-old i)
			      (elt joint-angle-limit-weight i))))
     (dotimes (i len) (setf (aref wmat i i) (aref weight i)))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "weight:")
       (dotimes (i  len) (warn "~7,3f " (elt weight i))) (warn "~%"))
     ;;
     ;; avoid-nspace-joint-limit: avoiding joint angle limit
     ;; 
     ;; dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
     ;;
     (when (> avoid-nspace-gain 0.0)
       (setq joint-angle-limit-nspace (joint-angle-limit-nspace (send-all link-list :joint) tmp-len2))
       (setq null-space-joint-limit
	 (scale avoid-nspace-gain
                joint-angle-limit-nspace tmp-len2))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn "nspace:")
	 (dotimes (i  len) (warn "~7,3f " (elt null-space-joint-limit i))) (warn "~%")))
     ;;
     ;; angle-speed-collision-avoidance: avoiding self collision
     ;;
     ;; qca = J#t a dx + ( I - J# J ) Jt b dx
     ;;
     ;; dx = p                     if |p| > d_yz
     ;;    = (dyz / |p|  - 1) p    else
     ;;
     ;; a : avoid-collision-joint-gain
     ;; b : avoid-collision-null-gain
     ;;
     ;; implimentation issue: 
     ;; when link and object are collide, 
     ;;  p = (nearestpoint_on_object_surface - center_of_link )
     ;; else
     ;;  p =  (nearestpoint_on_object_surface - neareset_point_on_link_surface)
     ;; 
     ;; H. Sugiura, M. Gienger, H. Janssen, C. Goerick: "Real-Time Self
     ;; Collision Avoidance for Humanoids by means of Nullspace Criteria
     ;; and Task Intervals" In Humanoids 2006.
     ;;
     ;; H. Sugiura, M. Gienger, H. Janssen and C. Goerick : "Real-Time
     ;; Collision Avoidance with Whole Body Motion Control for Humanoid
     ;; Robots", In IROS 2007, 2053--2058
     ;;
     (when (or (> avoid-collision-joint-gain 0.0)
               (> avoid-collision-null-gain 0.0))
       (setq angle-speed-collision-avoidance
             (send* self :collision-avoidance vel-pos
                    :avoid-collision-distance avoid-collision-distance
                    :avoid-collision-null-gain avoid-collision-null-gain
                    :avoid-collision-joint-gain avoid-collision-joint-gain
                    :collision-avoidance-link-pair pair-list args))
       (setq min-distance (car (elt (send self :get :collision-distance) 0)))
       (setq angle-speed-collision-blending
           (cond ((< min-distance (* 0.1 avoid-collision-distance))
                  1.0)
                 ((< min-distance avoid-collision-distance)
                  (/ (- avoid-collision-distance min-distance) 
                     (* 0.9 avoid-collision-distance)))
                 (t
                  0.0)))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn "colvel:")
	 (dotimes (i len) (warn "~7,3f " (elt angle-speed-collision-avoidance i))) (warn "~%")))
     (if (listp null-space) (setq null-space (eval null-space)))
     (if null-space
         (setq null-space-joint-limit
               (v+ null-space
                   null-space-joint-limit
                   null-space-joint-limit)))
     ;;
     ;; q = f(d) qca + {1 - f(d)} qwbm
     ;;
     ;; f(d) = (d - da) / (db - da), if d < da
     ;;      = 0                   , otherwise
     ;; da : avoid-collision-distance
     ;; db : avoid-collision-distance*0.1
     ;;
     ;; H. Sugiura, IROS 2007
     ;;
     ;; qwbm = J# x + N W y
     ;; 
     ;; J# = W Jt(J W Jt + kI)-1 (Weighted SR-Inverse)
     ;; N  = E - J#J
     ;;
     ;; SR-inverse : 
     ;; Y. Nakamura and H. Hanafusa : "Inverse Kinematic Solutions With
     ;; Singularity Robustness for Robot Manipulator Control"
     ;; J. Dyn. Sys., Meas., Control  1986. vol 108, Issue 3, pp. 163--172.
     ;; 
     (setq ret
         (send-message* self cascaded-link
                        :move-joints vel-pos vel-rot
                        :null-space null-space-joint-limit
                        :angle-speed angle-speed-collision-avoidance
                        :angle-speed-blending angle-speed-collision-blending
                        :weight weight
                        args))
     ;;
     ;; done
     (when (and debug-view *viewer*)
       (if (or (> avoid-collision-null-gain 0.0)
	       (> avoid-collision-joint-gain 0.0))
	 (let ((pwidth (send *viewer* :viewsurface :line-width))
	       (psize (send *viewer* :viewsurface :point-size))
	       (pcolor (send *viewer* :viewsurface :color))
	       np)
	   (send *viewer* :viewsurface :color #f(1 0 0))
	   (dotimes (i (length pair-list))
	     (setq np (elt (send self :get :collision-distance) i))
	     (unless np
               (warn ";; ERROR : could not get collision-distance ~A" (send-all pair-list :name))
               (return))
	     (when (<= (car np) 1.0)
               (send *viewer* :viewsurface :line-width 1)
               (dolist (obj (elt pair-list (car (last np))))
                 (when (and (derivedp obj bodyset-link)
                            (eq (send obj :analysis-level) :coords))
                   (send-all (obj . geo::bodies) :worldcoords))
                 (dolist (e (cdr (send obj :edges)))
                   (send *viewer* :viewsurface :3d-line
                         (e . pvert) (e . nvert) :depth-test t))))
             (if (< (car np) avoid-collision-distance)
                 (progn (send *viewer* :viewsurface :line-width 4)
                        (send *viewer* :viewsurface :point-size 6))
               (progn
                 (send *viewer* :viewsurface :line-width 1)
                 (send *viewer* :viewsurface :point-size 3)))
	     (send *viewer* :viewsurface :3d-line (elt np 1)(elt np 2) :depth-test t)
	     (send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
	     (send *viewer* :viewsurface :3d-point (elt np 2) :depth-test t)
             (send *viewer* :viewsurface :color #f(1.0 0.0 1.0)))
	   (send *viewer* :viewsurface :line-width pwidth)
	   (send *viewer* :viewsurface :point-size psize)
	   (send *viewer* :viewsurface :color pcolor)
	   ))
       (if (not (memq :no-flush debug-view))
	   (send *viewer* :viewsurface :flush))
       )
     ret))
  (:inverse-kinematics
   (target-coords &rest args 
		  &key (stop 50)
		  (link-list)
		  (move-target)
		  (debug-view) (warnp t)
		  (rotation-axis t)
		  (translation-axis t)
		  (look-at-target)
                  (fix-targets)
		  (thre 1) (rthre (deg2rad 1))
		  &allow-other-keys)
   (let* ((loop 0)
          (av0 (send-all (send-all link-list :joint) :joint-angle))
          (c0 (unless (send self :parent) (send self :copy-worldcoords)))
	  dif-pos dif-rot need-clear need-flush
	  ;;
	  (c (send self :calc-target-joint-dimension link-list))
	  (r (send self :calc-target-axis-dimension rotation-axis translation-axis))
	  (a (max c r))
	  (b (min c r))
	  (ret (make-matrix c r))
	  (wmat (make-matrix c c))
	  (tmat (make-matrix c r))
	  (umat (make-matrix r r))
	  (umat2 (make-matrix r r))
	  (mat-tmp (make-matrix c r))
	  (tmp-mcc (make-matrix c c))
	  (tmp-mcc2 (make-matrix c c))
	  (tmp-mrc (make-matrix r c))
	  (tmp-mrr (make-matrix r r))
	  (fik (make-matrix r c))
	  (tmp-v0 (instantiate float-vector 0))
	  (tmp-v1 (instantiate float-vector 1))
	  (tmp-v2 (instantiate float-vector 2))
	  (tmp-v3 (instantiate float-vector 3))
	  (tmp-v3a (instantiate float-vector 3))
	  (tmp-v3b (instantiate float-vector 3))
	  (tmp-v3c (instantiate float-vector 3))
	  (tmp-m33 (make-matrix 3 3))
	  (tmp-dim (instantiate float-vector r))
	  (tmp-len (instantiate float-vector c))
	  (tmp-len2 (instantiate float-vector c))
	  (tmp-pos (instantiate float-vector 3))
	  (tmp-rot (instantiate float-vector 3))
	  ndif-pos ndif-rot
          (success t) 
          (fix-targets-av 
           (mapcar #'(lambda (l) (send-all l :joint :joint-angle))
                   (mapcar #'(lambda (l) (send self :link-list l)) fix-targets)))
	  (old-analysis-level (send-all link-list :analysis-level))
	  )
     (send-all link-list :analysis-level :coords)
     (when debug-view
       (if (atom debug-view) (setq debug-view (list debug-view)))
       (when (not (memq :no-clear debug-view))
	 (setq need-clear t)
	 (push :no-clear debug-view))
       (when (not (memq :no-flush debug-view))
	 (setq need-flush t)
	 (push :no-flush debug-view)))
     ;; inverse kinematics loop
     (while (< (incf loop) stop)
       (setq dif-pos (send move-target :difference-position target-coords)
	     dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
       (setq ndif-pos (calc-dif-with-axis
		       (v- (send move-target :inverse-transform-vector dif-pos)
			   (send move-target :inverse-transform-vector (float-vector 0 0 0)))
		       translation-axis))
       (setq ndif-rot (calc-dif-with-axis
		       (v- (send move-target :inverse-transform-vector dif-rot)
			   (send move-target :inverse-transform-vector (float-vector 0 0 0)))
		       rotation-axis))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn ";; ~3d: dif-pos : ~a/(~a/~A), dif-rot : ~a/(~a/~A)~%"
	       loop ndif-pos (norm ndif-pos) thre ndif-rot (norm ndif-rot) rthre))
       ;; check loop end
       (if (and (> loop (/ stop 10))
		(< (norm ndif-pos) thre)
		(if rotation-axis (< (norm ndif-rot) rthre) t))
	   (return nil))
       ;; ik
       (when (and need-clear debug-view *viewer*)
	 (if need-clear (send *viewer* :viewsurface :clear)))
       (setq
        success
        (send-message* self cascaded-link
                       :move-joints-avoidance
                       dif-pos dif-rot :link-list link-list
                       :rotation-axis rotation-axis :translation-axis translation-axis
                       :warnp warnp :debug-view debug-view
                       :fix-targets fix-targets
                       ;; buffer for calculation
                       :dim r
                       :fik fik
                       :ret ret
                       :wmat wmat :tmat tmat :umat umat :umat2 umat2 :mat-tmp mat-tmp
                       :tmp-mcc tmp-mcc :tmp-mcc2 tmp-mcc2 :tmp-mrr tmp-mrr
                       :tmp-mrc tmp-mrc
                       :tmp-v0 tmp-v0 :tmp-v1 tmp-v1
                       :tmp-v2 tmp-v2 :tmp-v3 tmp-v3
                       :tmp-v3a tmp-v3a :tmp-v3b tmp-v3b  :tmp-v3c tmp-v3c
                       :tmp-m33 tmp-m33 
                       :tmp-dim tmp-dim
                       :tmp-len tmp-len :tmp-len2 tmp-len2
                       :tmp-pos tmp-pos :tmp-rot tmp-rot
                       args))
       (when (and debug-view  *viewer*)
	 (send target-coords :draw-on :flush nil :color #f(1 0 0))
	 (if need-flush (send *viewer* :viewsurface :flush)))
       (unless success (return nil))
       (if look-at-target
	   (send self :head :look-at (send target-coords :worldpos)))
       )
     ;; update difference
     (setq dif-pos (send move-target :difference-position target-coords)
	   dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
     (setq dif-pos (calc-dif-with-axis
		    (v- (send move-target :inverse-transform-vector dif-pos)
			(send move-target :inverse-transform-vector (float-vector 0 0 0)))
		    translation-axis))
     (setq dif-rot (calc-dif-with-axis
		    (v- (send move-target :inverse-transform-vector dif-rot)
			(send move-target :inverse-transform-vector (float-vector 0 0 0)))
		    rotation-axis))

     (mapcar #'(lambda (l a) (send l :analysis-level a)) link-list old-analysis-level)
     ;; check solved or not
     (if (and success (< (norm dif-pos) thre)  (< (norm dif-rot) rthre))
         (send self :angle-vector)
       (progn
	 (when warnp
	   (warn ";; inverse-kinematics failed.~%")
	   (warn ";; dif-pos : ~a/(~a/~a)~%" dif-pos (norm dif-pos) thre)
	   (warn ";; dif-rot : ~a/(~a/~a)~%" dif-rot (norm dif-rot) rthre)
	   (warn ";;  coords : ~a~%" 
		 (send (let ((p self)) (while (send p :parent) (setq p (send p :parent))) p) :worldcoords))
	   (warn ";;  angles : ~a~%" av0)
	   (warn ";;    args : ~a~%" (append (list target-coords) args))
	   )
	 (mapc #'(lambda (j a) (send j :joint-angle a)) (send-all link-list :joint) av0)
         (if fix-targets-av
             (mapcar #'(lambda (ll aa) (mapc #'(lambda (j a) (send j :joint-angle a)) (send-all ll :joint) aa))
                     (mapcar #'(lambda (l) (send self :link-list l)) fix-targets)
                     fix-targets-av))
         (if c0 (send self :newcoords c0))
	 nil))
     ))
  )

(defun calc-dif-with-axis (dif axis &optional tmp-v0 tmp-v1 tmp-v2)
  (case axis
	((:x :xx)
	 (if tmp-v2 
	     (progn
	       (setf (elt tmp-v2 0) (elt dif 1) (elt tmp-v2 1) (elt dif 2))
	       tmp-v2)
	   (float-vector (elt dif 1) (elt dif 2))))
	((:y :yy)
	 (if tmp-v2
	     (progn
	       (setf (elt tmp-v2 0) (elt dif 0) (elt tmp-v2 1) (elt dif 2))
	       tmp-v2)
	   (float-vector (elt dif 0) (elt dif 2))))
	((:z :zz)
	 (if tmp-v2
	     (progn
	       (setf (elt tmp-v2 0) (elt dif 0) (elt tmp-v2 1) (elt dif 1))
	       tmp-v2)
	   (float-vector (elt dif 0) (elt dif 1))))
	((:xy :yx)
	 (if tmp-v1
	     (progn 
	       (setf (elt tmp-v1 0) (elt dif 2))
	       tmp-v1)
	   (float-vector (elt dif 2))))
	((:yz :zy)
	 (if tmp-v1
	     (progn 
	       (setf (elt tmp-v1 0) (elt dif 0))
	       tmp-v1)
	   (float-vector (elt dif 0))))
	((:zx :xz)
	 (if tmp-v1
	     (progn 
	       (setf (elt tmp-v1 0) (elt dif 1))
	       tmp-v1)
	   (float-vector (elt dif 1))))
	(nil (if tmp-v0 tmp-v0 (float-vector)))
	(t dif)))

(defun calc-target-joint-dimension (joint-list)
   (let ((n 0))
     (dolist (j joint-list)
       (cond 
	((derivedp j wheel-joint)
	 (incf n 2))
	((derivedp j omniwheel-joint)
	 (incf n 3))
	((derivedp j sphere-joint)
	 (incf n 3))
	((derivedp j 6dof-joint)
	 (incf n 6))
	(t (incf n))))
     n))

(defun joint-angle-limit-weight (j-l &optional (res (instantiate float-vector (calc-target-joint-dimension j-l))))
  (let (j jang jmax jmin nang wang (e (deg2rad 1)) (k 0) (kk 0) )
    (dotimes (i (calc-target-joint-dimension j-l))
      (setq j (elt j-l k))
      (cond
       ((vectorp (send j :joint-angle))
        (setq jang (elt (send j :joint-angle) kk)
              jmax (elt (send j :max-angle) kk)
              jmin (elt (send j :min-angle) kk))
        (if (vectorp (elt (j . axis) kk))
            (setq jang (* 0.001 jang)
                  jmax (* 0.001 jmax)
                  jmin (* 0.001 jmin))
          (setq jang (deg2rad jang)
                jmax (deg2rad jmax)
                jmin (deg2rad jmin)))
        (when (>= (incf kk) (length (send j :joint-angle))) (setq kk 0) (incf k))
        )
       (t
        (setq jang (send j :joint-angle)
              jmax (send j :max-angle)
              jmin (send j :min-angle))
        (if (vectorp (j . axis))
            (setq jang (* 0.001 jang)
                  jmax (* 0.001 jmax)
                  jmin (* 0.001 jmin))
          (setq jang (deg2rad jang)
                jmax (deg2rad jmax)
                jmin (deg2rad jmin)))
        (incf k)))
      (cond
       ((and (eps= jang jmax e) (eps= jang jmin e)))
       ((eps= jang jmax e)
	(setq jang (- jmax e)))
       ((eps= jang jmin e)
	(setq jang (+ jmin e))))
      (cond
       ((or (eq (send j :name) :torso-waist-p)
	    (eq (send j :name) :waist-p))
	(setq nang 0.0)
	(cond
	 ((= nang jmax) (setq nang (- jmax (deg2rad 1))))
	 ((= nang jmin) (setq nang (+ jmin (deg2rad 1)))))
	(if (> jang nang) (setq wang (abs jmax)) (setq wang (abs jmin)))
	(setf (elt res i)
	      (abs (/ (* (expt (* 2 wang) 2) (* 2 (- jang nang)))
		      (* 4 (expt (- (* wang wang) (* jang jang)) 2))))))
       ((and (eps= jang jmax e) (eps= jang jmin e))
        (setf (elt res i) *inf*))
       (t
	(setq r
	      (abs (/ (* (expt (- jmax jmin) 2) (- (* 2 jang) jmax jmin))
		      (* 4 (expt (- jmax jang) 2) (expt (- jang jmin) 2)))))
	(if (and (< r 0.0) (> r 0.0)) (setq r 0.0))
	(setf (elt res i) r))
       )
      )
    res))

(defun joint-angle-limit-nspace (j-l &optional (res (instantiate float-vector (calc-target-joint-dimension j-l))))
  (let (j jang jmax jmin nang (k 0) (kk 0) r)
    (dotimes (i (calc-target-joint-dimension j-l))
      (setq j (elt j-l k))
      (cond
       ((vectorp (send j :joint-angle))
        (setq jang (elt (send j :joint-angle) kk)
              jmax (elt (send j :max-angle) kk)
              jmin (elt (send j :min-angle) kk))
        (if (vectorp (elt (j . axis) kk))
            (setq jang (* 0.001 jang)
                  jmax (* 0.001 jmax)
                  jmin (* 0.001 jmin))
          (setq jang (deg2rad jang)
                jmax (deg2rad jmax)
                jmin (deg2rad jmin)))
        (when (>= (incf kk) (length (send j :joint-angle))) (setq kk 0) (incf k))
        )
       (t
        (setq jang (send j :joint-angle)
              jmax (send j :max-angle)
              jmin (send j :min-angle))
        (if (vectorp (j . axis))
            (setq jang (* 0.001 jang)
                  jmax (* 0.001 jmax)
                  jmin (* 0.001 jmin))
          (setq jang (deg2rad jang)
                jmax (deg2rad jmax)
                jmin (deg2rad jmin)))
        (incf k)))
      (cond
       ((or (eq (send j :name) :torso-waist-p)
	    (eq (send j :name) :waist-p))
	(setq nang (- jmin (j . min-angle)))
	(cond
	 ((= nang jmax) (setq nang (- jmax (deg2rad 1))))
	 ((= nang jmin) (setq nang (+ jmin (deg2rad 1)))))
	(setf (elt res i)
	      (if (> jang nang)
		  (/ (- nang jang) (- jmax nang))
		(/ (- nang jang) (* 5.0 (- nang jmin))))))
       (t
	(setf (elt res i) (/ (- (/ (+ jmax jmin) 2.0) jang)
			     (/ (- jmax jmin) 2.0)))))
      (setq r (* (if (plusp (elt res i)) 1 -1) (expt (elt res i) 2)))
      (if (or (eq r *nan*) (eq r *inf*) (eq r *-inf*)) (setq r 0.0))
      (setf (elt res i) r))
    res))

(in-package "GEOMETRY")

(provide :irtmodel "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.60  2009-09-01 12:18:52  k-okada
;;; update ik methods, add references
;;;
;;; Revision 1.59  2009/09/01 12:16:52  k-okada
;;; fix joint-angle-limit-nspace/joint-angle-limit-weight
;;;
;;; Revision 1.58  2009/09/01 12:14:49  k-okada
;;; fix:add del-joint
;;;
;;; Revision 1.57  2009/09/01 12:14:15  k-okada
;;; add del-joint
;;;
;;; Revision 1.56  2009/09/01 12:13:38  k-okada
;;; update axis of wheel/shere/6dof
;;;
;;; Revision 1.55  2009/08/26 11:27:25  k-okada
;;; fix :collision-avoidance, use nearest joints for move-joint-avoidance
;;;
;;; Revision 1.54  2009/08/25 01:51:01  k-okada
;;; mat-tmp-cc, mat-tmp-cr nolonger used in pseudo-inverse
;;;
;;; Revision 1.53  2009/08/07 12:13:20  k-okada
;;; support rotation-axis :xx :yy :zz
;;;
;;; Revision 1.52  2009/08/07 11:19:10  k-okada
;;; set analysis-level in inverse-kinematics
;;;
;;; Revision 1.51  2009/08/04 08:35:28  ueda
;;; fix a typo
;;;
;;; Revision 1.50  2009/08/04 04:53:34  eus
;;; modify :move-joints-avoidance
;;;
;;; Revision 1.49  2009/07/31 11:10:02  ueda
;;; chage debug-view in move-joints for (pickview :separate t)
;;;
;;; Revision 1.48  2009/07/31 10:08:08  ueda
;;; support :obstacle and :collision-avoidance keyword for :collision-avoidance-link-pair-from-link-list
;;;
;;; Revision 1.47  2009/07/29 08:33:53  k-okada
;;; remove collision-avoidance-link-pair, add :collision-aovidance-link-pair-from-link-list
;;;
;;; Revision 1.46  2009/07/29 06:25:56  k-okada
;;; fix typo
;;;
;;; Revision 1.45  2009/07/29 06:21:46  k-okada
;;; rearrange, la, nul-col, dav-col to null-space-joint-limit, null-space-collision-avoidance, angle-speed-collision-avoidance
;;;
;;; Revision 1.44  2009/07/29 06:10:36  k-okada
;;; re-arraange collision-avoidance related codes :collision-avoidance, collisoin-avoidance-args :collision-avoidance-calc-distance
;;;
;;; Revision 1.43  2009/07/29 04:15:16  k-okada
;;; support :null-space '(....)
;;;
;;; Revision 1.42  2009/07/13 11:41:31  k-okada
;;; fix sphere/6dof-joint to zyx-angle
;;;
;;; Revision 1.41  2009/07/09 07:11:48  k-okada
;;; change 6d-joint -> 6dof->joint
;;;
;;; Revision 1.40  2009/07/09 02:41:45  k-okada
;;; fix joint-angle-limit-nspace not to return inf
;;;
;;; Revision 1.39  2009/07/09 01:18:25  k-okada
;;; support  :no-message in debug-view
;;;
;;; Revision 1.38  2009/07/07 07:23:20  k-okada
;;; support :no-message mode
;;;
;;; Revision 1.37  2009/07/03 11:00:48  k-okada
;;; set weight=0 when min-angle == max-angle, with avoid-weight-gain
;;;
;;; Revision 1.36  2009/07/03 10:40:22  k-okada
;;; add sphere and 6d joint
;;;
;;; Revision 1.35  2009/07/03 09:48:54  k-okada
;;; support avoid-weight-gain and avoid-null-gain for omniwheel/wheel-joints
;;;
;;; Revision 1.34  2009/07/03 06:16:47  k-okada
;;; set weight (wmax) even if  avoid-weight-gain == 0
;;;
;;; Revision 1.33  2009/07/02 15:28:29  k-okada
;;; restore when fillbody ik failed
;;;
;;; Revision 1.32  2009/07/02 14:43:31  k-okada
;;; support fullbody ik
;;;
;;; Revision 1.31  2009/06/30 11:08:42  k-okada
;;; change slot variable name from centroid to acentroid
;;;
;;; Revision 1.30  2009/06/30 08:35:14  k-okada
;;; add centroid
;;;
;;; Revision 1.29  2009/06/30 01:00:31  k-okada
;;; add :weight and :intertia-tensor
;;;
;;; Revision 1.28  2009/06/23 13:27:10  ueda
;;; #f() -> (float-vector) in wheel-joint, because it invekes SEGV
;;;
;;; Revision 1.27  2009/06/11 08:42:21  nagahama
;;; fixed typos in move-joints-avoidance
;;;
;;; Revision 1.26  2009/06/01 12:42:41  k-okada
;;; add wheel/omni-joint
;;;
;;; Revision 1.25  2009/05/29 06:23:56  nozawa
;;; fix typo
;;;
;;; Revision 1.24  2009/04/23 02:12:39  k-okada
;;; add default-coords to slot of joint class
;;;
;;; Revision 1.23  2009/04/09 08:52:17  k-okada
;;; add default-coords to bodyset-link, use default-coords in :joint-angle of linear-joint and rotational-joint
;;;
;;; Revision 1.22  2009/03/27 20:19:35  nozawa
;;; add argument(args) to :collision-avoidance-link-pair
;;;
;;; Revision 1.21  2009/03/23 08:40:04  k-okada
;;; fix debug-view angle: display format error
;;;
;;; Revision 1.20  2009/03/11 16:37:03  k-okada
;;; integrate calc-dif-with-{translation,rotation}-axis to calc-dif-with-axis
;;;
;;; Revision 1.19  2009/03/11 16:32:49  k-okada
;;; integrate calc-dif-with-{translation,rotation}-axis to calc-dif-with-axis
;;;
;;; Revision 1.18  2009/03/11 13:07:03  k-okada
;;; integrate calc-dif-with-{translation,rotation}-axis to calc-dif-with-axis
;;;
;;; Revision 1.17  2009/03/11 12:59:30  k-okada
;;; do not calculate weight when avoid-weight-gain and avoid-nspace-gain
;;;
;;; Revision 1.16  2009/03/10 09:58:59  k-okada
;;; add :angle-speed keyword to :move-joints
;;;
;;; Revision 1.15  2009/03/07 01:53:00  k-okada
;;; update (:collision-avoidance, (:move-joints-avoidance
;;;
;;; Revision 1.14  2009/03/05 10:45:13  k-okada
;;; fix wmat, umat, support when min/max-angle is *inf*
;;;
;;; Revision 1.13  2009/03/02 12:16:47  k-okada
;;; fix :calc-inverse-jacobian when degenerated link-list,  fix when p-limit/r-limit is nil
;;;
;;; Revision 1.12  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.11  2009/02/17 01:51:08  k-okada
;;; add :analysis-level to bodyset-link
;;;
;;; Revision 1.10  2009/01/02 08:09:27  k-okada
;;; declare analysis-level in bodyset-link
;;;
;;; Revision 1.9  2008/11/11 14:05:34  eus
;;; fix to work when no viewer created, again
;;;
;;; Revision 1.8  2008/11/11 11:09:35  k-okada
;;; fix when no pickview/irtviewer is created
;;;
;;; Revision 1.7  2008/11/07 06:30:07  k-okada
;;; fix
;;;
;;; Revision 1.6  2008/11/07 01:15:41  k-okada
;;; fix when avoid-collision-gain 0
;;;
;;; Revision 1.5  2008/11/06 17:19:47  k-okada
;;; fix to work with jskrbeusgl
;;;
;;; Revision 1.4  2008/10/10 05:12:28  k-okada
;;; add debug message
;;;
;;; Revision 1.3  2008/10/09 15:10:08  k-okada
;;; fix to work with jskrbeusgl
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
