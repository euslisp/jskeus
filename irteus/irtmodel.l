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
                weight acentroid inertia-tensor
                angular-velocity angular-acceleration
                joint-velocity joint-acceleration joint-torque
                spacial-velocity spacial-acceleration
                momentum-velocity angular-momentum-velocity
                momentum angular-momentum
                force moment ext-force ext-moment))
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
   (send self :reset-dynamics)
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
  (:reset-dynamics
   ()
   (setq joint-velocity 0 ;; [rad/s]
         joint-acceleration 0 ;; [rad/s^2]
         joint-torque 0 ;; [Nm]
         angular-velocity (float-vector 0 0 0) ;; [rad/s]
         angular-acceleration (float-vector 0 0 0) ;; [rad/s^2]
         spacial-velocity (float-vector 0 0 0) ;; [mm/s]
         spacial-acceleration (float-vector 0 0 0) ;; [mm/s^2]
         angular-momentum (float-vector 0 0 0) ;; [kg mm^2/s]
         momentum (float-vector 0 0 0) ;; [kg mm/s]
         angular-momentum-velocity (float-vector 0 0 0) ;; [kg mm^2/s^2]
         momentum-velocity (float-vector 0 0 0) ;; [kg mm^2/s^2]
         force (float-vector 0 0 0) ;; [N] = [kg m/s^2]
         moment (float-vector 0 0 0) ;; [Nm] = [kg m^2/s^2]
         ext-force (float-vector 0 0 0) ;; [N] = [kg m/s^2]
         ext-moment (float-vector 0 0 0))) ;; [Nm] = [kg m^2/s^2]
  (:joint-velocity (&optional jv) (if jv (setq joint-velocity jv) joint-velocity))
  (:joint-acceleration (&optional ja) (if ja (setq joint-acceleration ja) joint-acceleration))
  (:angular-velocity () angular-velocity)
  (:angular-acceleration () angular-acceleration)
  (:spacial-velocity () spacial-velocity)
  (:spacial-acceleration () spacial-acceleration)
  (:force () force)
  (:moment () moment)
  (:ext-force (&optional f) (if f (setq ext-force f) ext-force))
  (:ext-moment (&optional m) (if m (setq ext-moment m) ext-moment))
  (:joint-torque () joint-torque)
  (:forward-all-kinematics
   (&key (debug-view nil))
   (when (and joint (not (derivedp joint rotational-joint)))
     (warn ";; forward-all-kinematics of ~A is not implemented~%" (send (class joint) :name))
     (return-from :forward-all-kinematics nil))
   (if debug-view (format t ";; forward-all-kinematics link = ~A~%" (send self :name)))
   (when (and parent-link (derivedp joint rotational-joint))
     (let (sw sv dsv dsw paxis)
       (setq paxis (case (joint . axis)
                         (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                         (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                         (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                         (t (joint . axis))))
       (setq sw (send parent-link :rotate-vector paxis)
             sv (v* (send self :worldpos) sw)
             angular-velocity (v+ (send parent-link :angular-velocity) (scale joint-velocity sw))
             spacial-velocity (v+ (send parent-link :spacial-velocity) (scale joint-velocity sv)))
       
       (setq dsv (v+ (v* (send parent-link :angular-velocity) sv)
                     (v* (send parent-link :spacial-velocity) sw))
             dsw (v* (send parent-link :angular-velocity) sw)
             angular-acceleration (v+ (v+ (send parent-link :angular-acceleration)
                                          (scale joint-velocity dsw))
                                      (scale joint-acceleration sw))
             spacial-acceleration (v+ (v+ (send parent-link :spacial-acceleration)
                                          (scale joint-velocity dsv))
                                      (scale joint-acceleration sv)))
       ))
   (send-all child-links :forward-all-kinematics)
   )
  (:inverse-dynamics
   (&key (dt 0.005) (debug-view nil)) ;; dt:[s]
   (when (and joint (not (derivedp joint rotational-joint)))
     (warn ";; inverse-dynamics of ~A is not implemented~%" (send (class joint) :name))
     (return-from :inverse-dynamics nil))
   (if debug-view (format t ";; inverse-dynamics link = ~A~%" (send self :name)))
   (let (child sister c I c-hat P L fg w iner)
     (setq w (* 0.001 (send self :weight)) ;; [g] -> [kg]
           iner (scale-matrix 0.001 (send self :inertia-tensor)) ;; [g mm^2] -> [kg mm^2]
           fg (scale (* -1.0 w 0.001) *g-vec*) ;; [N]
           c (send self :centroid) ;; [mm]
           I (m* (m* (send self :worldrot) iner) (transpose (send self :worldrot))) ;; [kg mm^2]
           c-hat (outer-product-matrix c) ;; [mm]
           I (m+ I (scale-matrix w (m* c-hat (transpose c-hat)))) ;; [kg mm^2]
           P (scale w (v+ spacial-velocity (v* angular-velocity c))) ;; [kg mm/s]
           L (v+ (scale w (v* c spacial-velocity)) (transform I angular-velocity))) ;; [kg mm^2/s]
     
     (setq momentum-velocity (scale (/ 1.0 dt) (v- P momentum)) ;; [kg mm/s^2]
           angular-momentum-velocity (scale (/ 1.0 dt) (v- L angular-momentum))) ;; [kg mm^2/s^2]

     (setq momentum P angular-momentum L ;; [kg mm/s], [kg mm^2/s]
           force (scale 0.001 (v+ (scale w (v+ spacial-acceleration
                                               (v* angular-acceleration c)))
                                  (v* angular-velocity P))) ;; [N]
           moment (scale 1e-6 (v+ (v+ (v+ (scale w (v* c spacial-acceleration))
                                          (transform I angular-acceleration))
                                      (v* spacial-velocity P))
                                  (v* angular-velocity L)))) ;; [N]
     
     (setq force (v- force (v+ fg ext-force)) ;; [N]
           moment (v- moment (v+ (scale 0.001 (v* c (v+ fg ext-force))) ext-moment))) ;; [Nm]

     ;; propagation of force and moment from child-links
     (when child-links
       (dolist (child child-links)
         (send child :inverse-dynamics)
         (setq force (v+ force (send child :force))
               moment (v+ moment (send child :moment)))
         )
       )
     
     (when (and joint parent-link)
       (let (sw sv tau paxis)
         (setq paxis (case (joint . axis)
                           (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                           (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                           (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                           (t (joint . axis))))
         (setq sw (send parent-link :rotate-vector paxis)
               sv (v* (send self :worldpos) sw))
         ;; [Nm] = 0.001 * [mm] [N] or 0.001 * [rad] [Nm]
         (setq joint-torque (+ (* 0.001 (v. sv force)) (v. sw moment)))
         ))
     ))
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
   ;; rotation-axis, translation-axis -> both list and atom OK.
   (let ((dim (* 6 (if (atom rotation-axis) 1 (length rotation-axis)))))
     (dolist (axis (append (if (atom translation-axis)
                               (list translation-axis) translation-axis)
                           (if (atom rotation-axis)
                               (list rotation-axis) rotation-axis)))
       (case axis
	     ((:x :y :z :xx :yy :zz) (decf dim 1))
	     ((:xy :yx :yz :zy :zx :xz) (decf dim 2))
	     (nil (decf dim 3))))
     dim))
  (:calc-target-joint-dimension
   (link-list)
   ;; link-list -> both list of link-list and atom of link-list OK.
   (cond
    ((atom (car link-list))
     (calc-target-joint-dimension (send-all link-list :joint)))
    ((= (length link-list) 1)
     (calc-target-joint-dimension (send-all (car link-list) :joint)))
    (t (calc-target-joint-dimension (send-all (reduce #'union link-list) :joint)))
    ))
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
   (fik row column joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (setq tmp-v3 (send self :calc-jacobian-child-rotate-vector
			paxis child-link move-target tmp-v3 tmp-v3a tmp-v3b tmp-m33))

     (setq pos-r (calc-dif-with-axis tmp-v3 translation-axis tmp-v0 tmp-v1 tmp-v2))
     (setq pos-r (scale 0.001 pos-r pos-r));; scale [mm] -> [m]
     (dotimes (j (length pos-r)) (setf (aref fik (+ j row) column) (elt pos-r j)))
     (setq pos-i (calc-dif-with-axis #f(0 0 0) rotation-axis tmp-v0 tmp-v1 tmp-v2))
     (dotimes (j (length pos-i)) (setf (aref fik (+ j row (length pos-r)) column) (elt pos-i j)))
     ))
  (:calc-jacobian-rotational-joint
   (fik row column joint paxis child-link child-reverse
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
     (dotimes (j (length pos-r)) (setf (aref fik (+ j row) column) (elt pos-r j)))
     (setq pos-i (calc-dif-with-axis tmp-v3 rotation-axis tmp-v0 tmp-v1 tmp-v2))
     (dotimes (j (length pos-i)) (setf (aref fik (+ j row (length pos-r)) column) (elt pos-i j)))
     ))
  (:calc-jacobian-wheel-joint
   (fik row column joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik row column joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 1) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ;;(setf (aref fik 2 1) 0)
     ))
  (:calc-jacobian-omniwheel-joint
   (fik row column joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik row column joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik row (+ column 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 2) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-sphere-joint
   (fik row column joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-rotational-joint
	   fik row column joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 2) joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-6dof-joint
   (fik row column joint paxis child-link child-reverse
	move-target rotation-axis translation-axis
	pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (send self :calc-jacobian-linear-joint
	   fik row column joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik row (+ column 1) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-linear-joint
	   fik row (+ column 2) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 3) joint #f(0 0 1) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 4) joint #f(0 1 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (send self :calc-jacobian-rotational-joint
	   fik row (+ column 5) joint #f(1 0 0) child-link child-reverse
	   move-target rotation-axis translation-axis
	   pos-i pos-r tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  (:calc-jacobian-from-link-list
   (link-list &rest args 
	      &key move-target
	      (rotation-axis (cond 
			      ((atom move-target) nil)
			      (t (make-list (length move-target)))))
	      (translation-axis (cond 
				 ((atom move-target) t)
				 (t (make-list (length move-target) :initial-element t))))
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
	      &allow-other-keys)
   (let* (len ul row
	  pos-i pos-r
	  paxis
	  child-link child-reverse union-link-list
	  j)
     (unless fik (setq fik (make-matrix dim fik-len)))
;;
     (cond
      ((atom (car link-list))
       (setq union-link-list link-list
	     link-list (list link-list)))
      ((= (length link-list) 1)
       (setq union-link-list (car link-list)))
      (t (setq union-link-list (reduce #'union link-list))))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
;;
     (do ((col 0 (1+ col))
	  (i 0 (1+ i)))
         ((>= col (send self :calc-target-joint-dimension union-link-list)))
       (setq ul (elt union-link-list i) row 0)
       (dotimes (m (length link-list))
         (let ((link-list (elt link-list m))
               (move-target (elt move-target m))
               (rotation-axis (elt rotation-axis m))
               (translation-axis (elt translation-axis m)) l)
           (when (member ul link-list :test #'equal)
             (setq len (length link-list)
                   l (position ul link-list :test #'equal)
                   j (send ul :joint))
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
                     fik row col j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              ((derivedp j rotational-joint)
               (send self :calc-jacobian-rotational-joint
                     fik row col j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              ((derivedp j wheel-joint)
               (send self :calc-jacobian-wheel-joint
                     fik row i j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              ((derivedp j omniwheel-joint)
               (send self :calc-jacobian-omniwheel-joint
                     fik row col j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              ((derivedp j sphere-joint)
               (send self :calc-jacobian-sphere-joint
                     fik row col j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              ((derivedp j 6dof-joint)
               (send self :calc-jacobian-6dof-joint
                     fik row col j paxis child-link child-reverse move-target
                     rotation-axis translation-axis pos-i pos-r
                     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33))
              (t
               (warn "unknown joint type ~A~%" j)))
             ) ;; when
           (setq row (send self :calc-target-axis-dimension
			   rotation-axis translation-axis))
           )) ;; dotimes
       (cond
        ((derivedp j wheel-joint) (incf col 1))
        ((derivedp j omniwheel-joint) (incf col 2))
        ((derivedp j sphere-joint) (incf col 2))
        ((derivedp j 6dof-joint) (incf col 5)))
       ) ;; do
     fik))
  ;;
  (:calc-joint-angle-speed
   (union-vel
    &rest args
    &key
    jacobi jacobi# null-space debug-view
    weight wmat tmp-len tmp-len2 fik-len tmp-mcc tmp-mcc2
    &allow-other-keys)
   (let (vel-p vel-r J#x I-J#J)
     ;; dav = J#x + (I - J#J)y
     ;; particular : J#x
     (when (null jacobi)
       (warn ";; ERROR : jacobi is required in :calc-joint-angle-speed~%")
       (return-from :calc-joint-angle-speed null-space))
     (if (null fik-len) (setq fik-len (cadr (array-dimensions jacobi))))
     (if (null tmp-len) (setq tmp-len (instantiate float-vector fik-len)))
     (if (null tmp-len2) (setq tmp-len2 (instantiate float-vector fik-len)))
     (when (null jacobi#)
       (setq jacobi# (send* self :calc-inverse-jacobian jacobi args)))

     (setq J#x (transform jacobi# union-vel tmp-len))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "J#t   :")
       (dotimes (j (cadr (array-dimensions jacobi#)))
	 (if (/= j 0) (warn "       "))
	 (dotimes (i fik-len) (warn "~7,3f " (aref jacobi# i j))) (warn "~%"))
       (warn "x     :")
       (dotimes (i (length union-vel)) (warn "~7,3f " (elt union-vel i))) (warn "~%")
       (warn "J#x   :")
       (dotimes (i fik-len) (warn "~7,3f " (elt J#x i))) (warn "~%"))

     ;; if use null space
     (when (and (vectorp null-space)
		(= fik-len (length null-space)))
       (if (null tmp-mcc) (setq tmp-mcc (make-matrix fik-len fik-len)))
       (if (null tmp-mcc2) (setq tmp-mcc2 (make-matrix fik-len fik-len)))
       (if (null weight) (setq weight (fill (instantiate float-vector fik-len) 1)))
       ;; projection : I - J#J
       ;; (unit-matrix (car (array-dimensions jacobi#))))
       (fill (array-entity tmp-mcc) 0)
       (dotimes (i fik-len) (setf (aref tmp-mcc i i) 1.0))
       (setq I-J#J (m- tmp-mcc
		       (m* jacobi# jacobi tmp-mcc2) tmp-mcc))
       ;; add null space
       (if (null wmat) (setq wmat (make-matrix fik-len fik-len)))
       (dotimes (i fik-len) (setf (aref wmat i i) (aref weight i)))
       (v+ J#x (transform wmat (transform I-J#J null-space tmp-len2) tmp-len2) J#x))
     J#x))
  (:collision-avoidance-links
   (&optional l)
   (if l (setq collision-avoidance-links l)) collision-avoidance-links)
  (:collision-avoidance-link-pair-from-link-list
    (link-list &key obstacles ((:collision-avoidance-links collision-links) collision-avoidance-links) debug)
    ;; link-list -> both list and atom OK.
    (if (not (atom (car link-list)))
	(let ((ll (mapcar #'(lambda (x) (send self :collision-avoidance-link-pair-from-link-list x)) link-list)) ret)
	  (setq ret (reduce #'append ll))
	  (if (= (length link-list) 1) (car ret) ret)
	  )
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
     (when (and debug (not (memq :no-message debug)))
       (warn "collision-links : ~A~%" (send-all collision-links :name))
       (warn "parnet-links    : ~A~%" (send-all parent-links :name))
       (warn "valid-link-list : ~A~%" (send-all valid-link-list :name))
       (warn "index-links     : ~A~%" index-links))
     (dotimes (i len)
       (do ((j (1+ i) (1+ j)))
           ((>= j len))
           (if (and (or (setq i0 (elt index-links i))
                        (elt index-links j))
                    (not (eq (elt parent-links i)
                             (elt collision-links j)))
                    (not (eq (elt parent-links j)
                             (elt collision-links i)))
                    (not (eq (elt parent-links j)
                             (elt parent-links i))))
               (push (list (elt collision-links (if i0 i j))
                           (elt collision-links (if i0 j i))) ret))))
     (when obstacles
       (if (atom obstacles) (setq obstacles (list obstacles)))
       (dotimes (i len)
         (if (elt index-links i)
             (dolist (o obstacles)
               (push (list (elt collision-links i) o) ret)))))
     ret)))
  (:collision-avoidance-calc-distance
   (&rest args
	  &key union-link-list (warnp t)
          ((:collision-avoidance-link-pair pair-list))
	  &allow-other-keys)
   (let* ((pair-len (length pair-list))
	  (ret (make-list pair-len))
	  (col-list (send self :get :collision-distance))
	  (distance-limit 10)
	  (sc 0.0) np pair)
     (dotimes (i pair-len) (setf (elt ret i) (float-vector 0 0 0)))
     (when (or (null union-link-list) (= pair-len 0))
       (return-from :collision-avoidance-calc-distance ret))
     (if (or (null col-list) (/= (length col-list) pair-len))
	 (setq col-list (make-list pair-len)))
     ;; pair (part of this limb . part of another limb)
     (dotimes (i pair-len)
       (setq pair (elt pair-list i))
       (if (not (memq (car pair) union-link-list)) 
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
   ;; link-list -> must not be list of link-list
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
   (&rest args
            &key
	    avoid-collision-distance
	    avoid-collision-joint-gain
	    avoid-collision-null-gain	    
            ((:collision-avoidance-link-pair pair-list))
	    (union-link-list) (link-list) (jacobi) (jacobi#)
            ;; link-list -> both list and atom OK.
	    (fik-len (send self :calc-target-joint-dimension union-link-list))
            (tmp-mcc (make-matrix fik-len fik-len))
	    (tmp-mcc2 (make-matrix fik-len fik-len))
            debug-view
            &allow-other-keys)
   (let* ((ret (send* self :collision-avoidance-calc-distance args))
          (pair-len (length pair-list))
          (np (elt (send self :get :collision-distance) 0))
          (min-distance (car np))
          (pair (elt pair-list (car (last np))))
	  (dav-col (instantiate float-vector fik-len))
          drag-coords col-link-list col-jacobi I-J#J ddav-col col-args
          )
     (send self :put :collision-pair-list pair-list)
     (when (and debug-view (not (memq :no-message debug-view)))
       (let ((v (normalize-vector (v- (elt np 1) (elt np 2)))))
         (warn "min-distance : ~7,3f #f(" min-distance)
         (dotimes (i 3) (warn "~7,3f " (elt v i)))
         (warn ") ~A~%" (send-all pair :name))))
     (if (>= min-distance avoid-collision-distance)
         (return-from :collision-avoidance dav-col))
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (setq link-list (find-if #'(lambda (x) (member (car pair) x :test #'equal)) link-list))
     (setq col-args (send self :collision-avoidance-args pair link-list))

     (setq drag-coords (make-cascoords :pos (elt np 1)))
     (send (car pair) :assoc drag-coords)
     (setq col-link-list (send self :link-list (car pair) (car link-list)))
     (setq col-jacobi (send* self :calc-jacobian-from-link-list
                             col-link-list :move-target drag-coords
                             col-args))
     (send (car pair) :dissoc drag-coords)
     (setq ddav-col (transform (transpose col-jacobi) (elt ret (car (last np)))))
     
     (dotimes (i (length link-list))
       (setf (elt dav-col (position (elt link-list i) union-link-list :test #'equal) ) (elt ddav-col i)))
     (when (or (null jacobi) (null jacobi#))
       (warn ";; ERROR : jacobi or jacobi# are required in :collision-avoidance~%")
       (return-from :collision-avoidance dav-col))
     
     ;;;
     (fill (array-entity tmp-mcc) 0)
     (dotimes (i fik-len) (setf (aref tmp-mcc i i) 1.0))

     (setq I-J#J (m- tmp-mcc (m* jacobi# jacobi tmp-mcc2) tmp-mcc))
     (setq ddav-col (transform I-J#J dav-col))
     (dotimes (i fik-len) (setf (elt dav-col i)
                            (+ (* avoid-collision-joint-gain (elt dav-col i))
                               (* avoid-collision-null-gain (elt ddav-col i)))))
     (scale (- (* avoid-collision-distance (/ 1.0 min-distance)) 1) dav-col)))
  (:move-joints
   (union-vel &rest args
    &key
    union-link-list
    angle-speed
    (angle-speed-blending 0.5)
    (angle-speed-limit (/ pi 4)) ;; rad/sec
    (periodic-time 0.05)         ;; sec
    (debug-view nil)
    (fix-targets)
    &allow-other-keys)
   (let (dav dtheta j fix-target fix-targets-coords (ret t) (angle-speed-max 0) gain)
     (if fix-targets
         (setq fix-targets-coords (send-all fix-targets :copy-worldcoords)))
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (if (and debug-view (not (equal debug-view :no-clear)) (boundp '*viewer*))
	 (send *viewer* :viewsurface :clear))

     (setq dav (send* self :calc-joint-angle-speed union-vel args))
     (if angle-speed (midpoint angle-speed-blending dav angle-speed dav))
     (dotimes (i (length dav)) (setq angle-speed-max (max (abs (elt dav i)) angle-speed-max)))
     (if (> (/ angle-speed-max periodic-time) angle-speed-limit)
         (setq gain (/ angle-speed-limit (/ angle-speed-max periodic-time)))
       (setq gain 1))

     ;; update body
     (do ((i 0 (1+ i))
	  (l 0 (1+ l)))
	 ((>= l (length union-link-list)))
       (setq j (send (elt union-link-list l) :joint))
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
       (if (float-vector-p dtheta)
           (setq dtheta (scale gain dtheta dtheta))
         (setq dtheta (* gain dtheta)))
       (send (elt union-link-list l) :joint :joint-angle dtheta :relative t))
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
     ret))
  (:move-joints-avoidance
   (union-vel &rest args
              &key
              union-link-list link-list
              (fik-len (send self :calc-target-joint-dimension union-link-list))
              (weight)
              (null-space)
              (avoid-nspace-gain 0.01)
              (avoid-weight-gain 1.0)
              (avoid-collision-distance 200)
              (avoid-collision-null-gain 1.0)
              (avoid-collision-joint-gain 1.0)
              ((:collision-avoidance-link-pair pair-list)
               (send self :collision-avoidance-link-pair-from-link-list link-list :obstacles (cadr (memq :obstacles args)) :debug (cadr (memq :debug-view args))))
              (tmp-len (instantiate float-vector fik-len))
              (tmp-len2 (instantiate float-vector fik-len))
              (debug-view) (jacobi)
	    &allow-other-keys)
   (let* (joint-angle-limit-nspace
	  joint-angle-limit-weight
	  joint-angle-limit-weight-old
	  dav dtheta 
	  (null-space-joint-limit (instantiate float-vector fik-len))
          null-space-collision-avoidance
          angle-speed-collision-avoidance
          (angle-speed-collision-blending 0.0) min-distance
          (ret t) jacobi#)
     (when (null jacobi)
       (warn ";; ERROR : jacobi is required in :move-joints-avoidance~%")
       (return-from :move-joints-avoidance ret))
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (unless (get self :joint-angle-limit-weight-old)
       (setf (get self :joint-angle-limit-weight-old) (make-hash-table)))
     (setq joint-angle-limit-weight-old
       (gethash union-link-list (get self :joint-angle-limit-weight-old)))
     (unless joint-angle-limit-weight-old
       (setq joint-angle-limit-weight-old (instantiate float-vector fik-len))
       (fill joint-angle-limit-weight-old 1.0e+20)
      (setf (gethash union-link-list (get self :joint-angle-limit-weight-old))
	     joint-angle-limit-weight-old))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "angle :")
       (dolist (j (send-all union-link-list :joint))
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
     (setq weight (if weight
                      (copy-object weight)
                    (fill (instantiate float-vector fik-len) 1)))
     (when (> avoid-weight-gain 0.0)
       (setq joint-angle-limit-weight 
	 (scale avoid-weight-gain
		(joint-angle-limit-weight (send-all union-link-list :joint) tmp-len) tmp-len))
       ;; (setq joint-angle-limit-weight (instantiate float-vector fik-len))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn "cost  :")
	 (dotimes (i fik-len) (warn "~7,3f " (elt joint-angle-limit-weight i))) (warn "~%"))
       (dotimes (i fik-len)
	 (setf (elt weight i)
	       (* (elt weight i) 
		  (if (> (- (elt joint-angle-limit-weight i)
                            (elt joint-angle-limit-weight-old i)) 0.0)
		      (/ 1.0 (+ 1.0 (elt joint-angle-limit-weight i)))
		    1.0)))
         (setf (elt weight i) (max 0.001 (elt weight i)))
         (setf (elt joint-angle-limit-weight-old i) (elt joint-angle-limit-weight i))
         )
       )
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "weight:")
       (dotimes (i  fik-len) (warn "~7,3f " (elt weight i))) (warn "~%"))
     ;;
     ;; avoid-nspace-joint-limit: avoiding joint angle limit
     ;; 
     ;; dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
     ;;
     (when (> avoid-nspace-gain 0.0)
       (setq joint-angle-limit-nspace (joint-angle-limit-nspace (send-all union-link-list :joint) tmp-len2))
       (setq null-space-joint-limit
	 (scale avoid-nspace-gain
                joint-angle-limit-nspace tmp-len2))
       (when (and debug-view (not (memq :no-message debug-view)))
	 (warn "nspace:")
	 (dotimes (i fik-len) (warn "~7,3f " (elt null-space-joint-limit i))) (warn "~%")))
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
     (setq jacobi# (send* self :calc-inverse-jacobian jacobi :weight weight args))
     (when (and pair-list (> avoid-collision-distance 0.0)
		(or (> avoid-collision-joint-gain 0.0)
		    (> avoid-collision-null-gain 0.0)))
       (setq angle-speed-collision-avoidance
	     (send* self :collision-avoidance
			 :jacobi# jacobi#
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
	 (dotimes (i fik-len) (warn "~7,3f " (elt angle-speed-collision-avoidance i))) (warn "~%")
         (warn "belnd :~7,3f~%" angle-speed-collision-blending)))
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
                        :move-joints union-vel
                        :union-link-list union-link-list
                        :null-space null-space-joint-limit
                        :angle-speed angle-speed-collision-avoidance
                        :angle-speed-blending angle-speed-collision-blending
                        :weight weight :jacobi# jacobi#
                        args))
     ret))
  ;;
  (:inverse-kinematics-args
   (&rest args &key union-link-list rotation-axis translation-axis
          &allow-other-keys)
   (let* ((c (send self :calc-target-joint-dimension union-link-list))
          (r (send self :calc-target-axis-dimension rotation-axis translation-axis))
          (fik (make-matrix r c))
          (ret (make-matrix c r))
          (wmat (make-matrix c c))
          (tmat (make-matrix c r))
          (umat (make-matrix r r))
          (umat2 (make-matrix r r))
          (mat-tmp (make-matrix c r))
          (tmp-mcc (make-matrix c c))
          (tmp-mcc2 (make-matrix c c))
          (tmp-mrr (make-matrix r r))
          (tmp-mrc (make-matrix r c))
          (tmp-v0 (instantiate float-vector 0))
          (tmp-v1 (instantiate float-vector 1))
          (tmp-v2 (instantiate float-vector 2))
          (tmp-v3 (instantiate float-vector 3))
          (tmp-v3a (instantiate float-vector 3))
          (tmp-v3b (instantiate float-vector 3))
          (tmp-v3c (instantiate float-vector 3))
          (tmp-m33 (make-matrix 3 3))
          (tmp-dim (instantiate float-vector r))
          (tmp-dims)
          (tmp-len (instantiate float-vector c))
          (tmp-len2 (instantiate float-vector c))
          (tmp-pos (instantiate float-vector 3))
          (tmp-rot (instantiate float-vector 3)))
     (dotimes (i (length rotation-axis))
       (push (instantiate float-vector (send self :calc-target-axis-dimension (elt rotation-axis i) (elt translation-axis i))) tmp-dims))
     (nreverse tmp-dims)
     (list
      :dim r :fik-len c
      :fik fik
      :ret ret
      :wmat wmat :tmat tmat :umat umat :umat2 umat2 :mat-tmp mat-tmp
      :tmp-mcc tmp-mcc :tmp-mcc2 tmp-mcc2 :tmp-mrr tmp-mrr
      :tmp-mrc tmp-mrc
      :tmp-v0 tmp-v0 :tmp-v1 tmp-v1
      :tmp-v2 tmp-v2 :tmp-v3 tmp-v3
      :tmp-v3a tmp-v3a :tmp-v3b tmp-v3b  :tmp-v3c tmp-v3c
      :tmp-m33 tmp-m33 
      :tmp-dim tmp-dim :tmp-dims tmp-dims
      :tmp-len tmp-len :tmp-len2 tmp-len2
      :tmp-pos tmp-pos :tmp-rot tmp-rot
      )))
  (:draw-collision-debug-view
    ()
    (when (send self :get :collision-pair-list)
      (let ((pwidth (send *viewer* :viewsurface :line-width))
            (psize (send *viewer* :viewsurface :point-size))
            (pcolor (send *viewer* :viewsurface :color))
            (pair-list (send self :get :collision-pair-list))
            np)
        (send *viewer* :viewsurface :color #f(1 0 0))
        (send *viewer* :viewsurface :line-width 1)
        (send *viewer* :viewsurface :point-size 3)
        (dotimes (i (length pair-list))
          (setq np (elt (send self :get :collision-distance) i))
          (unless np
            (warn ";; ERROR : could not get collision-distance ~A"
                  (send-all pair-list :name))
            (return))
          (send *viewer* :viewsurface :color #f(0.7 0.0 0.7))
          (send *viewer* :viewsurface :line-width 1)
          (send *viewer* :viewsurface :point-size 2)
          (send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
          (send *viewer* :viewsurface :3d-point (elt np 2) :depth-test t)
          (send *viewer* :viewsurface :3d-line (elt np 1) (elt np 2)
                :depth-test t)
          (if (< (car np) 200);avoid-collision-distance
              (progn
                (send *viewer* :viewsurface :line-width 4)
                (send *viewer* :viewsurface :point-size 6))
            (progn
              (send *viewer* :viewsurface :line-width 2)
              (send *viewer* :viewsurface :point-size 3)))
          (send *viewer* :viewsurface :color #f(1.0 0.0 1.0))
          (send *viewer* :viewsurface :3d-line (elt np 1)(elt np 2)
                :depth-test nil)
          (send *viewer* :viewsurface :3d-point (elt np 1) :depth-test nil)
          (send *viewer* :viewsurface :3d-point (elt np 2) :depth-test nil)
          (when (<= (car np) 1.0)
            (dolist (obj (elt pair-list (car (last np))))
              (when (and (derivedp obj bodyset-link)
                         (eq (send obj :analysis-level) :coords))
                (send-all (obj . geo::bodies) :worldcoords))
              (dolist (e (cdr (send obj :edges)))
                (send *viewer* :viewsurface :3d-line
                      (e . pvert) (e . nvert) :depth-test t))))
          )
        (send *viewer* :viewsurface :line-width pwidth)
        (send *viewer* :viewsurface :point-size psize)
        (send *viewer* :viewsurface :color pcolor)
        ))
    )
  (:inverse-kinematics-loop
   (target-coords 
    &rest args
    &key (stop 1) (loop 0) link-list move-target rotation-axis translation-axis
    look-at-target thre rthre union-link-list
    debug-view ik-args
    &allow-other-keys)
   ;; target-coords, move-target, rotation-axis, translation-axis, link-list
   ;; -> both list and atom OK.
   (let (dif-pos dif-rot tmp-dim tmp-dims (vec-count 0) (success t) jacobi
         (method-args (nconc args ik-args)))
       (if (null union-link-list)
           (setq union-link-list (cond
                                  ((atom (car link-list)) link-list)
                                  ((= (length link-list) 1) (car link-list))
                                  (t (reduce #'union link-list)))))
       ;; atom -> list
       (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
       (if (and debug-view (not (equal debug-view :no-clear)) (boundp '*viewer*))
	   (send *viewer* :viewsurface :clear))
       (if (atom (car link-list)) (setq link-list (list link-list)))
       (if (atom move-target) (setq move-target (list move-target)))
       (if (atom target-coords) (setq target-coords (list target-coords)))
       (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
       (if (atom translation-axis) (setq translation-axis (list translation-axis)))       
       (if (atom thre) (setq thre (list thre)))
       (if (atom rthre) (setq rthre (list rthre)))
       ;; argument check
       (unless (= (length translation-axis) (length rotation-axis)
                  (length move-target) (length link-list) (length target-coords))
         (warn ";; ERROR: list length differ : translation-axis ~A rotation-axis ~A move-target ~A link-list ~A target-coords ~A~%"
               (length translation-axis) (length rotation-axis)
               (length move-target) (length link-list) (length target-coords))
         (return-from :inverse-kinematics-loop :ik-fail))

       (if (memq :tmp-dims ik-args)
           (setq tmp-dims (cadr (memq :tmp-dims ik-args)))
         (progn
           (dotimes (i (length rotation-axis))
             (push (instantiate float-vector (send self :calc-target-axis-dimension (elt rotation-axis i) (elt translation-axis i))) tmp-dims))
           (setq tmp-dims (nreverse tmp-dims))))
       (if (memq :tmp-dim ik-args)
           (setq tmp-dim (cadr (memq :tmp-dim ik-args)))
         (setq tmp-dim (instantiate float-vector (send self :calc-target-axis-dimension rotation-axis translation-axis))))
       
       (setq jacobi (send* self :calc-jacobian-from-link-list link-list
                           :translation-axis translation-axis
                           :rotation-axis rotation-axis
                           :move-target move-target
                           method-args))
       (dotimes (i (length move-target))
         (let ((move-target (elt move-target i))
               (target-coords (elt target-coords i))
               (rotation-axis (elt rotation-axis i))
               (translation-axis (elt translation-axis i))
               (thre (elt thre i)) (rthre (elt rthre i)))
           (setq dif-pos (send move-target :difference-position target-coords :translation-axis translation-axis)
                 dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
           (when (and debug-view (not (memq :no-message debug-view)))
             (warn ";; ~3d: dif-pos : ~a/(~a/~A), dif-rot : ~a/(~a/~A)~%"
                   loop dif-pos (norm dif-pos) thre dif-rot (norm dif-rot) rthre))
           (setq success (and success
                              (> loop (/ stop 10))
                              (if translation-axis (< (norm dif-pos) thre) t)
                              (if rotation-axis (< (norm dif-rot) rthre) t)))
           (send* self :calc-vel-from-dif-pos-rot
                 dif-pos dif-rot translation-axis rotation-axis
                 :tmp-dim (elt tmp-dims i) method-args)
           ))
       (dotimes (i (length tmp-dims))
         (dotimes (j (length (elt tmp-dims i)))
           (setf (elt tmp-dim (+ j vec-count)) (elt (elt tmp-dims i) j)))
         (incf vec-count (length (elt tmp-dims i))))
       
       ;; check loop end
       (if (and (> loop (/ stop 10)) success)
           (return-from :inverse-kinematics-loop :ik-succeed))
       (when (and debug-view (not (memq :no-clear debug-view)) *viewer*)
         (send *viewer* :viewsurface :clear))
       (send self :put :collision-pair-list nil)
       (setq
        success
        (send-message* self cascaded-link
		       :move-joints-avoidance tmp-dim
                       :union-link-list union-link-list
                       :rotation-axis rotation-axis :translation-axis translation-axis
                       :jacobi jacobi :debug-view debug-view
                       ;; buffer for calculation
                       method-args))
       (when (and debug-view  *viewer*)
         (send *viewer* :draw-objects :clear nil :flush nil)
         ;;
         ;; done
         (send self :draw-collision-debug-view)
         (dotimes (i (length target-coords))
           (send-message (elt move-target i) cascaded-link :draw-on :flush nil :size 100)
           (send (elt target-coords i) :draw-on :flush nil :color #f(1 0 0)))
         (if (not (memq :no-flush debug-view)) (send *viewer* :viewsurface :flush))
         )
       (unless success (return-from :inverse-kinematics-loop :ik-failed))
       (if look-at-target
	   (send self :head :look-at (send (car target-coords) :worldpos)))
       :ik-continues))
  (:inverse-kinematics
   (target-coords &rest args 
		  &key (stop 50)
		  (link-list)
		  (move-target)
		  (debug-view) (warnp t)
                  (rotation-axis (cond
				  ((atom move-target) t)
				  (t (make-list (length move-target) :initial-element t))))
                  (translation-axis (cond
				     ((atom move-target) t)
				     (t (make-list (length move-target) :initial-element t))))
		  (look-at-target)
                  (fix-targets)
		  (thre (cond
                         ((atom move-target) 1)
                         (t (make-list (length move-target) :initial-element 1))))
		  (rthre (cond
                          ((atom move-target) (deg2rad 1))
                          (t (make-list (length move-target) :initial-element (deg2rad 1)))))
		  &allow-other-keys)
   ;; target-coords, move-target, rotation-axis, translation-axis, link-list
   ;; -> both list and atom OK.
   (let* ((loop 0)
          (union-link-list (cond
			    ((atom (car link-list)) link-list)
			    ((= (length link-list) 1) (car link-list))
			    (t (reduce #'union link-list))))
          (av0 (send-all (send-all union-link-list :joint) :joint-angle))
          (c0 (unless (send self :parent) (send self :copy-worldcoords)))
	  dif-pos dif-rot
	  ;;
          (success t) 
          (fix-targets-av 
           (mapcar #'(lambda (l) (send-all l :joint :joint-angle))
                   (mapcar #'(lambda (l) (send self :link-list l)) fix-targets)))
	  (old-analysis-level (send-all union-link-list :analysis-level))
          ik-args)       
     (send-all union-link-list :analysis-level :coords)
     ;; argument check
     (when (or (null link-list) (null move-target))
       (warn ";; ERROR: :link-list or :move-target required~%")
       (return-from :inverse-kinematics t))
     (if (and (null translation-axis) (null rotation-axis))
         (return-from :inverse-kinematics t))
     ;; atom -> list
     (when debug-view
       (if (atom debug-view) (setq debug-view (list debug-view)))
       (when (memq :no-clear debug-view)
	 (push :no-clear debug-view))
       (when (memq :no-flush debug-view)
	 (push :no-flush debug-view)))
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom target-coords) (setq target-coords (list target-coords)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
     (if (atom thre) (setq thre (list thre)))
     (if (atom rthre) (setq rthre (list rthre)))
     ;; argument check
     (unless (= (length translation-axis) (length rotation-axis)
                (length move-target) (length link-list) (length target-coords))
       (warn ";; ERROR: list length differ : translation-axis ~A rotation-axis ~A move-target ~A link-list ~A target-coords ~A~%"
             (length translation-axis) (length rotation-axis)
             (length move-target) (length link-list) (length target-coords))
       (return-from :inverse-kinematics t))
     (setq ik-args (nconc (send* self :inverse-kinematics-args
				      :translation-axis translation-axis
				      :rotation-axis rotation-axis
				      :union-link-list union-link-list args) args))
     
     ;; inverse kinematics loop
     (while (< (incf loop) stop)
       (setq success (send* self :inverse-kinematics-loop
                           target-coords
                           :stop stop :loop loop
                           :rotation-axis rotation-axis
                           :translation-axis translation-axis
                           :move-target move-target
                           :union-link-list union-link-list
                           :thre thre :rthre rthre
                           :debug-view debug-view
                           :ik-args ik-args args))
       (case success
             (:ik-failed
              (setq success nil) (return nil))
             (:ik-succeed
              (setq success t) (return nil)))
       )
        ;; update difference
     (dotimes (i (length move-target))
       (let ((move-target (elt move-target i))
             (target-coords (elt target-coords i))
             (rotation-axis (elt rotation-axis i))
             (translation-axis (elt translation-axis i))
             (thre (elt thre i)) (rthre (elt rthre i)))
         (setq dif-pos (send move-target :difference-position target-coords :translation-axis translation-axis)
               dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
         (setq success (and success
                            (if translation-axis (< (norm dif-pos) thre) t)
                            (if rotation-axis (< (norm dif-rot) rthre) t)))
         ))
     (mapcar #'(lambda (l a) (send l :analysis-level a)) union-link-list old-analysis-level)
     ;; check solved or not
     (if success
         (send self :angle-vector)
       (progn
	 (when warnp
	   (warn ";; inverse-kinematics failed.~%")
           (dotimes (i (length move-target))
             (let ((move-target (elt move-target i))
                   (target-coords (elt target-coords i))
                   (rotation-axis (elt rotation-axis i))
                   (translation-axis (elt translation-axis i))
                   (thre (elt thre i)) (rthre (elt rthre i)))
               (setq dif-pos (send move-target :difference-position target-coords :translation-axis translation-axis)
                     dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
               (warn ";; dif-pos : ~a/(~a/~a)~%" dif-pos (norm dif-pos) thre)
               (warn ";; dif-rot : ~a/(~a/~a)~%" dif-rot (norm dif-rot) rthre)
               ))
           (warn ";;  coords : ~a~%" 
                 (send (let ((p self)) (while (send p :parent) (setq p (send p :parent))) p) :worldcoords))
	   (warn ";;  angles : ~a~%" av0)
	   (warn ";;    args : ~a~%" (append (list target-coords) args))
	   )
	 (mapc #'(lambda (j a) (send j :joint-angle a)) (send-all union-link-list :joint) av0)
         (if fix-targets-av
             (mapcar #'(lambda (ll aa) (mapc #'(lambda (j a) (send j :joint-angle a)) (send-all ll :joint) aa))
                     (mapcar #'(lambda (l) (send self :link-list l)) fix-targets)
                     fix-targets-av))
         (if c0 (send self :newcoords c0))
	 nil))
     ))
  (:calc-vel-from-dif-pos-rot
   (dif-pos dif-rot translation-axis rotation-axis
    &rest args
    &key (p-limit 100.0) (r-limit 0.5)
         (tmp-v0 (instantiate float-vector 0))
         (tmp-v1 (instantiate float-vector 1))
         (tmp-v2 (instantiate float-vector 2))
         (tmp-v3 (instantiate float-vector 3))
         (tmp-dim (instantiate float-vector (send self :calc-target-axis-dimension rotation-axis translation-axis)))
    &allow-other-keys)
   (let (vel-p vel-r)
     ;; limitation and unit system contvert
     ;; pos
     (if (> (norm dif-pos) p-limit)
	 (setq dif-pos (scale p-limit (normalize-vector dif-pos tmp-v3) tmp-v3)))
     (setq dif-pos (scale 0.001 dif-pos tmp-v3)) ;; scale [mm] -> [m]
     (setq vel-p (calc-dif-with-axis dif-pos translation-axis
				     tmp-v0 tmp-v1 tmp-v2))
     (dotimes (i (length vel-p)) (setf (elt tmp-dim i) (elt vel-p i)))
     ;; rot
     (if (> (norm dif-rot) r-limit)
	 (setq dif-rot (scale r-limit (normalize-vector dif-rot tmp-v3) tmp-v3)))
     (setq vel-r (calc-dif-with-axis dif-rot rotation-axis
				     tmp-v0 tmp-v1 tmp-v2))
     (dotimes (i (length vel-r)) (setf (elt tmp-dim (+ i (length vel-p))) (elt vel-r i)))
     tmp-dim))
  (:calc-torque
   (&key (dt 0.005) ;; dt [s]
         (debug-view nil) (av (send self :angle-vector)))
   (let ((torque-vector (instantiate float-vector (length joint-list)))
         pdq pl)
     ;; joint-angle update
     (dotimes (i (length joint-list))
       (setq pl (send (elt joint-list i) :child-link))
       (setq pdq (send pl :joint-velocity))
       (send pl :joint-velocity
             (deg2rad (/ (- (elt av i) (send (elt joint-list i) :joint-angle)) dt))) ;; [rad/s]
       (send pl :joint-acceleration (/ (- (send pl :joint-velocity) pdq) dt)) ;; [rad/s^2]
       (send (elt joint-list i) :joint-angle (elt av i))
       )
     
     ;; recursive calculation from root-link
     (send (car (send self :links)) :forward-all-kinematics
           :debug-view debug-view)
     (send (car (send self :links)) :inverse-dynamics
           :debug-view debug-view :dt dt)
     (dotimes (i (length torque-vector))
       (setf (elt torque-vector i) (send (send (elt joint-list i) :child-link) :joint-torque)))
     torque-vector))
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
  (let (j jang jmax jmin nang wang (e (deg2rad 1)) (k 0) (kk 0) r)
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
        (setq nang 0)
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
;;; Revision 1.107  2009-12-27 08:15:04  ueda
;;; make :draw-collision-debug-view method. because debug drawing in :inverse-kinematics-loop is too long and ugly.
;;;
;;; Revision 1.106  2009/12/09 09:39:37  nozawa
;;; add keyword argument to :calc-jacobian-from-link-list in :inverse-kinematics-loop ;; translation-axis, rotation-axis and move-target
;;;
;;; Revision 1.105  2009/11/28 03:15:22  nozawa
;;; use copy-object for weight in :move-joints-avoidance
;;;
;;; Revision 1.104  2009/11/27 12:34:14  k-okada
;;; weight does not have side effect on :inverse-kinematics, but :inverse-kinmatics-loop changes weight
;;;
;;; Revision 1.103  2009/11/27 11:25:58  k-okada
;;; revert to 1.101
;;;
;;; Revision 1.102  2009/11/25 11:04:56  nozawa
;;; fix bug in calcation of weight ;; weight was overwrited in :move-joints-avoidance
;;;
;;; Revision 1.101  2009/11/25 11:01:03  nozawa
;;; remove wmat <- unused in :move-joints-avoidance
;;;
;;; Revision 1.100  2009/11/18 04:20:47  k-okada
;;; joint-angle-limit-nspace, for wasit-y joint, neutral-angle is always 0
;;;
;;; Revision 1.99  2009/11/10 13:29:53  k-okada
;;; draw collision-avoidance-result in inverse-kinematics-loop for better display
;;;
;;; Revision 1.98  2009/11/10 13:05:45  k-okada
;;; fix :no-clear, no-flush rules
;;;
;;; Revision 1.97  2009/11/10 05:53:38  k-okada
;;; add angle-speed-collision-blending to debug-view message
;;;
;;; Revision 1.96  2009/11/10 05:17:33  k-okada
;;; add debug in :collision-avoidance
;;;
;;; Revision 1.95  2009/11/08 07:52:38  k-okada
;;; fix :collision-avoidance-link-pair-from-link-list, if two links have same parent, then never collide???
;;;
;;; Revision 1.94  2009/11/02 11:52:50  eus
;;; previous commit is mistake
;;;
;;; Revision 1.93  2009/11/02 11:40:34  ueda
;;; if :debug-view has :no-flush, does not flush in draw-objects of inverse-kinematics-loop
;;;
;;; Revision 1.92  2009/10/27 07:44:44  eus
;;; add :calc-torque to cascaded-link (nozawa)
;;;
;;; Revision 1.91  2009/10/27 02:49:30  nozawa
;;; add dynamics parameters to bodyset-link class and create new methods for dynamics calculation (:inverse-dynamics, :forward-all-kinematics...)
;;;
;;; Revision 1.90  2009/10/26 04:35:34  nozawa
;;; fix bug in :collision-avoidance
;;;
;;; Revision 1.89  2009/10/26 02:38:23  eus
;;; fix bug in :collision-avoidance(nozawa)
;;;
;;; Revision 1.88  2009/10/26 01:38:17  nozawa
;;; add debug-view to :no-clear in :inverse-kinematics-loop and fix default argument of thre and rthre
;;;
;;; Revision 1.87  2009/10/23 10:57:53  k-okada
;;; need-clear need-flush
;;;
;;; Revision 1.86  2009/10/06 13:32:48  nozawa
;;; adapt thre and rthre to list or atom in :inverse-kinematics and :inverse-kinematics-loop
;;;
;;; Revision 1.85  2009/10/06 13:19:27  nozawa
;;; fix calculation of col in :calc-jacobian-from-link-list
;;;
;;; Revision 1.84  2009/09/30 15:58:01  nozawa
;;; use args and ik-args for :calc-vel-from-dif-pos-rot
;;;
;;; Revision 1.83  2009/09/30 13:47:09  k-okada
;;; add angle-speed-limit
;;;
;;; Revision 1.82  2009/09/30 10:18:17  nozawa
;;; commit for dual-arm ik : main update are as follows : list of move-target,target-coords ... can be used in inverse-kinematics or so. :move-joints-avoidance requires jacobian
;;;
;;; Revision 1.81  2009/09/30 10:12:10  nozawa
;;; integrate vel-pos and vel-rot -> vel and add :calc-vel-from-dif-pos-rot
;;;
;;; Revision 1.80  2009/09/29 02:17:49  k-okada
;;; use args and ik-args in :inverse-kinematics-loop
;;;
;;; Revision 1.79  2009/09/25 09:54:22  k-okada
;;; move-target does not used in move-joint-avoidance
;;;
;;; Revision 1.78  2009/09/25 09:33:33  k-okada
;;; add default value to stop and loop in :inversekinematics-loop
;;;
;;; Revision 1.77  2009/09/24 11:26:59  nozawa
;;; add rotation-axis to ik-args and inverse-kinematics-loop and fix "check solved or not"
;;;
;;; Revision 1.76  2009/09/24 10:07:53  k-okada
;;; move-avoidance-* taes local difference
;;;
;;; Revision 1.75  2009/09/24 10:02:20  k-okada
;;; do not display error when pair-list is null
;;;
;;; Revision 1.74  2009/09/20 06:07:06  k-okada
;;; fix previsous commit
;;;
;;; Revision 1.73  2009/09/20 04:33:11  k-okada
;;; add :inverse-kinmatics-loop
;;;
;;; Revision 1.72  2009/09/20 04:29:22  k-okada
;;; add :inverse-kinematics-loop
;;;
;;; Revision 1.71  2009/09/20 04:28:08  k-okada
;;; add :debug to :collision-avoidance-link-pair-from-link-list
;;;
;;; Revision 1.70  2009/09/20 04:05:50  k-okada
;;; fix typo :inverse-kenematics-args -> :inverse-kinematics-args
;;;
;;; Revision 1.69  2009/09/20 03:38:22  k-okada
;;; add :inverse-kinematics-args
;;;
;;; Revision 1.68  2009/09/20 03:21:00  k-okada
;;; fix: if avoid-collision-distnace 0, do not call :collision-avoidance
;;;
;;; Revision 1.67  2009/09/20 03:13:43  k-okada
;;; print x when :debug
;;;
;;; Revision 1.66  2009/09/20 03:12:53  k-okada
;;; if avoid-collision-distnace 0, do not call :collision-avoidance
;;;
;;; Revision 1.65  2009/09/10 15:08:21  nozawa
;;; fix child-link when child-reverse (in :calc-jacobian-from-link-list)
;;;
;;; Revision 1.64  2009/09/10 13:27:00  k-okada
;;; fix previous commit / support :translation-axis for dif-pos
;;;
;;; Revision 1.63  2009/09/10 13:25:26  k-okada
;;; support :translation-axis for dif-pos
;;;
;;; Revision 1.62  2009/09/06 18:27:38  eus
;;; change variable r in joint-angle-limit-weight : global -> local (nozawa)
;;;
;;; Revision 1.61  2009/09/01 12:53:15  k-okada
;;; support :obstacles
;;;
;;; Revision 1.60  2009/09/01 12:18:52  k-okada
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
