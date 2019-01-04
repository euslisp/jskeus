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
(require :pqp)

(defclass joint
  :super propertied-object
  :slots (parent-link child-link joint-angle min-angle max-angle default-coords
		      joint-velocity joint-acceleration joint-torque
		      max-joint-velocity max-joint-torque
                      joint-min-max-table joint-min-max-target))

(defmethod joint
  (:init (&key (name (intern (format nil "joint~A" (sys::address self)) "KEYWORD"))
               ((:child-link clink)) ((:parent-link plink))
	       (min -90) (max 90) ;; use deg/mm unit
	       ((:max-joint-velocity mjv)) ;; use SI(m/s,rad/s) unit
               ((:max-joint-torque mjt)) ;; use SI(N,Nm) unit
	       ((:joint-min-max-table mm-table))
               ((:joint-min-max-target mm-target))
	       &allow-other-keys)
         "abstract class of joint, users need to use rotational-joint, linear-joint, sphere-joint, 6dof-joint, wheel-joint or omniwheel-joint.
          use :parent-link/:child-link for specifying links that this joint connect to and :min/:min for range of joint angle in degree."
	 (send self :name name)
	 (setq parent-link plink child-link clink
	       min-angle min max-angle max)
	 (if (or (and (float-vector-p mjv)
		      (not (v> mjv (instantiate float-vector (length mjv)))))
		 (and (not (float-vector-p mjv)) (< mjv 0.0)))
	     (error "[joint ~A] warning negative max-joint-velocity value ~A~%" name mjv))
	 (if (or (and (float-vector-p mjt)
		      (not (v> mjt (instantiate float-vector (length mjt)))))
		 (and (not (float-vector-p mjt)) (< mjt 0.0)))
	     (error "[joint ~A] warning negative max-joint-torque value ~A~%" name mjt))
	 (send self :max-joint-velocity mjv)
	 (send self :max-joint-torque mjt)
	 (send self :joint-min-max-table mm-table)
	 (send self :joint-min-max-target mm-target)
	 (setq default-coords (send child-link :copy-coords))
	 self)
  (:min-angle (&optional v)
   "If v is set, it updates min-angle of this instance. :min-angle returns minimal angle of this joint in degree."
   (if v (setq min-angle v)) min-angle)
  (:max-angle (&optional v)
   "If v is set, it updates max-angle of this instance. :max-angle returns maximum angle of this joint in degree."
   (if v (setq max-angle v)) max-angle)
  (:parent-link (&rest args)
   "Returns parent link of this joint. if any arguments is set, it is passed to the parent-link."
   (user::forward-message-to parent-link args))
  (:child-link (&rest args)
   "Returns child link of this joint. if any arguments is set, it is passed to the child-link."
   (user::forward-message-to child-link args))
  (:calc-dav-gain (dav i periodic-time) (warn "subclass's respoinsibility (send ~s :calc-dav-gain)~%" self))
  (:joint-dof ()
   "Returns Degree of Freedom of this joint."
   (warn "subclass's respoinsibility (send ~s :joint-dof)~%" self))
  (:speed-to-angle (&rest args)
   "Returns values in deg/mm unit of input value in SI(rad/m) unit."
   (warn "subclass's respoinsibility (send ~s :speed-to-angle)~%" self))
  (:angle-to-speed (&rest args)
   "Returns values in SI(rad/m) unit of input value in deg/mm unit."
   (warn "subclass's respoinsibility (send ~s :angle-to-speed)~%" self))
  (:calc-jacobian (&rest args) (warn "subclass's respoinsibility (send ~s :calc-jacobian)~%" self))
  (:joint-velocity (&optional jv)
   "If jv is set, it updates joint-velocity of this instance. :joint-velocity returns velocity of this joint in SI(m/s, rad/s) unit."
   (if jv (setq joint-velocity jv) joint-velocity))
  (:joint-acceleration (&optional ja)
   "If ja is set, it updates joint-acceleration of this instance. :joint-acceleration returns acceleration of this joint in SI(m/$s^2$, rad/$s^2$) unit."
   (if ja (setq joint-acceleration ja) joint-acceleration))
  (:joint-torque (&optional jt)
   "If jt is set, it updates joint-torque of this instance. :joint-torque returns torque of this joint in SI(N, Nm) unit."
   (if jt (setq joint-torque jt) joint-torque))
  (:max-joint-velocity (&optional mjv)
   "If mjv is set, it updates min-joint-velocity of this instance. :min-joint-velocity returns velocity of this joint in SI(m/s, rad/s)  unit."
   (if mjv (setq max-joint-velocity mjv) max-joint-velocity))
  (:max-joint-torque (&optional mjt)
   "If mjt is set, it updates min-joint-torque of this instance. :min-joint-torque returns velocity of this joint in SI(N, Nm) unit."
   (if mjt (setq max-joint-torque mjt) max-joint-torque))
  (:joint-min-max-table (&optional mm-table) (if mm-table (setq joint-min-max-table mm-table) joint-min-max-table))
  (:joint-min-max-target (&optional mm-target) (if mm-target (setq joint-min-max-target mm-target) joint-min-max-target))
  (:joint-min-max-table-angle-interpolate
   (target-angle min-or-max)
   (let* ((int-target-angle (floor target-angle)) ;; integer value convered as a matrix table key
          (target-range (list int-target-angle (1+ int-target-angle))) ;; range of target-joint's angle, e.g., (car target-range) <= target-ang <= (cadr target-range)
          (joint-range (mapcar #'(lambda (x)
                                   (aref
                                    joint-min-max-table
                                    (case min-or-max
                                          (:min-angle 1)
                                          (:max-angle 2)
                                          (t ))
                                    (round
                                     (-
                                      (min (max (aref joint-min-max-table 0 0) x)
                                           (aref joint-min-max-table 0 (- (array-dimension joint-min-max-table 1) 1)))
                                      (aref joint-min-max-table 0 0)))
                                    ))
                               target-range)) ;; range of joint-angle based on target-range
          (tmp-ratio (- target-angle (float int-target-angle))))
     ;; calc min max
     ;; interpolated joint-angle based on joint-range
     ;; e.g., (car joint-range) <= min[max]-angle <= (cadr joint-range) or (cadr joint-range) <= min[max]-angle <= (car joint-range) 
     (+ (* (car joint-range) (- 1 tmp-ratio)) (* (cadr joint-range) tmp-ratio))))
  (:joint-min-max-table-min-angle
   (&optional (target-angle (send joint-min-max-target :joint-angle)))
   (send self :joint-min-max-table-angle-interpolate target-angle :min-angle))
  (:joint-min-max-table-max-angle
   (&optional (target-angle (send joint-min-max-target :joint-angle)))
   (send self :joint-min-max-table-angle-interpolate target-angle :max-angle))
  )

(defun calc-jacobian-default-rotate-vector
  (paxis world-default-coords child-reverse transform-coords tmp-v3 tmp-m33)
  (setq tmp-v3 (scale (if child-reverse -1.0 1.0)
                      (normalize-vector (send world-default-coords :rotate-vector paxis tmp-v3) tmp-v3) tmp-v3))
  (transform (transpose (send transform-coords :worldrot) tmp-m33) tmp-v3 tmp-v3))

(defun calc-jacobian-rotational
  (fik row column joint paxis child-link world-default-coords child-reverse
       move-target transform-coords rotation-axis translation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
  (let* ((j-rot (calc-jacobian-default-rotate-vector ;; copied to tmp-v3
                 paxis world-default-coords child-reverse transform-coords tmp-v3 tmp-m33))
         (p-diff (scale 1e-3 ;; scale [mm] -> [m], copied to tmp-v3a
                        (transform (transpose (send transform-coords :worldrot) tmp-m33)
                                   (v- (send move-target :worldpos) (send child-link :worldpos) tmp-v3a)
                                   tmp-v3a) tmp-v3a))
         (j-trs (v* j-rot p-diff tmp-v3b))) ;; copied to tmp-v3b
    (setq j-trs (calc-dif-with-axis j-trs translation-axis tmp-v0 tmp-v1 tmp-v2)) ;; copied to tmp-v3b, tmp-v0, tmp-v1 or tmp-v2
    (dotimes (j (length j-trs)) (setf (aref fik (+ j row) column) (elt j-trs j)))
    (setq j-rot (calc-dif-with-axis j-rot rotation-axis tmp-v0 tmp-v1 tmp-v2)) ;; copied to tmp-v3, tmp-v0, tmp-v1 or tmp-v2
    (dotimes (j (length j-rot)) (setf (aref fik (+ j row (length j-trs)) column) (elt j-rot j)))
    ))

(defun calc-jacobian-linear
  (fik row column joint paxis child-link world-default-coords child-reverse
       move-target transform-coords rotation-axis translation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
  (let* ((j-trs (calc-jacobian-default-rotate-vector ;; copied to tmp-v3
                 paxis world-default-coords child-reverse transform-coords tmp-v3 tmp-m33))
         (j-rot #f(0 0 0)))
    (setq j-trs (calc-dif-with-axis j-trs translation-axis tmp-v0 tmp-v1 tmp-v2)) ;; copied to tmp-v3, tmp-v0, tmp-v1 or tmp-v2
    (dotimes (j (length j-trs)) (setf (aref fik (+ j row) column) (elt j-trs j)))
    (setq j-rot (calc-dif-with-axis j-rot rotation-axis tmp-v0 tmp-v1 tmp-v2)) ;; copied to j-rot, tmp-v0, tmp-v1 or tmp-v2
    (dotimes (j (length j-rot)) (setf (aref fik (+ j row (length j-trs)) column) (elt j-rot j)))
    ))

;; calculate angle-speed gain used in :move-joints method
;; calc-angle-speed-gain-scalar -> for rotational-joint and linear-joint
(defun calc-angle-speed-gain-scalar
  (j dav i periodic-time)
  (let ((dav-gain
	 (abs (/ (send j :max-joint-velocity)
		 (/ (elt dav i) periodic-time)))))
    (if (< dav-gain 1.0) dav-gain 1.0)))

;; calc-angle-speed-gain-vector -> for wheel-joint, omniwheel-joint, sphere-joint and 6dof-joint
(defun calc-angle-speed-gain-vector
  (j dav i periodic-time)
  (let ((dav-gain 1.0))
    (dotimes (ii (send j :joint-dof))
      (let ((tmp-gain
	     (abs (/ (elt (send j :max-joint-velocity) ii)
		     (/ (elt dav (+ ii i)) periodic-time)))))
	(if (< tmp-gain dav-gain) (setq dav-gain tmp-gain))))
    dav-gain))

(defclass rotational-joint
  :super joint
  :slots (axis))

(defmethod rotational-joint
  (:init (&rest args
          &key ((:axis ax) :z)
               ((:max-joint-velocity mjv) 5) ;; [rad/s]
               ((:max-joint-torque mjt) 100) ;; [Nm]
          &allow-other-keys)
         "create instance of rotational-joint. :axis is either (:x, :y, :z) or vector. :min-angle and :max-angle takes in radius, but velocity and torque are given in SI units."
	 (setq axis ax)
	 (setq joint-angle 0.0)
	 (send-super* :init
                      :max-joint-velocity mjv
                      :max-joint-torque mjt
                      args)
	 ;; set default value
	 (if (null min-angle) (setq min-angle -90.0))
	 (if (null max-angle) (setq max-angle (+ 180.0 min-angle)))
	 (send self :joint-velocity 0.0) ;; [rad/s]
	 (send self :joint-acceleration 0.0) ;; [rad/s^2]
	 (send self :joint-torque 0.0) ;; [Nm]
	 self)
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   "Return joint-angle if v is not set, if v is given, set joint angle. v is rotational value in degree."
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
	      (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate max-angle(~A)~%" self v max-angle))
	      (setq v max-angle)))
       (cond ((< v min-angle)
	      (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate min-angle(~A)~%" self v min-angle))
	      (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  (:joint-dof () "Returns DOF of rotational joint, 1." 1)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-scalar self dav i periodic-time))
  (:speed-to-angle (v) "Returns degree of given input in radian" (rad2deg v))
  (:angle-to-speed (v) "Returns radian of given input in degree" (deg2rad v))
  (:calc-jacobian (&rest args) (apply #'calc-jacobian-rotational args))
  )

(defclass linear-joint
  :super joint
  :slots (axis))

(defmethod linear-joint
  (:init (&rest args
          &key ((:axis ax) :z)
          ((:max-joint-velocity mjv) (/ pi 4)) ;; [m/s]
          ((:max-joint-torque mjt) 100) ;; [N]
          &allow-other-keys)
         "Create instance of linear-joint. :axis is either (:x, :y, :z) or vector. :min-angle and :max-angle takes in [mm], but velocity and torque are given in SI units."
	 (setq axis 
	   (if (float-vector-p ax) 
	       ax
	     (case ax (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
		   (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
		   (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1)))))
	 (setq joint-angle 0.0)
	 (send-super* :init
                      :max-joint-velocity mjv
                      :max-joint-torque mjt
                      args)
	 ;; set default value
	 (if (null min-angle) (setq min-angle -90.0))
	 (if (null max-angle) (setq max-angle  90.0))
	 (send self :joint-velocity 0.0) ;; [m/s]
	 (send self :joint-acceleration 0.0) ;; [m/s^2]
	 (send self :joint-torque 0.0) ;; [N]
	 self)
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   "return joint-angle if v is not set, if v is given, set joint angle. v is linear value in [mm]."
   (let ()
     (when v
       (if relative (setq v (+ v joint-angle)))
       (when (> v max-angle)
	 (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate max-angle(~A)~%" self v max-angle))
	 (setq v max-angle))
       (when (< v min-angle)
	 (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate min-angle(~A)~%" self v min-angle))
	 (setq v min-angle))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :translate (scale joint-angle axis)))
     joint-angle))
  (:joint-dof () "Returns DOF of linear joint, 1." 1)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-scalar self dav i periodic-time))
  (:speed-to-angle (v) "Returns [mm] of given input in [m]" (* 1000 v))
  (:angle-to-speed (v) "Returns [m] of given input in [mm]" (* 0.001 v))
  (:calc-jacobian (&rest args) (apply #'calc-jacobian-linear args))
  )

(defclass wheel-joint
  :super joint
  :slots (axis))

(defmethod wheel-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf*))
                (max (float-vector *inf* *inf*))
		((:max-joint-velocity mjv)
		 (float-vector (/ 0.08 0.05) ;; [m/s]
			       (/ pi 4))) ;; [rad/s]
                ((:max-joint-torque mjt)
		 (float-vector 100 100)) ;; [N] [Nm]
                &allow-other-keys)
         "Create instance of wheel-joint."
	 (setq joint-angle (float-vector 0 0))
         (setq axis (list #f(1 0 0) :z))
	 (send-super* :init :min min :max max
                      :max-joint-velocity mjv
                      :max-joint-torque mjt
                      args)
	 ;; set default value
	 self)
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   "return joint-angle if v is not set, if v is given, set joint angle. v is  joint-angle vector, which is  (float-vector translation-x[mm] rotation-z[deg])"
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
  (:joint-dof () "Returns DOF of linear joint, 2." 2)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-vector self dav i periodic-time))
  (:speed-to-angle (dv) "Returns [mm/deg] of given input in SI unit [m/rad]"
                   (float-vector (* 1000 (elt dv 0))
                                 (rad2deg (elt dv 1))))
  (:angle-to-speed (dv) "Returns SI unit [m/rad] of given input in [mm/deg]"
                   (float-vector (* 0.001 (elt dv 0))
                                 (deg2rad (elt dv 1))))
  (:calc-jacobian
   (fik row column joint paxis child-link world-default-coords child-reverse
	move-target transform-coords rotation-axis translation-axis
        tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (calc-jacobian-linear
      fik row column joint #f(1 0 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 1) joint #f(0 0 1) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  )

(defclass omniwheel-joint
  :super joint
  :slots (axis))

(defmethod omniwheel-joint
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf*))
		((:max-joint-velocity mjv)
		 (float-vector (/ 0.08 0.05) (/ 0.08 0.05) ;; [m/s]
			       (/ pi 4))) ;; [rad/s]
                ((:max-joint-torque mjt)
		 (float-vector 100 100 100)) ;; [N] [N] [Nm]
                &allow-other-keys)
         "create instance of omniwheel-joint."
       (setq joint-angle (float-vector 0 0 0))
       (setq axis (list #f(1 0 0) #f(0 1 0) :z))
       (send-super* :init :min min :max max
                    :max-joint-velocity mjv
                    :max-joint-torque mjt
                    args)
       ;; set default value
       self)
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   "return joint-angle if v is not set, if v is given, set joint angle. v is joint-angle vector, which is (float-vector translation-x[mm] translation-y[mm] rotation-z[deg])"
   (when v
     (if relative (setq v (v+ v joint-angle)))
     (setq joint-angle (vmin (vmax v min-angle) max-angle))
     (send child-link :replace-coords default-coords)
     (send child-link :translate (float-vector (elt joint-angle 0) (elt joint-angle 1) 0))
     (send child-link :rotate (deg2rad (elt joint-angle 2)) :z))
   joint-angle)
  (:joint-dof () "Returns DOF of linear joint, 3." 3)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-vector self dav i periodic-time))
  (:speed-to-angle (dv) "Returns [mm/deg] of given input in SI unit [m/rad]"
                   (float-vector (* 1000 (elt dv 0))
                                 (* 1000 (elt dv 1))
                                 (rad2deg (elt dv 2))))
  (:angle-to-speed (dv) "Returns SI unit [m/rad] of given input in [mm/deg]"
                   (float-vector (* 0.001 (elt dv 0))
                                 (* 0.001 (elt dv 1))
                                 (deg2rad (elt dv 2))))
  (:calc-jacobian
   (fik row column joint paxis child-link world-default-coords child-reverse
	move-target transform-coords rotation-axis translation-axis
        tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (calc-jacobian-linear
      fik row column joint #f(1 0 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-linear
      fik row (+ column 1) joint #f(0 1 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 2) joint #f(0 0 1) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  )

(defclass sphere-joint
  :super joint
  :slots (axis))

(defmethod sphere-joint
  (:init (&rest args
		&key
		;; min/max are defined as a region of anguler velocity
		(min (float-vector *-inf* *-inf* *-inf*))
		(max (float-vector *inf* *inf* *inf*))
		((:max-joint-velocity mjv)
		 (float-vector (/ pi 4) (/ pi 4) (/ pi 4))) ;; [rad/s]
                ((:max-joint-torque mjt)
                 (float-vector 100 100 100)) ;; [Nm] [Nm] [Nm]
		&allow-other-keys)
         "Create instance of sphere-joint. min/max are defiend as a region of angular velocity in degree."
    (setq axis (list :x :y :z))
    (send-super* :init :min min :max max
                 :max-joint-velocity mjv
                 :max-joint-torque mjt
                 args)
    ;; joint-angle is defined as anguler velocity equivalent to orthogonal orientation matrix
    (setq joint-angle (float-vector 0 0 0)) ;; [deg]
    self)
  (:joint-angle
    (&optional v &key relative &allow-other-keys)
    "return joint-angle if v is not set, if v is given, set joint angle.
     v is joint-angle vector [deg] by axis-angle representation, i.e (scale rotation-angle-from-default-coords[deg] axis-unit-vector)"
    (when v
      ;; Update joint-angle
      (cond
       (relative
        ;; if relative t
        ;; v is axis-angle difference vector calculated from difference-rotation or jacobian
        (let ((vec (map float-vector #'deg2rad v)))
          (unless (eps= (norm vec) 0.0 1e-20)
            ;; If vec is large, update joint-angle
            (let ((drot (matrix-exponent (map float-vector #'deg2rad joint-angle))))
              (m* (rotation-matrix (norm vec) (normalize-vector vec)) drot drot)
              (setq joint-angle (map float-vector #'rad2deg (matrix-log drot))))
            )))
       (t
        ;; if relative nil
        ;; v is axis-angle equivalent to orthogonal orientation matrix calculated from (send (send world-default-coords :transformation child-link) :worldrot)
        (setq joint-angle v)))
      ;; min max check
      (setq joint-angle (vmin (vmax joint-angle min-angle) max-angle))
      ;; update child-link
      (send child-link :replace-coords default-coords)
      (send child-link :transform
            (make-coords :rot (matrix-exponent (map float-vector #'deg2rad joint-angle)))))
    joint-angle)
  (:joint-angle-rpy ;; v and return-value -> [deg] and (float-vector yaw roll pitch)
    (&optional v &key relative)
    "Return joint-angle if v is not set, if v is given, set joint-angle vector by RPY representation, i.e. (float-vector yaw[deg] roll[deg] pitch[deg])"
    (when v
      (let ((ja (if relative
		    (v+ (map float-vector #'deg2rad v)
			(coerce (car (rpy-angle (matrix-exponent (map float-vector #'deg2rad joint-angle))))
				float-vector))
		  (map float-vector #'deg2rad v))))
	(send self :joint-angle (map cons #'rad2deg (matrix-log (rpy-matrix (elt ja 0) (elt ja 1) (elt ja 2)))))))
    (map float-vector #'rad2deg (car (rpy-angle (matrix-exponent (map float-vector #'deg2rad joint-angle))))))
  (:joint-dof () "Returns DOF of linear joint, 3." 3)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-vector self dav i periodic-time))
  (:speed-to-angle (dv) "Returns degree of given input in radian"
                   (float-vector (rad2deg (elt dv 0))
                                 (rad2deg (elt dv 1))
                                 (rad2deg (elt dv 2))))
  (:angle-to-speed (dv) "Returns radian of given input in degree"
                   (float-vector (deg2rad (elt dv 0))
                                 (deg2rad (elt dv 1))
                                 (deg2rad (elt dv 2))))
  (:calc-jacobian
   (fik row column joint paxis child-link world-default-coords child-reverse
	move-target transform-coords rotation-axis translation-axis
        tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (calc-jacobian-rotational
      fik row column joint #f(1 0 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 1) joint #f(0 1 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 2) joint #f(0 0 1) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  ;;
  (:joint-euler-angle
   (&key (axis-order '(:z :y :x)) ((:child-rot  m) (send child-link :rot)))
   "Return joint-angle if v is not set, if v is given, set joint-angle vector by euler representation."
   (let ()
     (map cons #'rad2deg
	  (case (length axis-order)
	    (1
	     (case (elt axis-order 0)
	       (:y
		(list (atan2 (- (aref m 2 0)) (aref m 2 2))))
	       (:x
		(list (atan2 (aref m 2 1) (aref m 2 2))))
	       (:z
		(list (atan2 (aref m 1 0) (aref m 1 1))))))
	    (2
	     (case (elt axis-order 0)
	       (:y
		(case (elt axis-order 1)
		  (:x (list (atan2 (- (aref m 2 0)) (aref m 0 0))
			    (atan2 (- (aref m 1 2)) (aref m 1 1))))
		  (:z (list (atan2 (aref m 0 2) (aref m 2 2))
			    (atan2 (aref m 1 0) (aref m 1 1))))
		  (:y (list 0 (atan2 (aref m 0 2) (aref m 0 0))))))
		(:x
		 (case (elt axis-order 1)
		  (:y (list (atan2 (aref m 2 1) (aref m 1 1))
			    (atan2 (aref m 0 2) (aref m 0 0))))
		  (:z (list (atan2 (- (aref m 1 2)) (aref m 2 2))
			    (atan2 (- (aref m 0 1)) (aref m 0 0))))
		  (:x (list 0 (atan2 (aref m 2 1) (aref m 2 2))))))
		(:z
		 (case (elt axis-order 1)
		  (:y (list (atan2 (- (aref m 0 1)) (aref m 1 1))
			    (atan2 (- (aref m 2 0)) (aref m 2 2))))
		  (:x (list (atan2 (aref m 1 0) (aref m 0 0))
			    (atan2 (aref m 2 1) (aref m 2 2))))
		  (:z (list 0 (atan2 (aref m 1 0) (aref m 1 1))))))))
	    (3
	     (matrix-to-euler-angle m axis-order)))
	  )))
  )

(defclass 6dof-joint
  :super joint
  :slots (axis))

(defmethod 6dof-joint
  ;; translation (3dof) and rotation (3dof) -> 6dof
  ;;   about rotation component -> please see sphere-joint declaration
  (:init (&rest args
                &key
                (min (float-vector *-inf* *-inf* *-inf* *-inf* *-inf* *-inf*))
                (max (float-vector *inf* *inf* *inf* *inf* *inf* *inf*))
		((:max-joint-velocity mjv)
		 (float-vector (/ 0.08 0.05) (/ 0.08 0.05) (/ 0.08 0.05) ;; [m/s]
			       (/ pi 4) (/ pi 4) (/ pi 4))) ;; [rad/s]
                ((:max-joint-mjt mjt)
		 (float-vector 100 100 100 100 100 100)) ;; [N] [N] [N] [Nm] [Nm] [Nm]
                (absolute-p nil)
                &allow-other-keys)
         "Create instance of 6dof-joint."
    (setq axis (list #f(1 0 0) #f(0 1 0) #f(0 0 1) :x :y :z))
    (send-super* :init :min min :max max
                 :max-joint-velocity mjv
                 :max-joint-torque mjt
                 args)
    (if absolute-p
        (let* ((c (send parent-link :transformation child-link))
               (b (make-coords :pos (send default-coords :worldpos)))
               (p (send b :inverse-transform-vector (send c :worldpos)))
               (r (matrix-log (m* (transpose (send b :worldrot)) (send c :worldrot)))))
          (setq joint-angle
                (float-vector (elt p 0) (elt p 1) (elt p 2) ;; [mm]
                              (rad2deg (elt r 0)) (rad2deg (elt r 1)) (rad2deg (elt r 2)))) ;; [deg]
          (setq default-coords b))
      (setq joint-angle (float-vector 0 0 0 0 0 0))) ;; [mm] [deg]
    self)
  (:joint-angle
    (&optional v &key relative &allow-other-keys) ;; v [mm] [deg]
    "Return joint-angle if v is not set, if v is given, set joint angle vector, which is 6D vector of 3D translation[mm] and 3D rotation[deg], i.e. (find-if #'(lambda (x) (eq (send (car x) :name) 'sphere-joint)) (documentation :joint-angle))"
    (when v
      (let (joint-angle-pos joint-angle-rot)
        ;; Update joint-angle
        ;;   translation
        (setq joint-angle-pos (if relative
                                  (v+ (subseq joint-angle 0 3) (subseq v 0 3))
                                (subseq v 0 3)))
        ;;   rotation
        (cond
         (relative
          ;; if relative t
          ;; rotation part of v is axis-angle difference vector calculated from difference-rotation or jacobian
          (let ((vec (map float-vector #'deg2rad (subseq v 3 6))))
            (if (eps= (norm vec) 0.0 1e-20)
                (setq joint-angle-rot (subseq joint-angle 3 6))
              (let ((drot (matrix-exponent (map float-vector #'deg2rad (subseq joint-angle 3 6)))))
                (m* (rotation-matrix (norm vec) (normalize-vector vec)) drot drot)
                (setq joint-angle-rot (map float-vector #'rad2deg (matrix-log drot))))
              )))
         (t
          ;; if relative nil
          ;; rotation part of v is axis-angle equivalent to orthogonal orientation matrix calculated from (send (send world-default-coords :transfor
          (setq joint-angle-rot (subseq v 3 6))
          ))
        (dotimes (i 3)
          (setf (elt joint-angle i) (elt joint-angle-pos i))
          (setf (elt joint-angle (+ i 3)) (elt joint-angle-rot i)))
        ;; min max check
        (setq joint-angle (vmin (vmax joint-angle min-angle) max-angle))
        ;; update child-link
        (send child-link :replace-coords default-coords)
        (send child-link :transform
              (make-coords :rot (matrix-exponent (map float-vector #'deg2rad joint-angle-rot)) :pos joint-angle-pos))))
    joint-angle)
  (:joint-angle-rpy ;; v and return-value -> [deg] and (float-vector yaw roll pitch)
    (&optional v &key relative)
    "Return joint-angle if v is not set, if v is given, set joint angle. v is joint-angle vector, which is 6D vector of 3D translation[mm] and 3D rotation[deg], for rotation, please see (find-if #'(lambda (x) (eq (send (car x) :name) 'sphere-joint)) (documentation :joint-angle-rpy))"
    (when v
      (let ((ja (if relative
		    (v+ (map float-vector #'deg2rad (subseq v 3 6))
			(coerce (car (rpy-angle (matrix-exponent (map float-vector #'deg2rad (subseq joint-angle 3 6)))))
				float-vector))
		  (map float-vector #'deg2rad (subseq v 3 6)))))
	(send self :joint-angle (concatenate float-vector
					     (subseq v 0 3)
					     (map cons #'rad2deg (matrix-log (rpy-matrix (elt ja 0) (elt ja 1) (elt ja 2))))))))
    (concatenate float-vector
		 (subseq joint-angle 0 3)
		 (map float-vector #'rad2deg (car (rpy-angle (matrix-exponent (map float-vector #'deg2rad (subseq joint-angle 3 6))))))))
  (:joint-dof () "Returns DOF of linear joint, 6." 6)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-vector self dav i periodic-time))
  (:speed-to-angle (dv) "Returns [mm/deg] of given input in SI unit [m/rad]"
                   (float-vector (* 1000 (elt dv 0))
                                 (* 1000 (elt dv 1))
                                 (* 1000 (elt dv 2))
                                 (rad2deg (elt dv 3))
                                 (rad2deg (elt dv 4))
                                 (rad2deg (elt dv 5))))
  (:angle-to-speed (dv) "Returns SI unit [m/rad] of given input in [mm/deg]"
                   (float-vector (* 0.001 (elt dv 0))
                                 (* 0.001 (elt dv 1))
                                 (* 0.001 (elt dv 2))
                                 (deg2rad (elt dv 3))
                                 (deg2rad (elt dv 4))
                                 (deg2rad (elt dv 5))))
  (:calc-jacobian
   (fik row column joint paxis child-link world-default-coords child-reverse
	move-target transform-coords rotation-axis translation-axis
        tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
   (let ()
     (calc-jacobian-linear
      fik row column joint #f(1 0 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-linear
      fik row (+ column 1) joint #f(0 1 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-linear
      fik row (+ column 2) joint #f(0 0 1) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 3) joint #f(1 0 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 4) joint #f(0 1 0) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     (calc-jacobian-rotational
      fik row (+ column 5) joint #f(0 0 1) child-link world-default-coords child-reverse
      move-target transform-coords rotation-axis translation-axis
      tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
     ))
  )

;;;
;;;

(defclass bodyset-link
  :super bodyset
  :slots (joint parent-link child-links analysis-level default-coords
                weight acentroid inertia-tensor
                angular-velocity angular-acceleration
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
   "Create instance of bodyset-link."
   (setq analysis-level level)
   (setq weight w inertia-tensor i acentroid c)
   (send self :reset-dynamics)
   (send-super* :init coords args))
  (:worldcoords
   (&optional (level analysis-level))
   "Returns a coordinates object which represents this coord in the world by concatenating all the cascoords from the root to this coords."
   (case
    level
    (:coords (send-message self cascaded-coords :worldcoords))
    (t       (send-super :worldcoords)))
   )
  (:analysis-level
   (&optional v)
   "Change analysis level :coords only changes kinematics level and :body changes geometry too."
   (if v (setq analysis-level v)) analysis-level)
  (:weight
   (&optional w)
   "Returns a weight of the link. If w is given, set weight."
   (if w (setq weight w)) weight)
  (:centroid
   (&optional c)
   "Returns a centroid of the link. If c is given, set new centroid."
   (if c (setq acentroid c))
   (send (send self :worldcoords) :transform-vector acentroid))
  (:inertia-tensor
   (&optional i)
   "Returns a inertia tensor of the link. If c is given, set new intertia tensor."
   (if i (setq inertia-tensor i)) inertia-tensor)
  (:joint (&rest args) "Returns a joint associated with this link. If args is given, args are forward to the joint." (user::forward-message-to joint args))
  (:add-joint (j) "Set j as joint of this link" (setq joint j))
  (:del-joint () "Remove current joint of this link" (setq joint nil))

  (:parent-link () "Returns parent link" parent-link)
  (:child-links () "Returns child links" child-links)

  (:add-child-links (l) "Add l to child links" (unless (or (member l child-links) (not l))(push l child-links)))
  (:add-parent-link (l) "Set l as parent link" (setq parent-link l))

  (:del-child-link  (l) "Delete l from child links" (setq child-links (delete l child-links)))
  (:del-parent-link ()  "Delete parent link" (setq parent-link nil))

  (:default-coords (&optional c) (if c (setq default-coords c)) default-coords)
  )

(defclass cascaded-link
  :super cascaded-coords
  :slots (links joint-list bodies
		collision-avoidance-links
                end-coords-list
                ))

(defmethod cascaded-link
  (:init (&rest args
		&key name
		&allow-other-keys)
         "Create cascaded-link."
	 (send-super-lexpr :init args)
	 self)
  (:init-ending
   ()
   "This method is to called finalize the instantiation of the cascaded-link. This update bodies and child-link and parent link from joint-list"
   (setq bodies (flatten (send-all links :bodies)))
   (dolist (j joint-list)
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (send (send j :parent-link) :add-child-links (send j :child-link))
     )
   (send self :update-descendants)
   ;; Make default collision shape by pqp
   (dolist (l (send self :links)) (send l :make-pqpmodel))
   )
  (:links (&rest args) "Returns links, or args is passed to links" (user::forward-message-to-all links args))
  (:joint-list (&rest args) "Returns joint list, or args is passed to joints" (user::forward-message-to-all joint-list args))
  (:link (name) "Return a link with given name." (find name links :test #'equal :key #'(lambda (x) (send x :name))))
  (:joint (name) "Return a joint with given name." (find name joint-list :test #'equal :key #'(lambda (x) (send x :name))))
  (:end-coords
   (name)
   "Returns end-coords with given name"
   (find name end-coords-list :test #'equal :key #'(lambda (x) (send x :name))))

  (:bodies (&rest args) "Return bodies of this object. If args is given it passed to all bodies" (user::forward-message-to-all bodies args))
  (:faces () "Return faces of this object." (flatten (send-all bodies :faces)))

  (:update-descendants
   (&rest args)
   (send-all links :worldcoords))

  (:angle-vector
   (&optional vec
              (angle-vector (instantiate float-vector (calc-target-joint-dimension joint-list))))
   "Returns angle-vector of this object, if vec is given, it updates angles of all joint. If given angle-vector violate min/max range, the value is modified."
   (let ((i 0))
     (dolist (j joint-list)
       (when vec
           ;; for joint w/ min-max-table, see http://sourceforge.net/p/jskeus/tickets/43/
           (when (and (send j :joint-min-max-table) (send j :joint-min-max-target))
             ;; currently only 1dof joint is supported
             (when (and (= (send j :joint-dof) 1) (= (send (send j :joint-min-max-target) :joint-dof) 1))
               ;; find index of joint-min-max-target
               (let* ((ii 0) (jj (elt joint-list ii)) (ji 0)
                     tmp-joint-angle tmp-joint-min-angle tmp-joint-max-angle
                     tmp-target-joint-angle tmp-target-joint-min-angle tmp-target-joint-max-angle)
                 (while (not (eq jj (send j :joint-min-max-target)))
                   (incf ii) (incf ji (send jj :joint-dof)) (setq jj (elt joint-list ii)))
                 (setq tmp-joint-angle (elt vec i) tmp-target-joint-angle (elt vec ji))
                 (setq tmp-joint-min-angle (send j :joint-min-max-table-min-angle tmp-target-joint-angle)
                       tmp-joint-max-angle (send j :joint-min-max-table-max-angle tmp-target-joint-angle))
                 (setq tmp-target-joint-min-angle (send jj :joint-min-max-table-min-angle tmp-joint-angle)
                       tmp-target-joint-max-angle (send jj :joint-min-max-table-max-angle tmp-joint-angle))
                 (cond ((<= tmp-joint-min-angle tmp-joint-angle tmp-joint-max-angle) ;; ok
                        ;; force change joint-angle to avoid problem
                        (setq (j . joint-angle) tmp-joint-angle
                              (jj . joint-angle) tmp-target-joint-angle))
                       (t
                        ;;
                        (do ((i 0 (incf i 0.1)))
                            ((> i 1))
                          (setq tmp-joint-min-angle (send j :joint-min-max-table-min-angle tmp-target-joint-angle)
                                tmp-joint-max-angle (send j :joint-min-max-table-max-angle tmp-target-joint-angle))
                          (setq tmp-target-joint-min-angle (send jj :joint-min-max-table-min-angle tmp-joint-angle)
                                tmp-target-joint-max-angle (send jj :joint-min-max-table-max-angle tmp-joint-angle))

                          (if (< tmp-joint-angle tmp-joint-min-angle)
                              (incf tmp-joint-angle (* (- tmp-joint-min-angle tmp-joint-angle) i)))
                          (if (> tmp-joint-angle tmp-joint-max-angle)
                              (incf tmp-joint-angle (* (- tmp-joint-max-angle tmp-joint-angle) i)))
                          (if (< tmp-target-joint-angle tmp-target-joint-min-angle)
                              (incf tmp-target-joint-angle (* (- tmp-target-joint-min-angle tmp-target-joint-angle) i)))
                          (if (> tmp-target-joint-angle tmp-target-joint-max-angle)
                              (incf tmp-target-joint-angle (* (- tmp-target-joint-max-angle tmp-target-joint-angle) i)))
                          )
                        (setq (j . joint-angle) tmp-joint-angle
                              (jj . joint-angle) tmp-target-joint-angle)
                        (setf (elt vec i) tmp-joint-angle)
                        (setf (elt vec ji) tmp-target-joint-angle)
                        )
                       );; cond
               );; let*
             )) ;; when
           (case (send j :joint-dof)
             (1 (send j :joint-angle (elt vec i)))
             (t (send j :joint-angle (subseq vec i (+ i (send j :joint-dof)))))
             ))
       (dotimes (k (send j :joint-dof))
         (setf (elt angle-vector i)
	       (case (send j :joint-dof)
		 (1 (send j :joint-angle))
		 (t (elt (send j :joint-angle) k))))
         (incf i)))
     angle-vector))
  ;;
  (:find-link-route
    (to &optional from)
    (let ((pl (send to :parent-link)))
      (cond
       ;; if to is not included in (send self :links), just trace parent-link
       ((and pl (not (find to (send self :links))))
	(send self :find-link-route pl from))
       ;; if (send self :links), append "to" link
       ((and pl (not (eq to from)))
        (append (send self :find-link-route pl from) (list to)))
       ;; if link-route, just return "from" link
       ((and pl (eq to from))
        (list from))
       )))
  (:link-list
   (to &optional from)
   "Find link list from to link to from link."
   (let (ret1 ret2)
     (setq ret1 (send self :find-link-route to from))
     (when (and from (not (eq from (car ret1))))
       (setq ret2 (send self :find-link-route from (car ret1)))
       (setq ret1 (nconc (nreverse ret2) ret1))
       )
     ret1))
  ;; for min-max table
  (:make-joint-min-max-table
   (l0 l1 joint0 joint1
    ;; l0, l1         : link pair to be checked
    ;; joint0, joint1 : joint to be check min and max angle
    &key (fat 0) (fat2 nil) (debug nil)
         (margin 0.0) ;; margin [deg] is margin angle from collision-based min-angle and max-angle
         (overwrite-collision-model nil))
   ;; initialize for :make-joint-min-max-table
   (if (null debug) (send-all (send self :links) :analysis-level :coords))
   ;; for debug
   (dolist (jj (list joint0 joint1))
     (send jj :put :org-min-angle (send jj :min-angle))
     (send jj :put :org-max-angle (send jj :max-angle)))
   (let* ((pc (send self :copy-worldcoords))
          (pav (send self :angle-vector))
          (min-joint0 (truncate (- (send joint0 :min-angle) 1e-4)))
          (max-joint0 (truncate (+ (send joint0 :max-angle) 1e-4)))
          (min-joint1 (truncate (- (send joint1 :min-angle) 1e-4)))
          (max-joint1 (truncate (+ (send joint1 :max-angle) 1e-4)))
          (joint-range0 (1+ (round (- max-joint0 min-joint0))))
          (joint-range1 (1+ (round (- max-joint1 min-joint1)))))
     (when debug
       (format t ";~10A ~20,10f ~20,10f~%" (send joint0 :name) (send joint0 :min-angle) (send joint0 :max-angle))
       (format t ";~10A ~20,10f ~20,10f~%" (send joint1 :name) (send joint1 :min-angle) (send joint1 :max-angle))
       (format t ";~10A ~20,10f ~20,10f~%" (send joint0 :name) min-joint0 max-joint0)
       (format t ";~10A ~20,10f ~20,10f~%" (send joint1 :name) min-joint1 max-joint1))
     (send self :newcoords (make-coords))
     (send self :init-pose)
     (when overwrite-collision-model
       (setf (get l0 :pqpmodel) nil)    
       (setf (get l1 :pqpmodel) nil)
       (pqp-collision-check l0 l1 geo::PQP_FIRST_CONTACT :fat fat :fat2 fat2))

     (send self :make-min-max-table-using-collision-check
           l0 l1 joint0 joint1 joint-range0 joint-range1 min-joint0 min-joint1 fat fat2 debug margin)

     ;; finalize for :make-joint-min-max-table
     (send self :angle-vector pav)
     (send self :newcoords pc)
     (if (null debug) (send-all (send self :links) :analysis-level :body))
     nil
     ))
  (:make-min-max-table-using-collision-check
   (l0 l1 joint0 joint1 joint-range0 joint-range1 min-joint0 min-joint1 fat fat2 debug margin)
   ;; put collision check results into collision matrix "col-mat".
   (let ((col-mat (make-matrix joint-range1 joint-range0)))
          ;; col-mat : joint-range1 x joint-range0 matrix
          ;;   collision-free region => 0
          ;;   collision region => 1
     (let (flag c (skip-count 10))
       (do ((j 0 (+ j 1))) ((>= j joint-range1))
           (send joint1 :joint-angle (round (+ j min-joint1)))
           (setq flag :low-limit)
           (when (= (mod j 10) 0)
             (format t ";~c~7d/~7d" #x0d (* j joint-range0) (* joint-range0 joint-range1)) 
             (finish-output))
           (do ((i 0 (+ i 1))) ((>= i joint-range0))
               (catch :min-max-collision-check
                 (send joint0 :joint-angle (round (+ i min-joint0)))
                 (cond
                  ((and (eq flag :free) (/= (mod i skip-count) 0) (<= i (- joint-range0 skip-count)))
                   (setq c t))
                  (t
                   (setq c (= (pqp-collision-check l0 l1 geo::PQP_FIRST_CONTACT :fat fat :fat2 fat2) 0))
                   (cond 
                    ((and (eq flag :low-limit) c) 
                     (setq flag :free))
                    ((and (eq flag :free) (not c))
                     (setq flag :high-limit) (setq i (max (- i skip-count) 0))
                     (throw :min-max-collision-check nil))
                    )))
                 (setf (aref col-mat j i) (if c 1 0))
                 (if debug (format t ";~10A ~7,3f / ~10A ~7,3f / ~A ~A~%"
                                   (send joint1 :name) (+ j min-joint1)
                                   (send joint0 :name) (+ i min-joint0) c flag))
                 ))))
     (format t "~%")
     ;; generate min-max table from collision matrix
     (labels
         ((gen-min-max-table-from-collision-matrix
           (self-joint-range target-joint-range
                             self-min-joint target-min-joint
                             aref-func)
           (let ((table (make-matrix 3 self-joint-range))) ;; row = self-joint-angle, target-min, target-max
             (dotimes (i self-joint-range)
               (let ((min-v (1+ self-joint-range)) (max-v 0) (flag nil))
                 (dotimes (j target-joint-range)
                   (when (= (funcall aref-func col-mat i j) 1)
                     (setq flag t)
                     (if (< j min-v)(setq min-v j)) (if (> j max-v)(setq max-v j))))
                 (if (null flag) (setq min-v (- target-min-joint) max-v (- target-min-joint)))
                 (setf (aref table 0 i) (round (+ self-min-joint i)))
                 (setf (aref table 1 i) (round (+ margin target-min-joint min-v)))
                 (setf (aref table 2 i) (round (+ (- margin) target-min-joint max-v)))
                 ))
             table)))
       (let ((table0 (gen-min-max-table-from-collision-matrix
                      joint-range0 joint-range1
                      min-joint0 min-joint1
                      #'(lambda (m i j) (aref m j i)))))
         (send joint1 :joint-min-max-target joint0)
         (send joint1 :joint-min-max-table table0))
       (let ((table1 (gen-min-max-table-from-collision-matrix
                      joint-range1 joint-range0
                      min-joint1 min-joint0
                      #'(lambda (m i j) (aref m i j)))))
         (send joint0 :joint-min-max-target joint1)
         (send joint0 :joint-min-max-table table1))
       )))
  ;; plot function for min max table
  (:plot-joint-min-max-table-common
   (joint0 joint1)
   (let ((ret))
     (null-output
      (do ((j0 (get joint0 :org-min-angle) (+ 1 j0))) ((> j0 (get joint0 :org-max-angle)))
          (send self :init-pose)
          (send joint0 :joint-angle j0)
          (let ((flg0 (eps= (float (send joint0 :joint-angle)) (float j0))))
            (do ((j1 (get joint1 :org-min-angle)(+ 1 j1))) ((> j1 (get joint1 :org-max-angle)))
                (send joint1 :joint-angle j1)
                (let ((flg1 (eps= (float (send joint1 :joint-angle)) (float j1))))
                  (if (and flg0 flg1) (push (list j0 j1) ret))
                  ))
            )))
     (reverse ret)))
  (:plot-joint-min-max-table
   (joint0 joint1)
   "Plot joint min max table on Euslisp window."
   (let* ((min-max-table-view
           (instance x::panel :create
                     :width  (round (- (send joint0 :get :org-max-angle) (send joint0 :get :org-min-angle)))
                     :height (round (- (send joint1 :get :org-max-angle) (send joint1 :get :org-min-angle)))
                     :atitle "min-max-table-view"))
          (ret (send self :plot-joint-min-max-table-common joint0 joint1)))
     (send min-max-table-view :color #xffffff)
     (send min-max-table-view :flush)
     (send min-max-table-view :color #xff0000)
     (dolist (jj ret)
       (send min-max-table-view :draw-line
             (float-vector (- (round (car jj)) (get joint0 :org-min-angle)) (- (round (cadr jj)) (get joint1 :org-min-angle)))
             (float-vector (- (round (car jj)) (get joint0 :org-min-angle)) (- (round (cadr jj)) (get joint1 :org-min-angle)))))
     (send min-max-table-view :flush)
     ))
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
  (:calc-union-link-list
   (link-list)
   (cond
    ((atom (car link-list))   link-list)
    ((= (length link-list) 1) (car link-list))
    (t                        (reduce #'union link-list))))
  (:calc-target-joint-dimension
   (link-list)
   ;; link-list -> both list of link-list and atom of link-list OK.
   (calc-target-joint-dimension (send-all (send self :calc-union-link-list link-list) :joint)))
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
  (:calc-jacobian-from-link-list
   (link-list &rest args 
	      &key move-target
              ;; jacobian is represented in transform-coords
              (transform-coords move-target)
	      (rotation-axis (cond 
			      ((atom move-target) nil)
			      (t (make-list (length move-target)))))
	      (translation-axis (cond 
				 ((atom move-target) t)
				 (t (make-list (length move-target) :initial-element t))))
              (col-offset 0)
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
   "Calculate jacobian matrix from link-list and move-target. Unit system is [m] or [rad], not [mm] or [deg]."
   "Joint order for this jacobian matrix follows link-list order. Joint torque[Nm] order is also the same.
    Ex1. One-Arm
    (setq *rarm-link-list* (send *robot* :link-list (send *robot* :rarm :end-coords :parent)))
    (send-all *rarm-link-list* :joint)
    Ex2. Two-Arm
    (setq *arms-link-list* (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) '(:rarm :larm)))
    (send-all (send *robot* :calc-union-link-list *arms-link-list*) :joint)
   "
   (let* (len ul row
	  child-reverse union-link-list
	  j jdim)
     (unless fik (setq fik (make-matrix dim fik-len)))
;;
     (setq union-link-list (send self :calc-union-link-list link-list))
     (setq jdim (send self :calc-target-joint-dimension union-link-list))
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom transform-coords) (setq transform-coords (list transform-coords)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
;;
     (do ((col col-offset (+ col (send j :joint-dof)))
	  (i 0 (1+ i)))
         ((>= col (+ col-offset jdim)))
       (setq ul (elt union-link-list i) row 0)
       (dotimes (m (length link-list))
         (let ((link-list (elt link-list m))
               (move-target (elt move-target m))
               (transform-coords (elt transform-coords m))
               (rotation-axis (elt rotation-axis m))
               (translation-axis (elt translation-axis m)) l)
           (when (member ul link-list :test #'equal)
             (setq len (length link-list)
                   l (position ul link-list :test #'equal)
                   j (send ul :joint))
             (labels ((find-parent
                       (pl ll)
                       (if (or (find pl ll) (null pl))
                           pl (find-parent (send pl :parent) ll))))
               (cond ((not (derivedp (send j :child-link) bodyset-link))
                      (setq child-reverse nil))
                     ((or
                       (and (< (+ l 1) len)
                            (not (eq (send j :child-link)
                                     (find-parent (send (elt link-list (+ l 1)) :parent-link) link-list))))
                       (and (= len (+ l 1))
                            (not (eq (send j :child-link)
                                     (find-parent (send move-target :parent) link-list)))))
                      (setq child-reverse t))
                     (t (setq child-reverse nil))))
	     (let* ((paxis (case (j . axis)
			     (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
			     (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
			     (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
			     (t (j . axis))))
		    (child-link (send j :child-link))
		    (parent-link (send j :parent-link))
		    (default-coords (j . default-coords))
		    (world-default-coords (send (send parent-link :copy-worldcoords)
						:transform default-coords)))
	       (send j :calc-jacobian
		     fik row col j paxis child-link world-default-coords child-reverse
		     move-target transform-coords rotation-axis translation-axis
		     tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
	       )) ;; when
           (incf row (send self :calc-target-axis-dimension
			   rotation-axis translation-axis))
           )) ;; dotimes
       ) ;; do
     fik))
  ;;
  (:calc-joint-angle-speed
   (union-vel
    &rest args
    &key
    angle-speed
    (angle-speed-blending 0.5)
    jacobi jacobi# null-space I-J#J debug-view
    weight wmat tmp-len tmp-len2 fik-len
    &allow-other-keys)
   ;; argument check
   (when (or (null jacobi) (null jacobi#))
     (warn ";; ERROR : jacobi or jacobi# is required in :calc-joint-angle-speed~%")
     (return-from :calc-joint-angle-speed null-space))
   (if (null fik-len) (setq fik-len (cadr (array-dimensions jacobi))))
   (if (null tmp-len) (setq tmp-len (instantiate float-vector fik-len)))
   (if (null tmp-len2) (setq tmp-len2 (instantiate float-vector fik-len)))

   ;; dav = J#x + (I - J#J)y
   ;; calculate J#x
   (let ((J#x (transform jacobi# union-vel tmp-len)))
     (when (and debug-view (not (memq :no-message debug-view)))
       (format-array jacobi "J     :")
       (warn "d(JJt):~7,3f~%" (matrix-determinant (m* jacobi (transpose jacobi))) )
       (format-array (transpose jacobi#) "J#t   :")
       (format-array union-vel "x     :")
       (format-array (map float-vector #'rad2deg J#x) "J#x   :"))

     ;; add angle-speed to J#x using angle-speed-blending
     (when angle-speed
       (setq J#x (midpoint angle-speed-blending J#x angle-speed J#x))
       (when (and debug-view (not (memq :no-message debug-view)))
         (format-array (map float-vector #'rad2deg angle-speed) "aspeed:")
         (format-array (map float-vector #'rad2deg J#x) "J#x   :")))

     ;; if use null space
     (when (and (vectorp null-space)
		(= fik-len (length null-space))
                I-J#J)
       (if (null weight) (setq weight (fill (instantiate float-vector fik-len) 1)))
       ;; add null space
       (if (null wmat) (setq wmat (make-matrix fik-len fik-len)))
       (dotimes (i fik-len) (setf (aref wmat i i) (aref weight i)))
       (v+ J#x (transform I-J#J null-space tmp-len2) J#x)
       (when (and debug-view (not (memq :no-message debug-view)))
         (format-array (map float-vector #'rad2deg tmp-len2) "Ny    :")
         (format-array (map float-vector #'rad2deg J#x) "dav   :")))
     J#x))
  (:calc-joint-angle-speed-gain
   (union-link-list dav periodic-time)
   (let* ((fik-len (send self :calc-target-joint-dimension union-link-list))
	  (av (instantiate float-vector fik-len)) j)
     (do* ((i 0 (+ i (send j :joint-dof)))
	   (l 0 (1+ l)))
	 ((>= l (length union-link-list)))
       (setq j (send (elt union-link-list l) :joint))
       (dotimes (k (send j :joint-dof))
	 (setf (elt av (+ i k)) (send j :calc-angle-speed-gain dav i periodic-time))))
      av))
  (:collision-avoidance-links
   (&optional l)
   (if l (setq collision-avoidance-links l)) collision-avoidance-links)
  (:collision-avoidance-link-pair-from-link-list
    (link-lists &key obstacles ((:collision-avoidance-links collision-links) collision-avoidance-links) debug)
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
           link-list valid-link-list index-links
           (len (length collision-links))
           i0 ret)
      (if (atom (car link-lists)) (setq link-lists (list link-lists)))
      (when (and debug (not (memq :no-message debug)))
        (warn "collision-links : ~A~%" (send-all collision-links :name))
        (warn "parnet-links    : ~A~%" (send-all parent-links :name)))
      (dolist (link-list link-lists)
        ;; https://github.com/euslisp/jskeus/issues/459
        ;; originally introduced in https://github.com/euslisp/jskeus/commit/b0e3b482f8469241b2bfd9e9c607c19542b4d234
        ;; heuristics to skip adjacent links with same axis
        (setq valid-link-list
              (subseq  ;; returns link-list when it's length is 1
               link-list
               (do ((i 1 (1+ i)))
                   ((not (and (< i (1- (length link-list)))
                              (eps= (distance (send (elt link-list 0) :worldpos)
                                              (send (elt link-list i) :worldpos)) 0.0))) (1- i)))))
        (setq index-links
              (mapcar #'(lambda (l) (position l valid-link-list)) collision-links))
        (when (and debug (not (memq :no-message debug)))
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
                  (push (list (elt collision-links i) o) ret))))))
      ret))
  (:collision-avoidance-calc-distance
   (&rest args
	  &key union-link-list (warnp t)
          ((:collision-avoidance-link-pair pair-list))
          ((:collision-distance-limit distance-limit) 10) ;; [mm]
	  &allow-other-keys)
   (let* ((pair-len (length pair-list))
	  (col-list (send self :get :collision-distance))
          np pair)
     (when (or (null union-link-list) (= pair-len 0))
       (return-from :collision-avoidance-calc-distance nil))
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
	   (setq np (nconc (pqp-collision-distance (car pair) (cadr pair) :qsize 2) (list (float-vector 0 0 0) i)))
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
           (scale 0.001 (normalize-vector (v- (elt np 1) (elt np 2) (elt np 3)) (elt np 3)) (elt np 3))
	   ))) ;; dotimes
     ;; Element of "col-list" (variable "np") is list of:
     ;;   distance-between-pair1<->pair2
     ;;   nearest-point-on-pair1
     ;;   nearest-point-on-pair2
     ;;   direction-vector-from-p2->p1
     ;;   index-of-this-pair-within-collision-avoidance-link-pair
     (sort col-list #'<= #'(lambda (p) (car p)))
     (send self :put :collision-distance col-list)
     t))
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
	    (union-link-list) (link-list) (weight)
            ;; link-list -> both list and atom OK.
	    (fik-len (send self :calc-target-joint-dimension union-link-list))
            debug-view
            &allow-other-keys)
   (send* self :collision-avoidance-calc-distance args)
   (let ((pair-len (length pair-list)) (n 0)
         (dav-col (instantiate float-vector fik-len)))
     (send self :put :collision-pair-list pair-list)
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (while (< n pair-len)
       (let* ((np (elt (send self :get :collision-distance) n))
              (pair (elt pair-list (car (last np)))))
         (incf n)
         (when (and debug-view (not (memq :no-message debug-view)))
           (let ((v (normalize-vector (v- (elt np 1) (elt np 2)))))
             (warn "col-distance : ~7,3f(<~7,3f) #f(" (elt np 0) avoid-collision-distance)
             (dotimes (i 3) (warn "~7,3f " (elt v i)))
             (warn ") ~A~%" (send-all pair :name))))
         (unless (>= (elt np 0) avoid-collision-distance)

         ;; extract a-link-list from link-list
         ;;   link-list = (list link-list0 link-list1 .. )
         ;;   a-link-list = extract a list of links including (car pair) from link-list
         ;;   col-javobi = jacobian derived from a-link-list instead of union-link-list
         ;;   ddav-col = joint velocities derived from a-link-list instead of union-link-list
         ;;   dav-col = joint velocities union-link-list
         (let* ((a-link-list (find-if #'(lambda (x) (member (car pair) x :test #'equal)) link-list))
                (col-args (send self :collision-avoidance-args pair a-link-list))
                (drag-coords (make-cascoords :pos (elt np 1)))
                (col-link-list (send self :link-list (car pair) (car a-link-list))))
           (send (car pair) :assoc drag-coords)
           (let* ((col-jacobi (send* self :calc-jacobian-from-link-list
                                     col-link-list :move-target drag-coords
                                     col-args))
                  (ddav-col (transform (transpose col-jacobi) (elt np 3)))
                  (distance-gain (- (/ avoid-collision-distance (elt np 0)) 1)))
             (send (car pair) :dissoc drag-coords)
             (dolist (al col-link-list)
               (labels ((calc-target-joint-dimension-from-subseq
                         (alink alinklist)
                         (calc-target-joint-dimension
                          (send-all (subseq alinklist 0 (position alink alinklist :test #'equal))
                                    :joint))))
                 ;; dav-col-idx -> counter for dav-col
                 ;; ddav-col-idx -> counter for ddav-col
                 (let ((dav-col-idx (calc-target-joint-dimension-from-subseq
                                     al union-link-list))
                       (ddav-col-idx (calc-target-joint-dimension-from-subseq
                                      al col-link-list)))
                   (dotimes (i (calc-target-joint-dimension (list (send al :joint))))
                     (setf (elt dav-col (+ i dav-col-idx))
                           (+ (elt dav-col (+ i dav-col-idx))
                              (* distance-gain (elt ddav-col (+ ddav-col-idx i)))))
                     )
                   )))
             ))
         )))

       (dotimes (i fik-len) (setf (elt dav-col i) (* (elt dav-col i) (elt weight i))))
       (send self :put :collision-avoidance-null-vector
	     (scale avoid-collision-null-gain dav-col))
       (send self :put :collision-avoidance-joint-vector
	     (scale avoid-collision-joint-gain dav-col))
       (when (and debug-view (not (memq :no-message debug-view)))
         (format-array (map float-vector #'rad2deg (send self :get :collision-avoidance-joint-vector)) "coljnt:")
         (format-array (map float-vector #'rad2deg (send self :get :collision-avoidance-null-vector)) "colnul:")))
     (send self :get :collision-avoidance-joint-vector))
  (:move-joints
   (union-vel &rest args
    &key
    union-link-list
    (periodic-time 0.05)         ;; sec
    (joint-args)
    (debug-view nil)
    (move-joints-hook)
    &allow-other-keys)
   (let (dav dtheta j)
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))

     (setq dav (send* self :calc-joint-angle-speed union-vel args))

     ;; truncate to speed limit
     (let ((tmp-gain (send self :calc-joint-angle-speed-gain union-link-list dav periodic-time))
	   (min-gain 1.0))
       (dotimes (i (length tmp-gain))
	 (if (< (elt tmp-gain i) min-gain)
	     (setq min-gain (elt tmp-gain i))))
       (setq dav (scale min-gain dav dav)))

     (when (and debug-view (not (memq :no-message debug-view)))
       (format-array (map float-vector #'rad2deg dav) "dav^  :"))

     ;; update body
     (do ((i 0 (+ i (send j :joint-dof)))
	  (l 0 (1+ l)))
	 ((>= l (length union-link-list)))
       (setq j (send (elt union-link-list l) :joint))
       (case (send j :joint-dof)
	 (1 (setq dtheta (send j :speed-to-angle (elt dav i))))
	 (t (setq dtheta (send j :speed-to-angle (subseq dav i (+ i (send j :joint-dof)))))))
       (send* (elt union-link-list l) :joint :joint-angle dtheta :relative t joint-args))
     (if move-joints-hook (funcall move-joints-hook))
     t))
  (:find-joint-angle-limit-weight-old-from-union-link-list
    (union-link-list)
    (assoc (send-all union-link-list :name) (get self :joint-angle-limit-weight-old)
           :test #'(lambda (x y)
                     (and (= (length x) (length y))
                          (every #'identity (mapcar #'(lambda (xl yl)
                                                        (cond
                                                         ((and (symbolp xl) (symbolp yl)) (eq xl yl))
                                                         ((and (stringp xl) (stringp yl)) (string= xl yl))
                                                         (t )))
                                                    x y))))))
  (:reset-joint-angle-limit-weight-old
    (union-link-list)
    (let ((tmp-joint-angle-limit-weight-old
           (send self :find-joint-angle-limit-weight-old-from-union-link-list union-link-list)))
      (if tmp-joint-angle-limit-weight-old
          (setf (cadr tmp-joint-angle-limit-weight-old) nil))))
  ;; calc weight according to joint limit
  ;;  this method should be called from :calc-inverse-kinematics-weight
  (:calc-weight-from-joint-limit
    (avoid-weight-gain fik-len link-list union-link-list
                       debug-view weight tmp-weight tmp-len)
    (let (joint-angle-limit-weight-old joint-angle-limit-weight)
      (let ((tmp-joint-angle-limit-weight-old
             (send self :find-joint-angle-limit-weight-old-from-union-link-list union-link-list)))
        (unless tmp-joint-angle-limit-weight-old
          (send self :put :joint-angle-limit-weight-old
                (cons (list (send-all union-link-list :name) nil) (get self :joint-angle-limit-weight-old)))
          (setq tmp-joint-angle-limit-weight-old (send self :find-joint-angle-limit-weight-old-from-union-link-list union-link-list)))
        (unless (cadr tmp-joint-angle-limit-weight-old)
          (setf (cadr tmp-joint-angle-limit-weight-old) (fill (instantiate float-vector fik-len) 1.0e+20)))
        (setq joint-angle-limit-weight-old (cadr tmp-joint-angle-limit-weight-old)))

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
                     (joint-angle-limit-weight (send-all union-link-list :joint) tmp-len) tmp-len))
        (when (and debug-view (not (memq :no-message debug-view)))
          (format-array joint-angle-limit-weight-old "ocost :")
          (format-array joint-angle-limit-weight "cost  :"))
        (dotimes (i fik-len)
          (setf (elt tmp-weight i)
                (if (>=  (- (elt joint-angle-limit-weight i)
                            (elt joint-angle-limit-weight-old i)) 0.0)
                    (/ 1.0 (+ 1.0 (elt joint-angle-limit-weight i)))
                  1.0))
          (setf (elt joint-angle-limit-weight-old i) (elt joint-angle-limit-weight i))
          ))
      (when (= avoid-weight-gain 0.0)
        (dotimes (i fik-len) (setf (elt tmp-weight i) (elt weight i))))
      (let ((w-cnt 0))
        (dolist (l union-link-list)
          (let* ((dup (count-if
                       #'(lambda (x) (member l x))
                       link-list))
                 (w-len (calc-target-joint-dimension (list (send l :joint)))))
            (when (> dup 1)
              (dotimes (i w-len)
                (setf (elt tmp-weight (+ w-cnt i)) (/ (elt tmp-weight (+ w-cnt i)) dup))))
            (incf w-cnt w-len)
            )))
      tmp-weight))
  ;; calc all weight
  ;;   weight -> additional weight which user can set
  (:calc-inverse-kinematics-weight-from-link-list
    (link-list
     &key (avoid-weight-gain 1.0)
          (union-link-list (send self :calc-union-link-list link-list))
          (fik-len (send self :calc-target-joint-dimension union-link-list))
          (weight (fill (instantiate float-vector fik-len) 1)) ;; additional weight
          (additional-weight-list) ;; (list (list link1 var1) (list link2 var2) ...) ;; var is value or function (lambda)
          (debug-view)
          (tmp-weight (instantiate float-vector fik-len))
          (tmp-len (instantiate float-vector fik-len)))
    (cond
     ((functionp weight) (setq weight (funcall weight union-link-list)))
     ((listp weight) (setq weight (eval weight))))
    (when (and debug-view (not (memq :no-message debug-view)))
      (format-array weight "usrwei:"))
    (dolist (ls additional-weight-list)
      (let ((ll-pos (position (car ls) union-link-list :test #'equal)))
        (when ll-pos
          (let ((idx (reduce #'+ (send-all (send-all (subseq union-link-list 0 ll-pos) :joint) :joint-dof)
                             :initial-value 0))
                (var (if (functionp (cadr ls)) (funcall (cadr ls)) (cadr ls))))
            (dotimes (ii (send (send (car ls) :joint) :joint-dof))
              (setf (elt weight (+ ii idx)) (* (elt weight (+ ii idx))
                                               (if (float-vector-p var) (elt var ii) var)))))
          )))
    (when (and debug-view (not (memq :no-message debug-view)) additional-weight-list)
      (format-array weight "addwei:"))
    ;; calc weight from joint limit
    (setq tmp-weight
          (send self :calc-weight-from-joint-limit
                avoid-weight-gain fik-len link-list union-link-list
                debug-view weight tmp-weight tmp-len))
    (dotimes (i fik-len)
      (setf (elt tmp-weight i) (* (elt weight i) (elt tmp-weight i))))
#|
    ;; calc weight from joint max velocity
    (let* ((dav (fill (instantiate float-vector fik-len) 1))
    	   (speed-limit (send self :calc-joint-angle-speed-gain union-link-list dav 0.5))
	   (max-gain 1.0))
      (dotimes (i (length speed-limit)) (setq max-gain (max (elt speed-limit i) max-gain)))
      (setq speed-limit (scale (/ 1.0 max-gain) speed-limit speed-limit))
      (setq tmp-weight (transform (diagonal speed-limit) tmp-weight tmp-weight)))
|#
    ;;
    (when (and debug-view (not (memq :no-message debug-view)))
      (format-array tmp-weight "weight:"))
    tmp-weight)
  ;; calc nspace according to joint limit
  ;;  this method should be called from :calc-inverse-kinematics-nspace-from-link-list
  (:calc-nspace-from-joint-limit
    (avoid-nspace-gain union-link-list weight debug-view tmp-nspace)
    ;;
    ;; avoid-nspace-joint-limit: avoiding joint angle limit
    ;; 
    ;; dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
    ;;
    (when (> avoid-nspace-gain 0.0)
      (setq tmp-nspace
            (scale avoid-nspace-gain
                   (joint-angle-limit-nspace (send-all union-link-list :joint) tmp-nspace) tmp-nspace))
      (dotimes (i (length weight))
        (setf (elt tmp-nspace i) (* (elt tmp-nspace i) (elt weight i))))
      (when (and debug-view (not (memq :no-message debug-view)))
        (format-array (map float-vector #'rad2deg tmp-nspace) "nspace:")))
    tmp-nspace)
  ;; calc all nspace
  ;;   null-space -> additional null space vector which user can set
  (:calc-inverse-kinematics-nspace-from-link-list
    (link-list
     &key (avoid-nspace-gain 0.01)
          (union-link-list (send self :calc-union-link-list link-list))
          (fik-len (send self :calc-target-joint-dimension union-link-list))
          (null-space) (debug-view)
          (additional-nspace-list) ;; (list (list link1 var1) (list link2 var2) ...) ;; var is value or function (lambda)
	  ;; if user wants to use cog-jacobian as null-space, please set cog-gain(> 0.0) and target-centroid-pos
	  (cog-gain 0.0) (target-centroid-pos) (centroid-offset-func) (cog-translation-axis :z) (cog-null-space nil)
          (weight (fill (instantiate float-vector fik-len) 1.0))
          (update-mass-properties t)
          (tmp-nspace (instantiate float-vector fik-len)))
    ;; calc null-space from joint-limit
    (setq tmp-nspace
          (send self :calc-nspace-from-joint-limit
                avoid-nspace-gain union-link-list weight debug-view tmp-nspace))
    ;; add null-space from cog-jacobian
    ;;   when cog-null-space is nil, null-space from cog-jacobian is not added
    (when cog-null-space
      (if (and (> cog-gain 0.0) target-centroid-pos)
          (setq tmp-nspace
                (v+ (send self :cog-jacobian-balance-nspace union-link-list
                          :weight weight :centroid-offset-func centroid-offset-func
                          :update-mass-properties update-mass-properties
                          :target-centroid-pos target-centroid-pos :cog-gain cog-gain :translation-axis cog-translation-axis)
                    tmp-nspace tmp-nspace))))
    ;; add null-space from arguments
    (dolist (ls additional-nspace-list)
      (let ((ll-pos (position (car ls) union-link-list :test #'equal)))
        (when ll-pos
          (let ((idx (reduce #'+ (send-all (send-all (subseq union-link-list 0 ll-pos) :joint) :joint-dof)
                             :initial-value 0))
                (var (if (functionp (cadr ls)) (funcall (cadr ls)) (cadr ls))))
            (dotimes (ii (send (send (car ls) :joint) :joint-dof))
              (setf (elt tmp-nspace (+ ii idx))
                    (+ (elt tmp-nspace (+ ii idx))
                       (* (if (float-vector-p var) (elt var ii) var)
                          (elt weight (+ ii idx)))))))
          )))
    (cond
     ((functionp null-space) (setq null-space (funcall null-space)))
     ((listp null-space) (setq null-space (eval null-space))))
    (if null-space
        (setq tmp-nspace
              (v+ null-space tmp-nspace tmp-nspace)))
    tmp-nspace)
  (:move-joints-avoidance
   (union-vel &rest args
              &key
              union-link-list link-list
              (fik-len (send self :calc-target-joint-dimension union-link-list))
              (weight (fill (instantiate float-vector fik-len) 1))
              (null-space)
              (avoid-nspace-gain 0.01)
              (avoid-weight-gain 1.0)
              (avoid-collision-distance 200)
              (avoid-collision-null-gain 1.0)
              (avoid-collision-joint-gain 1.0)
              ((:collision-avoidance-link-pair pair-list)
               (send self :collision-avoidance-link-pair-from-link-list link-list :obstacles (cadr (memq :obstacles args)) :debug (cadr (memq :debug-view args))))
	      (cog-gain 0.0) (target-centroid-pos) (centroid-offset-func) (cog-translation-axis :z) (cog-null-space nil)
              (additional-weight-list) (additional-nspace-list)
              (tmp-len (instantiate float-vector fik-len))
              (tmp-len2 (instantiate float-vector fik-len))
	      (tmp-weight (instantiate float-vector fik-len))
	      (tmp-nspace (instantiate float-vector fik-len))
              (tmp-mcc (make-matrix fik-len fik-len))
              (tmp-mcc2 (make-matrix fik-len fik-len))
              (debug-view) (jacobi)
	    &allow-other-keys)
   (let* (joint-angle-limit-nspace
	  dav dtheta 
          null-space-collision-avoidance
          angle-speed-collision-avoidance
          (angle-speed-collision-blending 0.0) min-distance
          jacobi# I-J#J)
     (when (null jacobi)
       (warn ";; ERROR : jacobi is required in :move-joints-avoidance~%")
       (return-from :move-joints-avoidance t))
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (when (and debug-view (not (memq :no-message debug-view)))
       (warn "angle :")
       (dolist (j (send-all union-link-list :joint))
	 (let ((a (send j :joint-angle)))
	   (if (vectorp a)
	       (dotimes (i (length a)) (warn "~7,1f " (elt a i)))
	     (warn "~7,1f " a))))
	   (warn "~%")
       (warn " min  :")
       (dolist (j (send-all union-link-list :joint))
	 (let ((a (send j :min-angle)))
	   (if (vectorp a)
	       (dotimes (i (length a)) (warn "~7,1f " (elt a i)))
	     (warn "~7,1f " a))))
	   (warn "~%")
       (warn " max  :")
       (dolist (j (send-all union-link-list :joint))
	 (let ((a (send j :max-angle)))
	   (if (vectorp a)
	       (dotimes (i (length a)) (warn "~7,1f " (elt a i)))
	     (warn "~7,1f " a))))
	   (warn "~%")
           )
     (setq weight ;; copied to tmp-weight
           (send self :calc-inverse-kinematics-weight-from-link-list
                 link-list :weight weight :fik-len fik-len
                 :avoid-weight-gain avoid-weight-gain :union-link-list union-link-list
                 :debug-view debug-view :tmp-len tmp-len :tmp-weight tmp-weight
                 :additional-weight-list additional-weight-list))

     ;; calc inverse jacobian and projection jacobian
     (setq jacobi# (send* self :calc-inverse-jacobian jacobi :weight weight args))
     (fill (array-entity tmp-mcc) 0)
     (dotimes (i fik-len) (setf (aref tmp-mcc i i) 1.0))
     (setq I-J#J (m- tmp-mcc
                     (m* jacobi# jacobi tmp-mcc2) tmp-mcc))

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
     (send self :put :collision-avoidance-null-vector nil)
     (send self :put :collision-avoidance-joint-vector nil)
     (when (and pair-list (> avoid-collision-distance 0.0)
		(or (> avoid-collision-joint-gain 0.0)
		    (> avoid-collision-null-gain 0.0)))
       (setq angle-speed-collision-avoidance
	     (send* self :collision-avoidance
			 :avoid-collision-distance avoid-collision-distance
			 :avoid-collision-null-gain avoid-collision-null-gain
			 :avoid-collision-joint-gain avoid-collision-joint-gain
                         :weight weight
			 :collision-avoidance-link-pair pair-list args))
       (setq min-distance (car (elt (send self :get :collision-distance) 0)))
       (setq angle-speed-collision-blending
	     (cond ((<= avoid-collision-joint-gain 0.0) 0.0)
		   ((< min-distance (* 0.1 avoid-collision-distance))
		    1.0)
		   ((< min-distance avoid-collision-distance)
		    (/ (- avoid-collision-distance min-distance) 
		       (* 0.9 avoid-collision-distance)))
		   (t
		    0.0)))
       (when (and debug-view (not (memq :no-message debug-view)))
         (format-array (map float-vector #'rad2deg angle-speed-collision-avoidance) "colvel:")
         (warn "blend :~7,3f~%" angle-speed-collision-blending)))
     (setq tmp-nspace
           (send self :calc-inverse-kinematics-nspace-from-link-list
                 link-list :union-link-list union-link-list
                 :avoid-nspace-gain avoid-nspace-gain :debug-view debug-view
		 :cog-gain cog-gain :target-centroid-pos target-centroid-pos :centroid-offset-func centroid-offset-func :cog-null-space cog-null-space
                 :update-mass-properties nil
                 :cog-translation-axis cog-translation-axis
                 :weight weight :fik-len fik-len :null-space null-space :tmp-nspace tmp-nspace
                 :additional-nspace-list additional-nspace-list))
     (if (send self :get :collision-avoidance-null-vector)
	 (v+ tmp-nspace (send self :get :collision-avoidance-null-vector) tmp-nspace))
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
     (send-message* self cascaded-link
		    :move-joints union-vel
		    :union-link-list union-link-list
		    :null-space tmp-nspace
		    :angle-speed angle-speed-collision-avoidance
		    :angle-speed-blending angle-speed-collision-blending
		    :weight weight :jacobi# jacobi# :I-J#J I-J#J
		    args)
     t))
  ;;
  (:inverse-kinematics-args
   (&rest args &key union-link-list rotation-axis translation-axis additional-jacobi-dimension
          &allow-other-keys)
   (let* ((c (send self :calc-target-joint-dimension union-link-list))
          ;; add dimensions of additonal-jacobi
          (r (+ (send self :calc-target-axis-dimension rotation-axis translation-axis) additional-jacobi-dimension))
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
            (min-distance (car (elt (send self :get :collision-distance) 0)))
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
          (send *viewer* :viewsurface :color
                (if (eps= min-distance (car np) 1.0) #f(1.0 0.0 0.0) #f(1.0 0.0 1.0)))
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
   (dif-pos dif-rot
    &rest args
    &key (stop 1) (loop 0) link-list move-target
    (rotation-axis (cond
		    ((atom move-target) t)
                    (t (make-list (length move-target) :initial-element t))))
    (translation-axis (cond
                       ((atom move-target) t)
                       (t (make-list (length move-target) :initial-element t))))
    (thre (cond
           ((atom move-target) 1)
           (t (make-list (length move-target) :initial-element 1))))
    (rthre (cond
            ((atom move-target) (deg2rad 1))
            (t (make-list (length move-target) :initial-element (deg2rad 1)))))
    (dif-pos-ratio 1.0)
    (dif-rot-ratio 1.0)
    union-link-list
    target-coords                 ;required for debug-view
    (jacobi)
    (additional-check)
    (additional-jacobi) (additional-vel)
    ;; Cog convergence params
    (centroid-thre 1.0) (target-centroid-pos) (centroid-offset-func) (cog-translation-axis :z) (cog-null-space nil) (cog-gain 1.0)
    (min-loop (/ stop 10))
    debug-view ik-args
    &allow-other-keys)
   ":inverse-kinematics-loop is one loop calculation for :inverse-kinematics.
    In this method, joint position difference satisfying workspace difference (dif-pos, dif-rot) are calculated and euslisp model joint angles are updated.
    Optional arguments:
     :additional-check
       This argument is to add optional best-effort convergence conditions.
       :additional-check should be function or lambda.
       best-effort => In :inverse-kinematics-loop, 'success' is overwritten by '(and success additional-check)'
                      In :inverse-kinematics, 'success is not overwritten.
                      So, :inverse-kinematics-loop wait until ':additional-check' becomes 't' as possible,
                      but ':additional-check' is neglected in the final :inverse-kinematics return.
     :min-loop
       Minimam loop count (nil by default).
       If integer is specified, :inverse-kinematics-loop does returns :ik-continues and continueing solving IK.
       If min-loop is nil, do not consider loop counting for IK convergence.
    "
   (if target-centroid-pos (send self :update-mass-properties))
   ;; dif-pos, dif-rot, move-target, rotation-axis, translation-axis, link-list
   ;; -> both list and atom OK.
   (let (tmp-dim tmp-dims (vec-count 0) (success t)
         (method-args (nconc args ik-args))
         (p-limit (cadr (memq :p-limit args)))
         (r-limit (cadr (memq :r-limit args))))
       (when (null union-link-list)
         (setq union-link-list (send self :calc-union-link-list link-list))
         ;; tempolary reset weight for prevision joint-angle-limit-weight-old using hash-table
         (if (/= (length union-link-list) (length link-list))
             (send self :reset-joint-angle-limit-weight-old union-link-list)))

       ;; atom -> list
       (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
       (if (and debug-view (not (equal debug-view :no-clear)) *viewer*)
	   (send *viewer* :viewsurface :clear))
       (if (atom (car link-list)) (setq link-list (list link-list)))
       (if (atom move-target) (setq move-target (list move-target)))
       (if (or (atom target-coords) (functionp target-coords)) (setq target-coords (list target-coords)))
       (if (atom dif-pos) (setq dif-pos (list dif-pos)))
       (if (atom dif-rot) (setq dif-rot (list dif-rot)))
       (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
       (if (atom translation-axis) (setq translation-axis (list translation-axis)))       
       (if (atom thre) (setq thre (list thre)))
       (if (atom rthre) (setq rthre (list rthre)))
       ;; argument check
       (unless (= (length translation-axis) (length rotation-axis)
                  (length move-target) (length link-list)
                  (length dif-pos) (length dif-rot))
         (warn ";; ERROR: list length differ : translation-axis ~A rotation-axis ~A move-target ~A link-list ~A dif-pos ~A dif-rot ~A~%"
               (length translation-axis) (length rotation-axis)
               (length move-target) (length link-list)
               (length dif-pos) (length dif-rot))
         (return-from :inverse-kinematics-loop :ik-continues))

       (if (memq :tmp-dims ik-args)
           (setq tmp-dims (cadr (memq :tmp-dims ik-args)))
         (progn
           (dotimes (i (length rotation-axis))
             (push (instantiate float-vector (send self :calc-target-axis-dimension (elt rotation-axis i) (elt translation-axis i))) tmp-dims))
           (setq tmp-dims (nreverse tmp-dims))))
       (if (memq :tmp-dim ik-args)
           (setq tmp-dim (cadr (memq :tmp-dim ik-args)))
         (setq tmp-dim (instantiate float-vector (send self :calc-target-axis-dimension rotation-axis translation-axis))))
       
       (if (functionp jacobi) (setq jacobi (funcall jacobi link-list move-target translation-axis rotation-axis)))
       (unless jacobi
         (setq jacobi (send* self :calc-jacobian-from-link-list link-list
                             :translation-axis translation-axis
                             :rotation-axis rotation-axis
                             :move-target move-target
                             method-args)))
       (when (and debug-view (not (memq :no-message debug-view)))
         (warn "loop: ~3d~%" loop)
         (warn "union-link-list: ~A~%" (send-all union-link-list :name))
         (warn "move-target: ~A~%" move-target)
         (warn "targe-coordst: ~A~%" target-coords)
	 )

       ;; convergence check
       (setq success
	     (send self :ik-convergence-check
		   (if min-loop (>= loop min-loop) t)
                   dif-pos dif-rot
		   rotation-axis translation-axis thre rthre
		   centroid-thre target-centroid-pos centroid-offset-func cog-translation-axis
                   :update-mass-properties nil))
       (if additional-check
           (setq success (and success (funcall additional-check))))

       ;; calculation of move-coords velocities from vel-p and vel-r
       (dotimes (i (length move-target))
         (let* ((tmp-dim (elt tmp-dims i))
                (vel-p (send* self :calc-vel-from-pos (elt dif-pos i) (elt translation-axis i)
                              (if p-limit (list :p-limit p-limit))))
                (vel-r (send* self :calc-vel-from-rot (elt dif-rot i) (elt rotation-axis i)
                              (if r-limit (list :r-limit r-limit)))))
           (when (and debug-view (not (memq :no-message debug-view)))
             (format-array (scale 1000.0 vel-p) "vel-pos :")
             (format-array vel-r "vel-rot :")
             (warn "vel-pos-norm : ~7,3f/~7,3f~%vel-rot-norm : ~7,3f/~7,3f~%"
                   (* 1000.0 (norm vel-p)) (elt thre i) (norm vel-r) (elt rthre i)))
           (dotimes (j (length vel-p)) (setf (elt tmp-dim j) (* dif-pos-ratio (elt vel-p j))))
           (dotimes (j (length vel-r)) (setf (elt tmp-dim (+ j (length vel-p))) (* dif-rot-ratio (elt vel-r j))))
           (when (send self :get :ik-target-error)
             (push (list vel-p vel-r) (cdr (assoc (read-from-string (format nil ":target-~d" i)) (send self :get :ik-target-error)))))
           ))
       (dotimes (i (length tmp-dims))
         (dotimes (j (length (elt tmp-dims i)))
           (setf (elt tmp-dim (+ j vec-count)) (elt (elt tmp-dims i) j)))
         (incf vec-count (length (elt tmp-dims i))))
       
       ;; Use cog jacobian as first task
       (when (and (not cog-null-space) target-centroid-pos)
         (setq additional-jacobi
               (append (list (send self :calc-cog-jacobian-from-link-list
                                   :update-mass-properties nil
                                   :link-list union-link-list
                                   :translation-axis cog-translation-axis))
                       additional-jacobi))
         (let ((tmp-cog-gain (if (> cog-gain 1.0) 1.0 cog-gain)))
           (setq additional-vel
                 (append (list
                          (send self :calc-vel-for-cog tmp-cog-gain cog-translation-axis target-centroid-pos
                                :centroid-offset-func centroid-offset-func
                                :update-mass-properties nil))
                         additional-vel))
           (when (send self :get :ik-target-error)
             (push (car additional-vel) (cdr (assoc :centroid (send self :get :ik-target-error)))))
           ))
       ;; append additional-jacobi and additional-vel
       (let (add-vel-list (row0) add-vel add-jacobi)
         (when additional-jacobi
           (setq add-vel-list (mapcar #'(lambda (x) (if (functionp x) (funcall x link-list) x)) additional-vel))
           (setq row0 (- (length tmp-dim) (reduce #'+ (mapcar #'length add-vel-list))))
           (dotimes (i-add-jacobi (length additional-jacobi))
             (setq add-jacobi (elt additional-jacobi i-add-jacobi))
             (setq add-jacobi (if (functionp add-jacobi) (funcall add-jacobi link-list) add-jacobi))
             (setq add-vel (elt add-vel-list i-add-jacobi))
             (dotimes (i-row (array-dimension add-jacobi 0))
               ;; set additional-jacobi
               (dotimes (i-col (array-dimension jacobi 1))
                 (setf (aref jacobi (+ row0 i-row) i-col) (aref add-jacobi i-row i-col)))
               (setf (elt tmp-dim (+ row0 i-row)) (elt add-vel i-row))
               )
             (setq row0 (+ row0 (length add-vel)))
             (when (and debug-view (not (memq :no-message debug-view)))
               (format-array add-jacobi (format nil "add-J~a    :" i-add-jacobi))
               (format-array add-vel (format nil "add-v~a    :" i-add-jacobi)))
             (when (send self :get :ik-target-error)
               (if (and (not cog-null-space) target-centroid-pos (/= i-add-jacobi 0)) ;; First additional-vel is for COG, so neglect
                   (push (car additional-vel) (cdr (assoc (read-from-string (format nil ":additional-~d" (- i-add-jacobi 1))) (send self :get :ik-target-error))))))
             )))

       ;; check loop end
       (if success
           (progn
             (send self :put :ik-draw-on-params nil) ;; Clear :ik-draw-on-params
             :ik-succeed)
         (progn
           (when (and debug-view (not (memq :no-clear debug-view)) *viewer*)
             (send *viewer* :viewsurface :clear))
           (send self :put :collision-pair-list nil)
           (send-message* self cascaded-link
                          :move-joints-avoidance tmp-dim
                          :union-link-list union-link-list
                          :rotation-axis rotation-axis
                          :translation-axis translation-axis
                          :jacobi jacobi :debug-view debug-view
                          ;; buffer for calculation
                          method-args)
           ;; Visualizaion for :inverse-kinematics-loop. In :inverse-kinematics-loop,
           ;;   1. Calculate of current value and target value
           ;;   2. Solve inverse kinematics, update robot state, and update robot's viewer state
           ;;   3. Add current value and target value viewer draw-on information.
           ;;   4. Call (send *viewer* :viewsurface :flush) and everything are displayed.
           ;;   For 3, we need to store information of 1 through :draw-collision-debug-view or :ik-draw-on-params
           (when (and debug-view  *viewer*)
             (send *viewer* :draw-objects :clear nil :flush nil)
             ;;
             ;; done
             (send self :draw-collision-debug-view)
             (when (car target-coords)
               (dotimes (i (length target-coords))
                 (send-message (elt move-target i) coordinates :draw-on :flush nil :size 100)
                 (send (elt target-coords i) :draw-on :flush nil :color #f(1 0 0))))
             (dolist (p (send self :get :ik-draw-on-params))
               (send* (car p) :draw-on :flush nil (cdr p)))
             (if (not (memq :no-flush debug-view)) (send *viewer* :viewsurface :flush))
             (x::window-main-one)
             )
           (send self :put :ik-draw-on-params nil) ;; Clear :ik-draw-on-params
           :ik-continues)
         )
       ))
  (:inverse-kinematics
   (target-coords &rest args 
		  &key (stop 50)
		  (link-list)
		  (move-target)
		  (debug-view) (warnp t) (revert-if-fail t)
                  (rotation-axis (cond
				  ((atom move-target) t)
				  (t (make-list (length move-target) :initial-element t))))
                  (translation-axis (cond
				     ((atom move-target) t)
				     (t (make-list (length move-target) :initial-element t))))
                  (joint-args)
		  (thre (cond
                         ((atom move-target) 1)
                         (t (make-list (length move-target) :initial-element 1))))
		  (rthre (cond
                          ((atom move-target) (deg2rad 1))
                          (t (make-list (length move-target) :initial-element (deg2rad 1)))))
                  (union-link-list)
		  (centroid-thre 1.0) (target-centroid-pos) (centroid-offset-func) (cog-translation-axis :z) (cog-null-space nil)
		  (dump-command :fail-only) (periodic-time 0.5) ;; [s]
                  (check-collision)
                  (additional-jacobi) (additional-vel)
		  &allow-other-keys)
   "Move move-target to target-coords.
   ;; target-coords, move-target, rotation-axis, translation-axis, link-list
   ;; -> both list and atom OK.
    target-coords : The coordinate of the target, or a function that returns coordinates. Use a list of targets to solve the IK relative to multiple end links simultaneously.
    link-list : List of links to control. When the target-coords is list, this should be a list of lists.
    move-target : Specify end-effector coordinate. When the target-coords is list, this should be list too.
    stop : Maximum number for IK iteration. Default is 50.
    debug-view : Set t to show debug message and visualization. Use :no-message to just show the irtview image. Default is nil.
    warnp : Set nil to not display debug message when the IK failed. Default is t.
    revert-if-fail : Set nil to keep the angle posture of IK solve iteration. Default is t, which return to original position when IK fails.
    rotation-axis : Use nil for position only IK. :x, :y, :z for the constraint around axis with plus direction, :-x :-y :-z for axis with minus direction. :zz :yy :zz for direction free axis. :xm :ym :zm for allowing rotation with respect to mirror direction. When the target-coords is list, this should be list too. Default is t.
    translation-axis : :x :y :z for constraint along the x, y, z axis. :xy :yz :zx for plane. Default is t.
    thre : Threshold for position error to terminate IK iteration. Default is 1 [mm].
    rthre : Threshold for rotation error to terminate IK iteration. Default is 0.017453 [rad] (1 deg).
    check-collision : Set t to return false when self collision occurs at found IK solution. Default is nil. To avoid collision within IK loop, use set links to collision-avoidance-links slot of cascaded-link.
    dump-command : should be t, nil, :always, :fail-only, :always-with-debug-log, or :fail-only-with-debug-log.
    Log are success/fail log and ik debug information log.
      t or :always           : dump log both in success and fail (for success/fail log).
      :always-with-debug-log : dump log both in success and fail (for success/fail log and ik debug information log).
      :fail-only             : dump log only in fail (for success/fail log).
      :always-with-debug-log : dump log only in fail (for success/fail log and ik debug information log).
      nil                    : do not dump log."
   (if (or (atom target-coords) (functionp target-coords)) (setq target-coords (list target-coords)))
   (let* ((loop 0)
          (union-link-list (if (functionp union-link-list) (funcall union-link-list link-list) (send self :calc-union-link-list link-list)))
          (av0 (send-all (remove-duplicates
			  (append (send-all union-link-list :joint) joint-list))
                         :joint-angle))
          (c0 (unless (send self :parent) (send self :copy-worldcoords)))
	  (target-coords0 target-coords)
	  dif-pos dif-rot
	  ;;
          (success t) 
	  (old-analysis-level (send-all union-link-list :analysis-level))
          ik-args collision-status
          ;; Parameters for log file
          command-directory command-filename command-id
          initial-target-coords initial-av initial-log-str
          )       
     (send-all union-link-list :analysis-level :coords)
     ;; argument check
     (when (or (null link-list) (null move-target))
       (warn ";; ERROR: both :link-list and :move-target required~%")
       (return-from :inverse-kinematics t))
     (if (and (null translation-axis) (null rotation-axis))
         (return-from :inverse-kinematics t))
     ;; setup initial parameters for logging
     (when dump-command
	(setq initial-log-str
              (format nil ";; ik ~A log at ~A on ~A~%;;~%" (if success "success" "fail") (string-trim '(10) (unix:asctime (unix:localtime))) lisp::lisp-implementation-version))
        (setq initial-log-str (format nil "~A;; link-list ~A~%" initial-log-str link-list))
	(setq initial-log-str (format nil "~A;; move-target ~A~%" initial-log-str move-target))
	(setq initial-log-str (format nil "~A;; rotatoin-axis ~A, translation-axis ~A~%" initial-log-str rotation-axis translation-axis))
	(setq initial-log-str (format nil "~A;; thre ~A, rthre ~A, stop ~A~%" initial-log-str thre rthre stop))
  (setq initial-target-coords `(setq c0 ',(mapcar #'(lambda (x) x) target-coords)))
        (setq initial-av `(setq av0 ,(send self :angle-vector))))
     ;; atom -> list
     (when debug-view
       (if (atom debug-view) (setq debug-view (list debug-view)))
       (when (memq :no-clear debug-view)
	 (push :no-clear debug-view))
       (when (memq :no-flush debug-view)
	 (push :no-flush debug-view)))
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
     (if (atom thre) (setq thre (list thre)))
     (if (atom rthre) (setq rthre (list rthre)))
     (if (and additional-jacobi (or (atom additional-jacobi) (functionp additional-jacobi))) (setq additional-jacobi (list additional-jacobi)))
     (if (and additional-vel (or (atom additional-vel) (functionp additional-vel))) (setq additional-vel (list additional-vel)))
     ;; argument check
     (unless (= (length translation-axis) (length rotation-axis)
                (length move-target) (length link-list) (length target-coords))
       (warn ";; ERROR: list length differ : translation-axis ~A rotation-axis ~A move-target ~A link-list ~A target-coords ~A~%"
             (length translation-axis) (length rotation-axis)
             (length move-target) (length link-list) (length target-coords))
       (return-from :inverse-kinematics t))
     (unless (= (length additional-jacobi) (length additional-vel))
       (warn ";; ERROR: list length differ : additional-jacobi ~A additional-vel ~A~%"
             (length additional-jacobi) (length additional-vel))
       (return-from :inverse-kinematics t))
     (let ((tmp-additional-jacobi (mapcar #'(lambda (aj) (if (functionp aj) (funcall aj link-list) aj)) additional-jacobi)))
       (setq ik-args (nconc (send* self :inverse-kinematics-args
                                   :translation-axis translation-axis
                                   :rotation-axis rotation-axis
                                   ;; evaluate additional-jacobi function and calculate row dimension of additional-jacobi
                                   :additional-jacobi-dimension
                                   (+ (if (and (not cog-null-space) target-centroid-pos)
                                          (send self :calc-target-axis-dimension nil cog-translation-axis)
                                        0)
                                      (reduce #'+ (mapcar #'(lambda (aj) (array-dimension (if (functionp aj) (funcall aj link-list) aj) 0)) tmp-additional-jacobi)))
                                   :union-link-list union-link-list args) args)))
     (send self :reset-joint-angle-limit-weight-old union-link-list) ;; reset weight
     (when (memq dump-command '(:always-with-debug-log :fail-only-with-debug-log))
       ;; If ik debug information log is enabled, initialize empty list for ik target errors.
       (send self :put :ik-target-error
             (append
              (let ((tmp-count -1)) (mapcar #'(lambda (x) (list (read-from-string (format nil ":target-~d" (incf tmp-count))))) (make-list (length target-coords)))) ;; For target-coords error
              (if target-centroid-pos (list (list :centroid))) ;; For taget-centroid-pos error
              (let ((tmp-count -1)) (mapcar #'(lambda (x) (list (read-from-string (format nil ":additional-~d" (incf tmp-count))))) (make-list (length additional-vel)))) ;; For additional-vel error
              )))
     
     ;; inverse kinematics loop
     (while (< (incf loop) stop)
       (let* ((target-coords
               (mapcar #'(lambda (x)
                           (if (functionp x) (funcall x) x))
                       target-coords))
              (dif-pos
               (mapcar #'(lambda (mv tc trans-axis)
                           (send mv :difference-position tc
                                 :translation-axis trans-axis))
                       move-target target-coords translation-axis))
              (dif-rot
               (mapcar #'(lambda (mv tc rot-axis)
                           (send mv :difference-rotation tc
                                 :rotation-axis rot-axis))
                       move-target target-coords rotation-axis)))
         (setq success (send* self :inverse-kinematics-loop
                              dif-pos dif-rot
                              :target-coords target-coords
                              :periodic-time periodic-time
                              :stop stop :loop loop
                              :rotation-axis rotation-axis
                              :translation-axis translation-axis
                              :move-target move-target
                              :union-link-list union-link-list
                              :thre thre :rthre rthre
                              :additional-jacobi additional-jacobi
                              :additional-vel additional-vel
                              :debug-view debug-view
                              :ik-args ik-args args))
         (when (eq success :ik-succeed)
	   (setq success t) (return nil))
         ))
     (if target-centroid-pos (send self :update-mass-properties))
     (let* ((target-coords
	     (mapcar #'(lambda (x)
			 (if (functionp x) (funcall x) x))
		     target-coords)))
       (setq dif-pos (mapcar #'(lambda (mt tc ta)
				 (send mt :difference-position tc :translation-axis ta))
			     move-target target-coords translation-axis)
	     dif-rot (mapcar #'(lambda (mt tc ra)
				 (send mt :difference-rotation tc :rotation-axis ra))
			     move-target target-coords rotation-axis)
	     success
	     (send self :ik-convergence-check
		   success dif-pos dif-rot
		   rotation-axis translation-axis thre rthre
		   centroid-thre target-centroid-pos centroid-offset-func cog-translation-axis
                   :update-mass-properties nil)))
        ;; update difference
     (mapcar #'(lambda (l a) (send l :analysis-level a)) union-link-list old-analysis-level)
     ;; reset weight
     (send self :reset-joint-angle-limit-weight-old union-link-list)
     ;; Get final ik success result flag
     (if check-collision (setq collision-status (send self :self-collision-check)))
     (setq success (and (or success (not revert-if-fail)) (not collision-status)))
     ;; Dump log
     (when (or (memq dump-command '(:always :always-with-debug-log t))
               (and (memq dump-command '(:fail-only :fail-only-with-debug-log)) (not success)))
       (setq command-directory
	     (format nil "/tmp/irtmodel-ik-~A" (unix::getpid))
	     command-id ;; command-id including [robot name], [localtime], and [usec time]
             (let ((lt (unix::localtime)) (gtd (cadr (unix::gettimeofday))))
               (substitute #\0 #\  (format nil "~A-~04d-~02d-~02d-~02d-~02d-~02d-~A-~A-~A"
                                           (send (class self) :name) ;; name
                                           (+ 1900 (elt lt 5)) (+ 1 (elt lt 4)) (elt lt 3) (elt lt 2) (elt lt 1) (elt lt 0) ;; localtime
                                           (subseq (format nil "~0,6d" gtd) 0 2) (subseq (format nil "~0,6d" gtd) 2 4) (subseq (format nil "~0,6d" gtd) 4) ;; usec
                                           )))
             command-filename (format nil "~A/~A-~A.l" command-directory command-id (if success "success" "failure")))
       (unix::mkdir command-directory)
       (with-open-file
	(f command-filename :direction :output)
        (format f "~A" initial-log-str)
        (dump-structure f initial-target-coords)
        (dump-structure f initial-av)
        )
       (if (memq dump-command '(:fail-only-with-debug-log :always-with-debug-log)) ;; For ik debug information log
           (with-open-file
            (f (format nil "~A/~A-~A-debug-log.l" command-directory command-id (if success "success" "failure")) :direction :output)
            (format f "(setq *ik-loop* ~A~%" loop)
            (format f "      *ik-target-error* '~A~%" (mapcar #'(lambda (x) (append (list (car x)) (reverse (cdr x)))) (send self :get :ik-target-error)))
            (format f "      *ik-target-error-thre* '~A)~%"
                    (append
                     (let ((tmp-count -1)) (mapcar #'(lambda (tr rtr) (list (read-from-string (format nil ":target-~d" (incf tmp-count))) (list (* 1e-3 tr) rtr))) thre rthre)) ;; For target-coords threshold (thre[m], rthre[rad])
                     (if target-centroid-pos (list (list :centroid (* 1e-3 centroid-thre)))))) ;; For target-centroid-pos threshold [m]
            (remprop self :ik-target-error)
            ))
       )
     ;; check solved or not
     (if success
         (send self :angle-vector)
       (progn
	 (when warnp
	   (warn ";; inverse-kinematics failed.~%")
           (dotimes (i (length move-target))
	     (warn ";; dif-pos : ~a/(~a/~a)~%" (elt dif-pos i) (norm (elt dif-pos i)) (elt thre i))
	     (warn ";; dif-rot : ~a/(~a/~a)~%" (elt dif-rot i) (norm (elt dif-rot i)) (elt rthre i)))
           (if target-centroid-pos
               (let ((dif-cog (send self :difference-cog-position target-centroid-pos :centroid-offset-func centroid-offset-func :translation-axis cog-translation-axis :update-mass-properties nil)))
                 (warn ";; cog-dif : ~a/(~a/~a)~%" dif-cog (norm dif-cog) centroid-thre)))
           (warn ";;  coords : ~a~%" 
                 (send (let ((p self)) (while (send p :parent) (setq p (send p :parent))) p) :worldcoords))
	   (warn ";;  angles : ~a~%" av0)
	   (if check-collision (warn ";;  collision : ~a~%" collision-status))
	   (warn ";;    args : ~a~%" (append (list target-coords) args))
	   (when dump-command
	     (let (i print-args command-init command-setup command-args)
               (labels (;; escape functions to dump structure
                        (escape-string ;; escape string like : "aa" => "\"aa\""
                         (name) ;; If string, escape it. Otherwise (=symbol), return original value.
                         (if (stringp name) (format nil "\"~A\"" name) name))
                        (escape-list ;; escape list like : (a) => (list a)
                         (lst idx)
                         (when (consp (elt lst idx))
                           (setf (elt lst idx) (append '(list) (elt lst idx))) ;; escape
                           (dotimes (i (length (elt lst idx))) (escape-list (elt lst idx) i)))) ;; traverse list
                        (escape-robot-link ;; escape link like : alink => (send r :link "alink-name")
                         (lst idx)
                         (when (and (derivedp (elt lst idx) bodyset-link) (send self :link (send (elt lst idx) :name))) ;; escape
                           (setf (elt lst idx) `(send r :link ,(escape-string (send (elt lst idx) :name)))))
                         (if (consp (elt lst idx))
                             (dotimes (i (length (elt lst idx))) (escape-robot-link (elt lst idx) i)))) ;; traverse list
                        ;; move target escaping
                        (transformation-list-until-links
                         (acoords)
                         (if (find (send acoords :parent) (send self :links))
                             (list acoords)
                           (append (transformation-list-until-links (send acoords :parent)) (list acoords))))
                        (get-move-target-transform-list
                         (mcoords)
                         (let ((move-target-transformation-list (transformation-list-until-links mcoords)))
                           `(let* ((p (send r :link ,(escape-string (send (send (car move-target-transformation-list) :parent) :name))))
                                   (mt (make-cascoords :coords
                                                       (send (send p :copy-worldcoords) :transform (make-coords :4x4 ,(send (send (send (car move-target-transformation-list) :parent) :transformation
                                                                                                          (car (last move-target-transformation-list))) :4x4))))))
                              (send p :assoc mt)
                              mt)))
                        ;; make deep copy list instead of (copy-list lst)
                        (deep-copy-list
                         (lst)
                         (mapcar #'(lambda (l)
                                     (if (consp l) (deep-copy-list l) l))
                                 lst)))
               (setq print-args (deep-copy-list args)) ;; generate deep copy list for destructive operation
               (dotimes (i (/ (length print-args) 2)) ;; escape list
                 (unless (eq (elt print-args (* 2 i)) :move-target) ;; neglect :move-target because move-target is fixed later
                   (escape-list print-args (+ 1 (* 2 i)))))
               (dotimes (i (length print-args)) (escape-robot-link print-args i)) ;; escape robot link
	       (dotimes (j (count :move-target print-args)) ;; escape move-target
		 (if (setq i (position :move-target print-args :count (1+ j)))
		     (cond ((atom (elt print-args (+ i 1)))
                            (setf (elt print-args (+ i 1)) (get-move-target-transform-list (elt print-args (+ i 1)))))
			   (t
			    (setf (elt print-args (+ i 1))
				  (append '(list)
					  (mapcar #'(lambda (x) (get-move-target-transform-list x)) (elt print-args (+ i 1)))))))))
	       (setq command-init `(instance ,(send (class self) :name) :init)
		     command-setup `(progn  (send r :newcoords (make-coords :4x4 ,(send self :4x4))) (mapc #'(lambda (j a) (send* j :joint-angle a ,joint-args)) (list . ,(mapcar #'(lambda (x) `(send r ,(intern (string-upcase x) *keyword-package*))) (send-all (remove-duplicates (append (send-all union-link-list :joint) joint-list)) :name))) ,(list 'quote av0)) (objects (list r))))
               (setq command-args ;; append target-coords and print-args
                     (append '(list)
                             (list `(,@(mapcar #'(lambda (x) `(make-coords :pos ,(send (if (functionp x) (funcall x) x) :worldpos) :rot ,(send (if (functionp x) (funcall x) x) :worldrot))) target-coords)))
                             (list :dump-command nil :debug-view t) print-args))
               (setf (elt command-args 1) (append '(list) (elt command-args 1))) ;; escape target-coords list
               ) ;; labels
	       (warn ";; command : ~a~%" `(let ((r ,command-init)) ,command-setup (send* r :inverse-kinematics ,command-args)))
	       (warn ";; dump debug command to ~A~%" command-filename)
	       (warn ";; (progn (load \"~A\")(ik-setup)(ik-check))~%" command-filename)
	       ;; dump
	       (with-open-file
		(f command-filename :direction :output  :if-exists :append)
		(format f "(defun ~A-setup () (let ((r ~A)) (setq *robot* r) ~A (objects (list *robot*))))~%" command-id command-init command-setup)
		(format f "(defun ~A-check () (let ((r *robot*)) (send* r :inverse-kinematics ~A)))~%" command-id command-args)
		(format f "(defun ik-setup () (~A-setup))~%" command-id)
		(format f "(defun ik-check () (~A-check))~%" command-id)
                (format f "(setq ik-failed '(")
                (format f "(:dif-pos . ~A) (:dif-rot . ~A~%)" dif-pos dif-rot)
                (if target-centroid-pos (format f "(:dif-cog . ~A)" (send self :difference-cog-position target-centroid-pos :centroid-offset-func centroid-offset-func :translation-axis cog-translation-axis :update-mass-properties nil)))
                (format f "))~%")
		)
	       ) ;;let
	     ) ;; dump
	   ) ;; warnp
	 (mapc #'(lambda (j a) (send* j :joint-angle a joint-args))
	       (remove-duplicates
		(append (send-all union-link-list :joint) joint-list)) av0)
         (if c0 (send self :newcoords c0))
	 nil))
     ))
  (:ik-convergence-check
   (success dif-pos dif-rot
    rotation-axis translation-axis thre rthre
    centroid-thre target-centroid-pos centroid-offset-func cog-translation-axis
    &key (update-mass-properties t))
   (dotimes (i (length dif-pos))
     (setq success (and success
			(if (elt translation-axis i) (< (norm (elt dif-pos i)) (elt thre i)) t)
			(if (elt rotation-axis i) (< (norm (elt dif-rot i)) (elt rthre i)) t))))
   (if target-centroid-pos
       (setq success (and success (send self :cog-convergence-check centroid-thre target-centroid-pos
                                        :centroid-offset-func centroid-offset-func
                                        :translation-axis cog-translation-axis
                                        :update-mass-properties update-mass-properties))))
   success)
  (:calc-vel-from-pos
   (dif-pos translation-axis
    &rest args
    &key (p-limit 100.0)
         (tmp-v0 (instantiate float-vector 0))
         (tmp-v1 (instantiate float-vector 1))
         (tmp-v2 (instantiate float-vector 2))
         (tmp-v3 (instantiate float-vector 3))
    &allow-other-keys)
   (let (vel-p)
     ;; limitation and unit system contvert
     ;; pos
     (if (> (norm dif-pos) p-limit)
	 (setq dif-pos (scale p-limit (normalize-vector dif-pos tmp-v3) tmp-v3)))
     (setq dif-pos (scale 0.001 dif-pos tmp-v3)) ;; scale [mm] -> [m]
     (setq vel-p (calc-dif-with-axis dif-pos translation-axis
				     tmp-v0 tmp-v1 tmp-v2))
     vel-p))
  (:calc-vel-from-rot
   (dif-rot rotation-axis
    &rest args
    &key (r-limit 0.5)
         (tmp-v0 (instantiate float-vector 0))
         (tmp-v1 (instantiate float-vector 1))
         (tmp-v2 (instantiate float-vector 2))
         (tmp-v3 (instantiate float-vector 3))
    &allow-other-keys)
   (let (vel-r)
     ;; limitation and unit system contvert
     ;; rot
     (if (> (norm dif-rot) r-limit)
	 (setq dif-rot (scale r-limit (normalize-vector dif-rot tmp-v3) tmp-v3)))
     (setq vel-r (calc-dif-with-axis dif-rot rotation-axis
				     tmp-v0 tmp-v1 tmp-v2))
     vel-r))
  ;; collision check methods
  (:collision-check-pairs
   (&key ((:links ls) (cons (car links) (all-child-links (car links)))))
   (let (pairs l neighbors)
     (while (setq l (pop ls))
       (setq neighbors (remove nil
                               (append
                                (send l :descendants)
                                (send l :child-links)
                                (list (send l :parent-link) (send l :parent)))))
       (dolist (l2 ls)
	 (if (not (memq l2 neighbors))
	     (push (cons l l2) pairs))
	 )
       )
     pairs))
  (:self-collision-check
   (&key (mode :all) (pairs (send self :collision-check-pairs)) (collision-func 'pqp-collision-check))
   (let ((cpairs) (col-count 0))
     (dolist (p pairs)
       (let ((colp (/= (funcall collision-func (car p) (cdr p)) 0)))
	 (when colp
	   (incf col-count)
	   (if (eq mode :first)
	       (return-from :self-collision-check p)
	     (push p cpairs)))
	 ))
     cpairs))
  (:calc-grasp-matrix
   (contact-points
    &key (ret (make-matrix 6 (* 6 (length contact-points))))
         (contact-rots (mapcar #'(lambda (r) (unit-matrix 3)) contact-points)))
   "Calc grasp matrix.
      Grasp matrix is defined as
       | E_3 0   E_3 0   ... |
       | p1x E_3 p2x E_3 ... |
    Arguments:
      contact-points is list of contact points[mm].
      contact-rots is list of contact coords rotation[rad].
      If contact-rots is specified, grasp matrix as follow:
       | R1    0  R2    0  ... |
       | p1xR1 R1 p2xR2 R2 ... |"
   (let* ((contact-matrices (mapcar #'(lambda (p r) (m* (outer-product-matrix (scale 1e-3 p)) r)) contact-points contact-rots)))
     (dotimes (c (length contact-points))
       (dotimes (i 3)
         (dotimes (j 3)
           (setf (aref ret i (+ (* c 6) j)) (aref (elt contact-rots c) i j))
           (setf (aref ret (+ 3 i) (+ (* c 6) 3 j)) (aref (elt contact-rots c) i j))
           (setf (aref ret (+ 3 i) (+ (* c 6) j)) (aref (elt contact-matrices c) i j))
           )))
     ret))
  ;; closed loop kinematics method
  (:inverse-kinematics-for-closed-loop-forward-kinematics
   (target-coords
    &rest args
    &key (move-target) (link-list)
         (move-joints-hook) (additional-weight-list)
         ;; closed loop params
         (constrained-joint-list) (constrained-joint-angle-list)
         (min-loop 2) ;; 2 is minimum loop count
    &allow-other-keys)
   "Solve inverse-kinematics for closed loop forward kinematics.
    Move move-target to target-coords with link-list.
    link-list loop should be close when move-target reachs target-coords.
    constrained-joint-list is list of joints specified given joint angles in closed loop.
    constrained-joint-angle-list is list of joint-angle for constrained-joint-list."
   ;; Set joint-angle
   (mapcar #'(lambda (jnt len)
               (send jnt :joint-angle len))
           constrained-joint-list constrained-joint-angle-list)
   ;; Solve IK
   (send* self :inverse-kinematics
          target-coords
          :move-target move-target :link-list link-list
          :move-joints-hook
          #'(lambda ()
              ;; Reset joint-angle during IK
              (mapcar #'(lambda (jnt len)
                          (send jnt :joint-angle len))
                      constrained-joint-list constrained-joint-angle-list)
              (if (functionp move-joints-hook) (funcall move-joints-hook))
              nil)
          ;; Set constrained-joint-list's weight 0 not to move joints
          :additional-weight-list
          (append additional-weight-list
                  (mapcar #'(lambda (jnt) (list (send jnt :child-link) 0.0)) constrained-joint-list))
          :min-loop min-loop
          args)
   )
  (:calc-jacobian-for-interlocking-joints
   (link-list &key (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Calculate jacobian to keep interlocking joint velocity same.
    d\theta_0 = d\theta_1 => [... 0 1 0 ... 0 -1 0 .... ][...d\theta_0...d\theta_1...]"
   (let* ((union-link-list (send self :calc-union-link-list link-list))
          (jl (send-all union-link-list :joint))
          (pairs (remove-if-not #'(lambda (pair) (and (find (car pair) jl) (find (cdr pair) jl))) interlocking-joint-pairs))
          (jacobi (make-matrix (length pairs) (send self :calc-target-joint-dimension union-link-list))))
     (mapcar #'(lambda (pair)
                 (let ((idx (reduce #'+ (send-all (subseq jl 0 (position (car pair) jl)) :joint-dof))))
                   (setf (aref jacobi (position pair pairs) idx) 1.0))
                 (let ((idx (reduce #'+ (send-all (subseq jl 0 (position (cdr pair) jl)) :joint-dof))))
                   (setf (aref jacobi (position pair pairs) idx) -1.0)))
             pairs)
     jacobi))
  (:calc-vel-for-interlocking-joints
   (link-list &key (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Calculate 0 velocity for keeping interlocking joint at the same joint angle."
   (let* ((jl (send-all (send self :calc-union-link-list link-list) :joint))
          (pairs (remove-if-not #'(lambda (pair) (and (find (car pair) jl) (find (cdr pair) jl))) interlocking-joint-pairs))
          (vel (instantiate float-vector (length pairs))))
     vel))
  (:set-midpoint-for-interlocking-joints
   (&key (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Set interlocking joints at mid point of each joint angle."
   (mapcar #'(lambda (pair)
               (let ((mid (* 0.5 (+ (send (car pair) :joint-angle) (send (cdr pair) :joint-angle)))))
                 (send (car pair) :joint-angle mid)
                 (send (cdr pair) :joint-angle mid)))
           interlocking-joint-pairs)
   )
  (:interlocking-joint-pairs
   ()
   "Interlocking joint pairs.
    pairs are (list (cons joint0 joint1) ... )
    If users want to use interlocking joints, please overwrite this method."
   )
  (:check-interlocking-joint-angle-validity
   (&key (angle-thre 0.001) ;; [deg]
         (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Check if all interlocking joint pairs are same values."
   (let ((ret
          (mapcar #'(lambda (pair)
                      (> angle-thre (abs (- (send (car pair) :joint-angle) (send (cdr pair) :joint-angle)))))
                  interlocking-joint-pairs)))
     (every #'identity ret)))
  )

(defun all-child-links (s &optional (pred #'identity))
  (append (mapcan #'(lambda (x)
		      (if (funcall pred x) (list x))) (send s :child-links))
	  (mapcan #'(lambda (x) (all-child-links x pred)) (send s :child-links))))

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
        ((:xm :ym :zm)
         dif)
	(t dif)))

(defun calc-target-joint-dimension (joint-list)
   (let ((n 0))
     (dolist (j joint-list)
       (incf n (send j :joint-dof)))
     n))

(defun calc-joint-angle-min-max-for-limit-calculation (j kk jamm)
  ;; fix unit system ;; [mm] -> [m], [deg] -> [rad]
  (cond
   ((vectorp (send j :joint-angle)) ;; multi-dof joint such as sphere-joint, *wheel-joint and 6dof-joint
    (setf (elt jamm 0) (elt (send j :angle-to-speed (send j :joint-angle)) kk))
    (setf (elt jamm 1) (elt (send j :angle-to-speed (send j :max-angle)) kk))
    (setf (elt jamm 2) (elt (send j :angle-to-speed (send j :min-angle)) kk)))
   (t ;; 1-dof joint such as rotational-joint and linear-joint
    (setf (elt jamm 0) (send j :angle-to-speed (send j :joint-angle)))
    (setf (elt jamm 1) (send j :angle-to-speed (send j :max-angle)))
    (setf (elt jamm 2) (send j :angle-to-speed (send j :min-angle))))
   )
  jamm)

(defun joint-angle-limit-weight (j-l &optional (res (instantiate float-vector (calc-target-joint-dimension j-l))))
  (let ((k 0) (kk 0) (jamm (float-vector 0 0 0)))
    (dotimes (i (calc-target-joint-dimension j-l))
      (let ((j (elt j-l k)))
	(calc-joint-angle-min-max-for-limit-calculation j kk jamm)
	(let ((jang (elt jamm 0)) (jmax (elt jamm 1)) (jmin (elt jamm 2)) (e (deg2rad 1)))
	  (if (vectorp (send j :joint-angle))
	      (when (>= (incf kk) (length (send j :joint-angle))) (setq kk 0) (incf k))
	    (incf k))
	  ;; limitation
	  (cond
	   ((and (eps= jang jmax e) (eps= jang jmin e)))
	   ((eps= jang jmax e)
	    (setq jang (- jmax e)))
	   ((eps= jang jmin e)
	    (setq jang (+ jmin e))))
	  ;; calculate weight
	  (cond
	   ((or (eq (send j :name) :torso-waist-p)
		(eq (send j :name) :waist-p))
	    (let* ((nang (cond
			  ((= 0.0 jmax) (- jmax e))
			  ((= 0.0 jmin) (+ jmin e))
			  (t 0.0)))
		   (wang (if (> jang nang) (abs jmax) (abs jmin))))
	      (setf (elt res i)
		    (abs (/ (* (expt (* 2 wang) 2) (* 2 (- jang nang)))
			    (* 4 (expt (- (* wang wang) (* jang jang)) 2)))))))
	   ((and (eps= jang jmax e) (eps= jang jmin e))
	    (setf (elt res i) *inf*))
	   (t
	    (let ((r (abs (/ (* (expt (- jmax jmin) 2) (- (* 2 jang) jmax jmin))
			     (* 4 (expt (- jmax jang) 2) (expt (- jang jmin) 2))))))
	      (if (and (> r 0) (< r 0)) (setq r 0.0)) ;; nan check
	      (setf (elt res i) r))))
	  ))) ;; dotimes
    res))

(defun joint-angle-limit-nspace (j-l &optional (res (instantiate float-vector (calc-target-joint-dimension j-l))))
  (let ((k 0) (kk 0) (jamm (float-vector 0 0 0)))
    (dotimes (i (calc-target-joint-dimension j-l))
      (let ((j (elt j-l k)))
	(calc-joint-angle-min-max-for-limit-calculation j kk jamm)
	(let ((jang (elt jamm 0)) (jmax (elt jamm 1)) (jmin (elt jamm 2)) (e (deg2rad 1)))
	  (if (vectorp (send j :joint-angle))
	      (when (>= (incf kk) (length (send j :joint-angle))) (setq kk 0) (incf k))
	    (incf k))
	  ;; calculate weight
	  (cond
	   ((or (eq (send j :name) :torso-waist-p)
		(eq (send j :name) :waist-p))
	    (let* ((nang (cond
			  ((= 0.0 jmax) (- jmax e))
			  ((= 0.0 jmin) (+ jmin e))
			  (t 0.0))))
	      (setf (elt res i)
		    (if (> jang nang)
			(/ (- nang jang) (- jmax nang))
		      (/ (- nang jang) (* 5.0 (- nang jmin)))))))
	   (t
	    (setf (elt res i) (/ (- (/ (+ jmax jmin) 2.0) jang)
				 (/ (- jmax jmin) 2.0)))))
	  (let ((r (* (if (plusp (elt res i)) 1 -1) (expt (elt res i) 2))))
	    (if (or (and (> r 0) (< r 0)) (eq r *inf*) (eq r *-inf*)) (setq r 0.0))
	    (setf (elt res i) r))
	  ))) ;; dotimes
    res))

;; calc jacobian from robot joints and obj virtual-joint
(defun calc-jacobian-from-link-list-including-robot-and-obj-virtual-joint
  (link-list move-target obj-move-target robot
   &key (rotation-axis '(t t)) (translation-axis '(t t))
        (fik (make-matrix ;; result is copied to fik
              (send robot :calc-target-axis-dimension rotation-axis translation-axis)
              (send robot :calc-target-joint-dimension link-list))))
  (let* ((robot-ll (mapcar #'(lambda (l) ;; link-list for object link
                               (remove-if-not #'(lambda (x) (member x (send robot :links))) l))
                           link-list))
         (obj-ll (remove nil ;; obj-move-target should corresponds to head of link-list
                         (mapcar #'(lambda (l) ;; link-list for robot link
                                     (remove-if #'(lambda (x) (member x (send robot :links))) l))
                                 link-list)))
         (robot-ll-len (send robot :calc-target-joint-dimension robot-ll)))
    (send robot :calc-jacobian-from-link-list
          obj-ll :col-offset robot-ll-len :fik fik
          :transform-coords move-target :move-target obj-move-target
          :rotation-axis rotation-axis :translation-axis translation-axis)
    (send robot :calc-jacobian-from-link-list
          robot-ll :move-target move-target :fik fik
          :rotation-axis rotation-axis :translation-axis translation-axis)
    (dotimes (i (car (array-dimensions fik)))
      (dotimes (j (- (cadr (array-dimensions fik)) robot-ll-len))
        (setf (aref fik i (+ robot-ll-len j)) (* -1.0 (aref fik i (+ robot-ll-len j))))))
    fik))

;; function for append virtual obj joint for object manipulation
;; arguments are (ll ul ot link-list target-coords robot)
;; in detail
;;   link-list, target-coords -> original link-list and target-coords
;;   joint-class, joint-args -> joint class and argument for joint's :init
;;   vplink -> parent-link for virtual joint
;;   vplink-coords, vclink-coords -> initial coords for virtual links
;; return
;;   link-list with object virtual link and move-target associated with virtual link
;; example formulation
;;   [\dot{x}_{arm1}]   [ J_{arm1}        0  J_{obj1} ][\dot{\theta}_{arm1}]
;;   [\dot{x}_{arm2}] = [        0 J_{arm2}  J_{obj2} ][\dot{\theta}_{arm2}]
;;                                                     [ \dot{\theta}_{obj}]
(defun append-obj-virtual-joint
  (link-list target-coords
             &key (joint-class 6dof-joint)
                  (joint-args) (vplink) (vplink-coords) (vclink-coords))
  (labels ((make-virtual-link
            (name coords)
            (instance bodyset-link :init (make-cascoords :coords coords)
                      :bodies (list (make-cube 10 10 10))
                      :name name :weight 0
                      :centroid #f(0 0 0) :inertia-tensor (make-matrix 3 3))))
    (let* (;; vclink's move-target
           ;; robot's end-effectors servoes obj-move-target and obj-move-target servoes robot's end-effectors
           (move-target
            (mapcar #'(lambda (_tc)
                        (make-cascoords
                         :coords (send _tc :copy-worldcoords)))
                    target-coords))
           ;; virtual link in order to drive target-coords, which is parent for 6dof-joint
           (vplink
            (if vplink
                vplink
              (make-virtual-link 'virtual-parent-link
                                 (if vplink-coords
                                     vplink-coords
                                     (if (= 1 (length target-coords))
                                         (send (car move-target) :copy-worldcoords)
                                       (apply #'midcoords 0.5 move-target))
                                     ))))
           ;; virtual link in order to drive target-coords, which is child for 6dof-joint
           (vclink (make-virtual-link 'virtual-child-link
                                      (if vclink-coords
                                          vclink-coords
                                        (send vplink :copy-worldcoords)))))
      (dolist (omt move-target) (send vclink :assoc omt))
      (send vplink :assoc vclink)
      (send-message vclink bodyset-link :add-joint
                    (instance* joint-class :init
                               :child-link vclink
                               :parent-link vplink
                               joint-args))
      (send vclink :add-parent-link vplink)
      (send vplink :add-child-links vclink)
      (list (mapcar #'(lambda (l) (append l (list vclink))) link-list)
            move-target))))

;; for multiple constraint
(defun append-multiple-obj-virtual-joint
  (link-list target-coords
   &key (joint-class '(6dof-joint))
        (joint-args '(nil))
        (vplink) (vplink-coords) (vclink-coords))
  (let ((ll link-list) (pl vplink) (pc vplink-coords) (ret))
    (mapcar
     #'(lambda (jc ja)
         (setq ret
               (append-obj-virtual-joint
                ll target-coords
                :joint-class jc
                :joint-args ja
                :vplink pl
                :vplink-coords pc
                :vclink-coords vclink-coords))
         (setq ll (car ret)
               pl (car (last (caar ret)))))
     joint-class joint-args)
     ret))

(defmacro with-difference-position-and-rotation
  (params &rest args)
  ;; params := (dif-pos dif-rot move-target
  ;;            target-coords :translation-axis x :rotation-axis x)
  (with-gensyms
   (move-target target-coords rotation-axis translation-axis)
   `(let ((,move-target ,(caddr params))
          (,target-coords ,(cadddr params))
          (,translation-axis ,(if (member :translation-axis params)
                                  (cadr (member :translation-axis params))
                                t))
          (,rotation-axis ,(if (member :rotation-axis params)
                               (cadr (member :rotation-axis params))
                             t)))
      (let ((,(car params) (send ,move-target :difference-position
                                 ,target-coords :translation-axis
                                 ,translation-axis))
            (,(cadr params) (send ,move-target :difference-rotation
                                  ,target-coords :rotation-axis
                                  ,rotation-axis)))
        ,@args))
   ))

(defmacro with-difference-positions-and-rotations
  (params &rest args)
  ;; params := (dif-pos dif-rot move-target
  ;;            target-coords :translation-axis x :rotation-axis x)
  (with-gensyms
   (move-target target-coords rotation-axis translation-axis)
   `(let* ((,move-target ,(caddr params))
           (,target-coords ,(cadddr params))
           (,translation-axis (if ,(cadr (member :translation-axis params))
                                  ,(cadr (member :translation-axis params))
                                (make-list (length ,(caddr params))
                                           :initial-element t)))
           (,rotation-axis (if ,(cadr (member :rotation-axis params))
                               ,(cadr (member :rotation-axis params))
                             (make-list (length ,(caddr params))
                                        :initial-element t))))
      (let ((,(car params)
             (mapcar #'(lambda (m tc ta)
                         (send m :difference-position
                               tc :translation-axis ta))
                     ,move-target ,target-coords ,translation-axis))
            (,(cadr params)
             (mapcar #'(lambda (m tc ra)
                         (send m :difference-rotation
                               tc :rotation-axis ra))
                     ,move-target ,target-coords ,rotation-axis)))
        ,@args))
   ))

;; macro for IK with move-target and link-list
(defmacro with-move-target-link-list
  (params &rest bodies)
  ;; params := (move-target link-list robot limbs;; necessary parameter
  ;;            :move-target _move-target) ;; additional parameter
  ;; if use :move-target option, _move-target is used.
  ;; otherwise, :end-coords is used.
  (with-gensyms
   (move-target link-list robot limbs)
   `(let* ((,robot ,(caddr params))
           (,limbs (if (atom ,(cadddr params))
                       (list ,(cadddr params))
                     ,(cadddr params)))
           (,move-target (cond
                          (,(cadr (member :move-target params))
                           (if (atom ,(cadr (member :move-target params)))
                               (list ,(cadr (member :move-target params)))
                             ,(cadr (member :move-target params))))
                          (t
                           (mapcar #'(lambda (_limb) (send ,robot _limb :end-coords)) ,limbs))))
           (,link-list (mapcar
                        #'(lambda (_move-target)
                            (send ,robot :link-list
                                  (send _move-target :parent)))
                        ,move-target)))
      (let ((,(car params) ,move-target)
            (,(cadr params) ,link-list))
        ,@bodies)
      )))

(defmacro with-append-root-joint (params &rest bodies)
  ;; Return value:
  ;;   list of link-list
  ;; Arguments:
  ;;   params := (output-link-list robot input-link-list :joint-class joint-class :joint-args joint-args)
  ;;     output-link-list, robot, input-link-list : Necessary parameters
  ;;     joint-class, joint-args : Additional parameter, which can be omitted
  ;;     If joint-class is omitted, 6dof-joint is used as root-joint.
  ;;     If joint-args is omitted, nil.
  ;;   bodies : any S-expressions.
  ;; Supported types of input-link-list:
  ;;   1. link-list
  ;;        input-link-list : (list link0 link1 ...)
  ;;        =>
  ;;        output-link-list : (list root-link link0 link1 ...)
  ;;   2. list of link-list (without "nil")
  ;;        input-link-list : (list (list link00 link01 ...) (list link10 link11 ...) ...)
  ;;        =>
  ;;        output-link-list : (list (list root-link link00 link01 ...) (list root-link link10 link11 ...) ...)
  ;;   3. list of link-list including "nil", where "nil" derived from move-target at the root-link
  ;;        input-link-list : (list nil (list link00 link01 ...) ...)
  ;;        =>
  ;;        output-link-list : (list (list root-link) (list root-link link00 link01 ...) ...)
  ;; More info:
  ;;   Please see irtrobot.l's :fullbody-inverse-kinematics for usage and arguments.
  ;;   Please see test/test-irt-motion.l for testing and types of input-link-list.
  (with-gensyms
   (robot link-list vlink rlink joint-args)
   `(progn
      (let ((,robot ,(cadr params))
            (,link-list
             (if (every #'listp ,(caddr  params)) ;; Check all is list or not. Note that both (list link-list-0 link-list-1) and (list link-list-0 nil) is "t" at this check.
                 ,(caddr  params)
               (list ,(caddr  params))))
            (,joint-args ,(cadr (member :joint-args params))))
        (let ((,vlink (instance bodyset-link :init (make-cascoords)
                                :bodies (list (make-cube 150 10 400))
                                :name 'virtual-link
                                :weight 0 :centroid (float-vector 0 0 0)
                                :inertia-tensor (make-matrix 3 3)))
              (,rlink (car (send ,robot :links))))
          (send-message ,rlink bodyset-link :add-joint
                        (instance* ,(or (cadr (member :joint-class params)) 6dof-joint) :init
                                   :child-link ,robot
                                   :parent-link ,vlink
                                   ,joint-args))
          (send ,rlink :add-parent-link ,vlink)
          (send ,vlink :add-child-links ,rlink)
          (unwind-protect
              (let ((,(car params) (mapcar #'(lambda (l)
                                               (cons ,rlink l))
                                           ,link-list)))
                ,@bodies)
            (send ,rlink :del-joint)
            (send ,rlink :del-parent-link)
            (send ,vlink :del-child-link ,rlink)))))))

;; macro for IK with associating move-target
;; usage
;;  (with-assoc-move-target (mt :move-taget ... :parent-link ...) (ik-codes... :move-target mt))
;;  mt is symbol name for move-target variable
(defmacro with-assoc-move-target (params &rest bodies)
  (with-gensyms
   (new-move-target parent-link)
   `(let ((,new-move-target (if (atom ,(cadr (member :move-target params)))
                                (list ,(cadr (member :move-target params)))
                              ,(cadr (member :move-target params))))
          (,parent-link (if (atom ,(cadr (member :parent-link params)))
                            (list ,(cadr (member :parent-link params)))
                          ,(cadr (member :parent-link params)))))
      (setq ,new-move-target
            (mapcar #'(lambda (tmp-new-move-target)
                        (make-cascoords :coords
                                        (send tmp-new-move-target :copy-worldcoords)))
                    ,new-move-target))
      (mapcar #'(lambda (tmp-parent-link tmp-new-move-target)
                  (send tmp-parent-link :assoc tmp-new-move-target)) ,parent-link ,new-move-target)
      (unwind-protect
          (let ((,(car params) ,new-move-target)) ,@bodies)
        (mapcar #'(lambda (tmp-move-target)
                    (send (send tmp-move-target :parent) :dissoc tmp-move-target))
                ,new-move-target)
        ))))

(defun eusmodel-validity-check-one (robot)
  ;; root-link-validity-check
  (let ((root-link (car (send robot :links))))
    (assert (null (send root-link :parent-link))
	    (format nil "root link should have no parent-link!! ~A ~A~%" root-link (send root-link :parent-link)))
    (assert (equal robot (send root-link :parent))
            (format nil "root link ~A should be :assoced with robot ~A!!~%" root-link robot))
    )

  ;;link-joint-length-check-for-serial-link-manipulator
  (let ((joint-list
         (remove-duplicates
          (append (mapcar #'cdr (remove-if-not #'(lambda (s) (derivedp (cdr s) joint)) (send robot :slots)))
                  (send robot :joint-list))))
	(links (remove-duplicates
               (append (mapcar #'cdr (remove-if-not #'(lambda (s) (derivedp (cdr s) bodyset-link)) (send robot :slots)))
                       (send robot :links)))))
    (assert (= (+ (length joint-list) 1) (length links))
	    (format nil ";; link(~A) = joint(=~A) + 1 <- for serial link manipulator!!~%" (length links) (length joint-list)))
    ;; chain-validity-check
    (dolist (j joint-list)
      ;; joint should have child-link derived from bodyset-link class
      (assert (and (send j :child-link) (derivedp (send j :child-link) bodyset-link))
	      (format nil "joint should have child-link derived from bodyset-link class ~A ~A" j (send j :child-link)))
      ;; joint should have parent-link derived from bodyset-link class
      (assert (and (send j :parent-link) (derivedp (send j :parent-link) bodyset-link))
	      (format nil "child-link should associated with parent-link ~A ~A"
		      j (send j :parent-link)))
      ;; definition of child-link and parent-link should consistent in joints and links
      (assert (and (member (send j :child-link) (send (send j :parent-link) :descendants))
		   (equal (send j :parent-link) (send (send j :child-link) :parent)))
	      (format nil "definition of child-link and parent-link should consistent in joints and links ~A ~A ~A" j (send j :parent-link) (send j :child-link)))
      ;; definition of child-link and parent-link should consistent in joints and links
      (assert (and (equal (send (send j :child-link) :parent-link) (send j :parent-link))
		   (member (send j :child-link) (send (send j :parent-link) :child-links)))
	      (format nil "definition of child-link and parent-link should consistent in joints and links ~A ~A ~A" j (send j :child-link) (send j :parent-link)))
      )))

(defun eusmodel-validity-check (robot)
  "Check if the robot model is validate"
  (require :unittest "lib/llib/unittest.l")
  (init-unit-test)
  (setq lisp::*exit-on-fatal-error* nil)
  (eval `(deftest eusmodel-validity-check-test
	   (eusmodel-validity-check-one ,robot)))
  (run-all-tests)
  )

(in-package "GEOMETRY")

(provide :irtmodel "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.130  2010-03-09 15:28:35  k-okada
;;; special value for :waist-p
;;;
;;; Revision 1.129  2010/02/03 08:45:53  nozawa
;;; fix > to >= ;; original paper is if d|dH/dt| >= 0
;;;
;;; Revision 1.128  2010/01/26 17:29:24  eus
;;; :collision-avoidance fix if sentence
;;;
;;; Revision 1.127  2010/01/15 09:03:35  nozawa
;;; remove unused mode :ik-failed and replace unused return-value ret -> t in :move-joints-avoidance and :move-joints
;;;
;;; Revision 1.126  2010/01/15 07:11:06  nozawa
;;; add default value for thre, rthre, rotation-axis and translation-axis
;;;
;;; Revision 1.125  2010/01/15 01:33:43  k-okada
;;; use :calc-union-link-list in :inverse-kinematics
;;;
;;; Revision 1.124  2010/01/14 15:07:14  k-okada
;;; :calc-vel-from-dif-pos-rot -> calc-vel-from-dif/calc-vel-from-rot
;;;
;;; Revision 1.123  2010/01/14 13:51:19  k-okada
;;; fix :collision-avoidance-link-pair-from-link-list
;;;
;;; Revision 1.122  2010/01/14 12:10:00  k-okada
;;; change :  q = f(d) qca + {1-f(d)} J# b + N W y
;;;
;;; Revision 1.121  2010/01/14 12:06:18  k-okada
;;; fix :collision-avoidance-link-pair-from-link-list
;;;
;;; Revision 1.120  2010/01/14 11:47:59  k-okada
;;; fix global variables
;;;
;;; Revision 1.119  2010/01/14 11:45:42  k-okada
;;; draw min-distance collision-pair in red
;;;
;;; Revision 1.118  2010/01/14 11:42:15  k-okada
;;; add col-dav when twe collision-link-pair has same min-distance
;;;
;;; Revision 1.117  2010/01/14 09:40:12  k-okada
;;; update debug message
;;;
;;; Revision 1.116  2010/01/14 09:22:56  k-okada
;;; use [deg] unit for debug message
;;;
;;; Revision 1.115  2010/01/14 09:17:43  k-okada
;;; add debug message
;;;
;;; Revision 1.114  2010/01/14 08:49:47  k-okada
;;; add calc-union-link-list
;;;
;;; Revision 1.113  2010/01/14 08:27:54  k-okada
;;; add debug message
;;;
;;; Revision 1.112  2009/12/31 14:33:28  nozawa
;;; fix count of row in :calc-jacobian-from-link-list ;; in previous revision r1.111, link-list which length is over 3 doesn't work
;;;
;;; Revision 1.111  2009/12/27 09:13:41  ueda
;;; add utility macro with-difference-position-and-rotation and with-difference-positions-and-rotations
;;;
;;; Revision 1.110  2009-12-27 08:56:03  ueda
;;; remove fix-targets support...
;;;
;;; Revision 1.109  2009-12-27 08:52:45  ueda
;;; remove look-at-target from inverse-kinematics-loop and add :look-at codes to :inverse-kinematics for compatibility
;;;
;;; Revision 1.108  2009-12-27 08:46:18  ueda
;;; IMPORTANT commit. change arguments of inverse-kinematics-loop from coordinates to position and rotation velicity
;;;
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
