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
(require :irtmodel)

(defmethod joint
  (:calc-inertia-matrix (&rest args)
    (warn "subclass's respoinsibility (send ~s :calc-inertia-matrix)~%" self)))

;; Calculation of the inertia matrices is mainly based on the following paper.
;; S.Kajita, F.Kanehiro, K.Kaneko, K.Fujiwara, K.Harada, K.Yokoi, H.Hirukawa:
;; "Resolved Momentum Control:Humanoid Motion Planning based on the Linear and Angular Momentum"
;; In IROS 2003.

(defun calc-inertia-matrix-rotational
  (mat row column ;; output matrix
       paxis m-til c-til I-til axis-for-angular child-link world-default-coords
       translation-axis rotation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m) ;; buffer
  (let* ((ax (normalize-vector (send world-default-coords :rotate-vector paxis tmp-va) tmp-va))
	 (mt (* 1e-3 m-til)) ;; [g] -> [kg]
	 (ct (scale 1e-3 c-til tmp-vb)) ;; [mm] -> [m]
	 (It (scale-matrix 1e-9 I-til tmp-m)) ;; [g mm^2] -> [kg m^2]
	 (mj (scale mt
		    (v* ax (v- ct (scale 1e-3 (send child-link :worldpos) tmp-vc) tmp-vc)
			tmp-vd) tmp-vc))
	 (hj (v- (v+ (v* ct mj tmp-vd) (transform It ax tmp-vb) tmp-vb)
		 (v* (scale 1e-3 axis-for-angular tmp-va) mj tmp-vd) tmp-vb)))
    (let ((mv (calc-dif-with-axis mj translation-axis tmp-v0 tmp-v1 tmp-v2)))
      (dotimes (i (length mv)) (setf (aref mat (+ row i) column) (elt mv i)))
      (let ((hv (calc-dif-with-axis hj rotation-axis tmp-v0 tmp-v1 tmp-v2)))
	(dotimes (i (length hv)) (setf (aref mat (+ row i (length mv)) column) (elt hv i)))
	))))

(defun calc-inertia-matrix-linear
  (mat row column ;; output matrix
       paxis m-til c-til I-til axis-for-angular child-link world-default-coords
       translation-axis rotation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m) ;; buffer
  (let* ((ax (normalize-vector (send world-default-coords :rotate-vector paxis tmp-va) tmp-va))
	 (mt (* 1e-3 m-til)) ;; [g] -> [kg]
	 (ct (scale 1e-3 c-til tmp-vb)) ;; [mm] -> [m]
	 (It (scale-matrix 1e-9 I-til tmp-m)) ;; [g mm^2] -> [kg m^2]
	 (mj (scale mt ax tmp-va))
	 (hj (v- (v* ct mj tmp-vc) (v* (scale 1e-3 axis-for-angular tmp-vb) mj tmp-vd) tmp-vb)))
    (let ((tv (calc-dif-with-axis mj translation-axis tmp-v0 tmp-v1 tmp-v2)))
      (dotimes (i (length tv)) (setf (aref mat (+ row i) column) (elt tv i)))
      (let ((rv (calc-dif-with-axis hj rotation-axis tmp-v0 tmp-v1 tmp-v2)))
	(dotimes (i (length rv)) (setf (aref mat (+ row i (length rv)) column) (elt rv i)))
	))))

(defmethod rotational-joint
  (:calc-inertia-matrix
    (mat row column ;; output matrix
	 paxis m-til c-til I-til axis-for-angular world-default-coords
	 translation-axis rotation-axis
	 tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row column
     paxis m-til c-til I-til axis-for-angular child-link world-default-coords
     translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m))
  )

(defmethod linear-joint
  (:calc-inertia-matrix
    (mat row column ;; output matrix
	 paxis m-til c-til I-til axis-for-angular world-default-coords
	 translation-axis rotation-axis
	 tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row column
     paxis m-til c-til I-til axis-for-angular child-link world-default-coords
     translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m))
  )

(defmethod omniwheel-joint
  (:calc-inertia-matrix
    (mat row column ;; output matrix
	 paxis m-til c-til I-til axis-for-angular world-default-coords
	 translation-axis rotation-axis
	 tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row column #f(1 0 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row (+ column 1) #f(0 1 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 2) #f(0 0 1) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    )
  )

(defmethod sphere-joint
  (:calc-inertia-matrix
    (mat row column ;; output matrix
	 paxis m-til c-til I-til axis-for-angular world-default-coords
	 translation-axis rotation-axis
	 tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row column #f(1 0 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 1) #f(0 1 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 2) #f(0 0 1) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    )
  )

(defmethod 6dof-joint
  (:calc-inertia-matrix
    (mat row column ;; output matrix
	 paxis m-til c-til I-til axis-for-angular world-default-coords
	 translation-axis rotation-axis
	 tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row column #f(1 0 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row (+ column 1) #f(0 1 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-linear
     mat row (+ column 2) #f(0 0 1) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 3) #f(1 0 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 4) #f(0 1 0) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    (calc-inertia-matrix-rotational
     mat row (+ column 5) #f(0 0 1) m-til c-til I-til axis-for-angular
     child-link world-default-coords translation-axis rotation-axis
     tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
    )
  )

(defmethod bodyset-link
  ;; user shuold not call :append-weight-no-update, :append-centroid-no-update, and :append-inertia-no-update directly because these methods are for :append-mass-properties
  (:append-weight-no-update
   (additional-weights
    &key (len (length additional-weights)))
   (let ((ret (send self :weight)))
     (dotimes (i len)
       (setq ret (+ (elt additional-weights i) ret)))
     ret))
  (:append-centroid-no-update
   (additional-weights
    additional-centroids
    self-centroid new-weight
    &key (tmp-va (float-vector 0 0 0))
         (tmp-vb (float-vector 0 0 0))
         (len (length additional-weights)))
   (scale
    (/ 1.0 new-weight)
    (let ((ret (scale (send self :weight) self-centroid tmp-vb)))
      (dotimes (i len)
	(v+ ret (scale (elt additional-weights i) (elt additional-centroids i) tmp-va) ret))
      ret)
    tmp-vb))
  (:append-inertia-no-update
   (additional-weights
    additional-centroids
    additional-inertias
    self-centroid new-centroid
    &key (tmp-ma (make-matrix 3 3))
         (tmp-mb (make-matrix 3 3))
         (tmp-mc (make-matrix 3 3))
	 (tmp-md (make-matrix 3 3))
	 (tmp-va (float-vector 0 0 0))
         (len (length additional-weights)))
   (labels
       ((DD (r)
	    (let ((r2 (outer-product-matrix r tmp-ma)))
	      (m* (transpose r2 tmp-mb) r2 tmp-mc))))
     (let ((ret (m* (m* (send self :worldrot) (send self :inertia-tensor) tmp-ma)
		    (transpose (send self :worldrot) tmp-mb)
		    tmp-md)))
       (m+ (scale-matrix
	    (send self :weight)
	    (DD (v- self-centroid new-centroid tmp-va)) tmp-ma)
	   ret ret)
       (dotimes (i len)
	 (m+ (elt additional-inertias i) ret ret)
	 (m+ (scale-matrix (elt additional-weights i)
			   (DD (v- (elt additional-centroids i) new-centroid tmp-va))
			   tmp-ma)
	     ret ret))
       ret)))
  ;; append mass properties of additional-links to self link
  (:append-mass-properties
   (additional-links
    &key (update t) ;; if update is nil, mass properties of self link is not updated.
         ;; buffers
         (tmp-va (float-vector 0 0 0))
         (tmp-vb (float-vector 0 0 0))
         (tmp-ma (make-matrix 3 3))
         (tmp-mb (make-matrix 3 3))
         (tmp-mc (make-matrix 3 3))
	 (tmp-md (make-matrix 3 3))
	 ;;   centroid and inertia-tensor : world
         (additional-weights
	  (send-all additional-links :weight))
	 (additional-centroids
	  (send-all additional-links :centroid))
	 (additional-inertias
	  (mapcar #'(lambda (x)
		      (m* (m* (send x :worldrot) (send x :inertia-tensor) tmp-ma)
			  (transpose (send x :worldrot) tmp-mb)))
		  additional-links))
	 (self-centroid (send self :centroid)))
   (let* ((len (length additional-links))
	  (new-weight
	   (send self :append-weight-no-update additional-weights :len len))
	  (new-centroid ;; <- tmp-vb's buffer
	   (send self :append-centroid-no-update additional-weights additional-centroids
		 self-centroid new-weight :tmp-va tmp-va :tmp-vb tmp-vb :len len))
	  (new-inertia ;; <- tmp-md's buffer
	   (send self :append-inertia-no-update
		 additional-weights additional-centroids
		 additional-inertias self-centroid new-centroid
		 :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc :tmp-md tmp-md :tmp-va tmp-va :len len)))
     ;; update mass properties of self links
     ;;   centroid and inertia-tensor : world -> local
     (when update
       (send self :weight new-weight)
       (setq acentroid (send self :inverse-transform-vector new-centroid acentroid tmp-va tmp-ma))
       (send self :inertia-tensor
	     (m* (m* (transpose (send self :worldrot) tmp-ma) new-inertia tmp-mb)
		 (send self :worldrot)
		 (send self :inertia-tensor))))
     (list new-weight new-centroid new-inertia)))
  ;; propagate mass properties
  ;;   unit system ;; [g] [mm]
  ;;   m-til -> the mass of all link structure driven by joint of this bodyset-link.
  ;;   c-til -> the COM of all link structure driven by joint of this bodyset-link.
  (:propagate-mass-properties
    (&key (debug-view nil)
          ;; buffers for calculation
          (tmp-va (instantiate float-vector 3))
	  (tmp-vb (instantiate float-vector 3))
	  (tmp-ma (make-matrix 3 3))
	  (tmp-mb (make-matrix 3 3))
	  (tmp-mc (make-matrix 3 3))
    &allow-other-keys)
    ;; weight [g], centroid [mm], inertia [g mm^2]
    (if child-links
	(progn
	  ;; propagation of m-til and c-til from children
	  (dolist (child child-links)
	    (send child :propagate-mass-properties :debug-view debug-view
		  :tmp-va tmp-va :tmp-vb tmp-vb
		  :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc))
	  ;; update m-til, c-til, and i-til
	  ;;  c-til and i-til are arleady copied by using tmp-vb and tmp-md buffer
	  (send self :put :m-til
		(car (send self :append-mass-properties
			   child-links
			   :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc :tmp-md (send self :get :i-til)
			   :tmp-va tmp-va :tmp-vb (send self :get :c-til)
			   :additional-weights (send-all child-links :get :m-til)
			   :additional-centroids (send-all child-links :get :c-til)
			   :additional-inertias (send-all child-links :get :i-til)
			   :self-centroid (send self :centroid) :update nil)))
	  )
      ;; if end-link
      (let ((cen (send self :centroid)))
	(dotimes (i 3) (setf (elt (send self :get :c-til) i) (elt cen i)))
	(send self :put :m-til (send self :weight))
	))
    (if debug-view
	(warn ";; joint = ~A ;; m-til = ~A[g], c-til = ~A[mm]~%"
	      (send self :name) (send self :get :m-til) (send self :get :c-til)))
    t)
  ;; calculation of one column inertia matrix
  (:calc-inertia-matrix-column
   (column
    &rest args
    &key (debug-view nil)
         (rotation-axis nil)
	 (translation-axis t)
         ((:inertia-matrix im))
	 (axis-for-angular (float-vector 0 0 0))
	 ;; buffers for calculation
	 (tmp-v0 (instantiate float-vector 0))
	 (tmp-v1 (instantiate float-vector 1))
	 (tmp-v2 (instantiate float-vector 2))
	 (tmp-va (instantiate float-vector 3))
	 (tmp-vb (instantiate float-vector 3))
	 (tmp-vc (instantiate float-vector 3))
	 (tmp-vd (instantiate float-vector 3))
	 (tmp-ma (make-matrix 3 3))
    &allow-other-keys)
   ;; calculation of the column of inertia matrix
   (let* ((paxis (case (joint . axis)
		       (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
		       (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
		       (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
		       (t (joint . axis))))
	  (row 0)
	  (world-default-coords (send (send (send joint :parent-link) :copy-worldcoords)
				      :transform (joint . default-coords))))
     (send joint :calc-inertia-matrix im row column
	   paxis (send self :get :m-til) (send self :get :c-til) (send self :get :I-til)
	   axis-for-angular world-default-coords translation-axis rotation-axis
	   tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-ma))
   t)
  )

;; recursive computation of COG jacobian, inertia matrix
;;  inertia matrix and COG jacobian is at the world coordinates
;;  link-list should be union-link-list (cannot use list of link-list)
(defmethod cascaded-link
  ;; return   ;; inertia-matrix [m * kg]
  (:calc-inertia-matrix-from-link-list
    (&rest args
     &key (link-list (send-all joint-list :child-link))
          (rotation-axis nil) (translation-axis t)
          (axis-dim (send self :calc-target-axis-dimension rotation-axis translation-axis))
	  (inertia-matrix
	   (make-matrix axis-dim (send self :calc-target-joint-dimension link-list)))
	  (debug-view nil)
	  (update-mass-properties t)
          ;; buffers for calculation
	  (tmp-v0 (instantiate float-vector 0))
	  (tmp-v1 (instantiate float-vector 1))
	  (tmp-v2 (instantiate float-vector 2))
	  (tmp-va (instantiate float-vector 3))
	  (tmp-vb (instantiate float-vector 3))
	  (tmp-vc (instantiate float-vector 3))
	  (tmp-vd (instantiate float-vector 3))
	  (tmp-ma (make-matrix 3 3))
	  (tmp-mb (make-matrix 3 3))
	  (tmp-mc (make-matrix 3 3))
     &allow-other-keys)
    (when update-mass-properties 
      ;; parameter initialization
      (dolist (l (send self :links))
	(send l :put :m-til 0)
	(dotimes (i 3)
	  (setf (elt (send l :get :c-til) i) 0.0))
	(dotimes (j 3)
	  (dotimes (i 3)
	    (setf (aref (send l :get :I-til) i j) 0.0))))
      ;; calc all links' mass properties
      (send* (car (send self :links)) :propagate-mass-properties args))
    ;; calc inertia matrix
    (let ((j))
      (do ((column 0 (+ column (send j :joint-dof)))
	   (l 0 (+ l 1)))
	  ((>= l (length link-list)))
	  (setq j (send (elt link-list l) :joint))
	  (send* (elt link-list l) :calc-inertia-matrix-column column :inertia-matrix inertia-matrix
		 :axis-for-angular (send (car (send self :links)) :get :c-til)
		 :translation-axis translation-axis :rotation-axis rotation-axis args)))
    inertia-matrix)
  ;; return   ;; COG jacobian [m]
  (:calc-cog-jacobian-from-link-list
    (&rest args
     &key (link-list (send-all joint-list :child-link))
          (rotation-axis nil) (translation-axis t)
          (axis-dim (send self :calc-target-axis-dimension rotation-axis translation-axis))
          (inertia-matrix
	   (make-matrix axis-dim (send self :calc-target-joint-dimension link-list)))
     &allow-other-keys)
    (let ((all-M (* 0.001 (reduce #'+ (send-all (send self :links) :weight))))) ;; [kg]
      (send* self :calc-inertia-matrix-from-link-list
	     :inertia-matrix inertia-matrix :link-list link-list args)
      ;; calc cog jacobian from inertia matrix
      (dotimes (i axis-dim)
	(dotimes (j (cadr (array-dimensions inertia-matrix)))
	  (setf (aref inertia-matrix i j) (/ (aref inertia-matrix i j) all-M))))
      inertia-matrix))
  (:cog-jacobian-balance-nspace
    (link-list
     &rest args
     &key (cog-gain 1.0)
          (translation-axis :z)
          (target-centroid-pos (send (car (send self :links)) :get :c-til))
     &allow-other-keys)
    (transform
     (send* self :calc-inverse-jacobian
	    (send* self :calc-cog-jacobian-from-link-list
		   :link-list link-list
		   :translation-axis translation-axis args)
	    args)
     (scale (* 0.001 cog-gain) ;; [mm] -> [m]
	    (calc-dif-with-axis (v- target-centroid-pos (send (car (send self :links)) :get :c-til))
				translation-axis))
     )
    )
  (:cog-convergence-check
   (centroid-thre target-centroid-pos)
   (> centroid-thre
      (norm
       (subseq (v- target-centroid-pos (send self :centroid)) 0 2))))
  )

;; recursive computation for inverse-dynamics
(defmethod joint
  (:calc-spacial-velocity-jacobian
    (&rest args)
    (warn "subclass's respoinsibility (send ~s :calc-spacial-velocity-jacobian)~%" self))
  (:calc-angular-velocity-jacobian
    (&rest args)
    (warn "subclass's respoinsibility (send ~s :calc-angular-velocity-jacobian)~%" self))
  (:calc-spacial-acceleration-jacobian
    (&rest args)
    (warn "subclass's respoinsibility (send ~s :calc-spacial-acceleration-jacobian)~%" self))
  (:calc-angular-acceleration-jacobian
    (&rest args)
    (warn "subclass's respoinsibility (send ~s :calc-angular-acceleration-jacobian)~%" self))
  )

(defmethod rotational-joint
  (:calc-spacial-velocity-jacobian
    (ax tmp-va tmp-vb) ;; return-value is set to tmp-vb
    (v* (scale 0.001 (send child-link :worldpos) tmp-va) ax tmp-vb)) ;; [m]
  (:calc-angular-velocity-jacobian
    (ax tmp-va) ;; return-value is set to tmp-va
    (dotimes (i 3) (setf (elt tmp-va i) (elt ax i)))
    tmp-va) ;; []
  (:calc-spacial-acceleration-jacobian
    (svj avj tmp-va tmp-vb) ;; return-value is set to tmp-vb
    (v+ (v* (send parent-link :angular-velocity) svj tmp-va)
	(v* (send parent-link :spacial-velocity) avj tmp-vb) tmp-vb)) ;; [m/s]
  (:calc-angular-acceleration-jacobian
    (avj tmp-va) ;; return-value is set to tmp-va
    (v* (send parent-link :angular-velocity) avj tmp-va)) ;; [rad/s]
  )

(defmethod linear-joint
  (:calc-spacial-velocity-jacobian
    (ax tmp-va tmp-vb) ;; return-value is set to tmp-vb
    (dotimes (i 3) (setf (elt tmp-vb i) (elt ax i)))
    tmp-vb) ;; []
  (:calc-angular-velocity-jacobian
    (ax tmp-va) ;; return-value is set to tmp-va
    (dotimes (i 3) (setf (elt tmp-va i) 0.0))
    tmp-va) ;; []
  (:calc-spacial-acceleration-jacobian
    (svj avj tmp-va tmp-vb) ;; return-value is set to tmp-vb
    (v* (send parent-link :angular-velocity) svj tmp-vb)) ;; [rad/s]
  (:calc-angular-acceleration-jacobian
    (avj tmp-va) ;; return-value is set to tmp-va
    (dotimes (i 3) (setf (elt tmp-va i) 0.0))
    tmp-va) ;; []
  )

(defmethod bodyset-link
  (:reset-dynamics
   ()
   (send self :put :m-til 0)
   (send self :put :c-til (float-vector 0 0 0))
   (send self :put :I-til (make-matrix 3 3))
   (setq angular-velocity (float-vector 0 0 0) ;; [rad/s]
         angular-acceleration (float-vector 0 0 0) ;; [rad/s^2]
         spacial-velocity (float-vector 0 0 0) ;; [m/s]
         spacial-acceleration (float-vector 0 0 0) ;; [m/s^2]
         angular-momentum (float-vector 0 0 0) ;; [kg m^2/s]
         momentum (float-vector 0 0 0) ;; [kg m/s]
         angular-momentum-velocity (float-vector 0 0 0) ;; [kg m^2/s^2]
         momentum-velocity (float-vector 0 0 0) ;; [kg m^2/s^2]
         force (float-vector 0 0 0) ;; [N] = [kg m/s^2]
         moment (float-vector 0 0 0) ;; [Nm] = [kg m^2/s^2]
         ext-force (float-vector 0 0 0) ;; [N] = [kg m/s^2]
         ext-moment (float-vector 0 0 0))) ;; [Nm] = [kg m^2/s^2]
  (:angular-velocity () angular-velocity)
  (:angular-acceleration (&optional aa) (if aa (setq angular-acceleration aa) angular-acceleration))
  (:spacial-velocity () spacial-velocity)
  (:spacial-acceleration (&optional sa) (if sa (setq spacial-acceleration sa) spacial-acceleration))
  (:force () force)
  (:moment () moment)
  (:ext-force (&optional f) (if f (setq ext-force f) ext-force))
  (:ext-moment (&optional m) (if m (setq ext-moment m) ext-moment))
  (:forward-all-kinematics
   (&key (debug-view nil)
	 ;; buffers for calculation
	 (tmp-va (float-vector 0 0 0)))
   (if debug-view (format t ";; forward-all-kinematics link = ~A~%" (send self :name)))
   (when parent-link
     (let* ((paxis (case (joint . axis)
		     (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
		     (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
		     (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
		     (t (joint . axis))))
	    (ax (normalize-vector (send (send (send parent-link :copy-worldcoords) :transform
					      (joint . default-coords))
					:rotate-vector paxis tmp-va)
				  (send self :get :angular-velocity-jacobian))) ;; []
	    (svj (send joint :calc-spacial-velocity-jacobian ax tmp-va
		       (send self :get :spacial-velocity-jacobian)))
	    (avj (send joint :calc-angular-velocity-jacobian ax
		       (send self :get :angular-velocity-jacobian))))
       (setq angular-velocity ;; [rad/s]
	     (v+ (send parent-link :angular-velocity)
		 (scale (send joint :joint-velocity) avj tmp-va) angular-velocity)
	     spacial-velocity ;; [m/s]
	     (v+ (send parent-link :spacial-velocity)
		 (scale (send joint :joint-velocity) svj tmp-va) spacial-velocity))

       (let* ((saj (send joint :calc-spacial-acceleration-jacobian svj avj tmp-va spacial-acceleration)))
	 (setq spacial-acceleration ;; [m/s^2]
	       (v+ (v+ (send parent-link :spacial-acceleration)
		       (scale (send joint :joint-velocity) saj tmp-va) tmp-va)
		   (scale (send joint :joint-acceleration) svj spacial-acceleration) spacial-acceleration)))
       (let ((aaj (send joint :calc-angular-acceleration-jacobian avj tmp-va)))
	 (setq angular-acceleration ;; [rad/s^2]
	       (v+ (v+ (send parent-link :angular-acceleration)
		       (scale (send joint :joint-velocity) aaj tmp-va) tmp-va)
		   (scale (send joint :joint-acceleration) avj angular-acceleration) angular-acceleration)))
       ))
   (send-all child-links :forward-all-kinematics :debug-view debug-view :tmp-va tmp-va)
   )
  (:inverse-dynamics
   (&key (debug-view nil)
	 ;; buffers for computation
	 (tmp-va (float-vector 0 0 0))
	 (tmp-vb (float-vector 0 0 0))
	 (tmp-vc (float-vector 0 0 0))
	 (tmp-ma (make-matrix 3 3))
	 (tmp-mb (make-matrix 3 3))
	 (tmp-mc (make-matrix 3 3))
	 (tmp-md (make-matrix 3 3)))
   (if debug-view (format t ";; inverse-dynamics link = ~A~%" (send self :name)))
   (let* ((w (* 1e-3 (send self :weight))) ;; [g] -> [kg]
          (fg (scale (* -1.0 w 1e-3) *g-vec* tmp-va)) ;; [N]
	  (c (scale 1e-3 (send self :centroid) tmp-vb)) ;; [m]
	  (iner (m* (m* (send self :worldrot)
			(scale-matrix 1e-9 (send self :inertia-tensor) tmp-ma) tmp-mb)
		    (transpose (send self :worldrot) tmp-mc) tmp-ma)) ;; [g mm^2] -> [kg m^2]
	  (c-hat (outer-product-matrix c tmp-mb)) ;; [m]
	  (I (m+ iner (scale-matrix w (m* c-hat (transpose c-hat tmp-mc) tmp-md) tmp-mb) tmp-mb))) ;; [kg m^2]
     (setq momentum (scale w (v+ spacial-velocity (v* angular-velocity c tmp-vc) tmp-vc) momentum) ;; [kg m/s]
	   angular-momentum (v+ (scale w (v* c spacial-velocity tmp-vc) angular-momentum)
				(transform I angular-velocity tmp-vc) angular-momentum) ;; [kg m^2/s]
	   force (v+ (scale w (v+ spacial-acceleration
				  (v* angular-acceleration c tmp-vc) force) force)
		     (v* angular-velocity momentum tmp-vc) force) ;; [N]
           moment (v+ (v+ (v+ (scale w (v* c spacial-acceleration tmp-vc) moment)
			      (transform I angular-acceleration tmp-vc) moment)
			  (v* spacial-velocity momentum tmp-vc) moment)
		      (v* angular-velocity angular-momentum tmp-vc) moment)) ;; [N/m]
     ;; use ext-force and ext-moment
     ;; user must add moment around the origin caused by ext-force to ext-moment
     ;; i.e. ext-moment (in this method) <- ext-moment (user defined) + pos_{ext-force} x ext-force
     (setq force (v- force (v+ fg ext-force tmp-vc) force) ;; [N]
           moment (v- moment (v+ (v* c fg tmp-vc) ext-moment tmp-va) moment))) ;; [Nm]

   ;; propagation of force and moment from child-links
   (dolist (child child-links)
     (send child :inverse-dynamics :debug-view debug-view
	   :tmp-va tmp-va :tmp-vb tmp-vb :tmp-vc tmp-vc
	   :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc :tmp-md tmp-md)
     (setq force (v+ force (send child :force) force)
	   moment (v+ moment (send child :moment) moment)))

   (when (and joint parent-link)
     (send joint :joint-torque
	   (+ (v. (send self :get :spacial-velocity-jacobian) force)
	      (v. (send self :get :angular-velocity-jacobian) moment))))
   )
  )

(defmethod cascaded-link
  (:max-torque-vector
   ()
   (let ((ret (instantiate float-vector (calc-target-joint-dimension joint-list)))
         (i 0))
     (dolist (j joint-list)
       (dotimes (k (send j :joint-dof))
         (setf (elt ret i)
	       (case (send j :joint-dof)
		 (1 (send j :max-joint-torque))
		 (t (elt (send j :max-joint-torque) k))))
         (incf i)))
     ret))
  (:calc-torque
   (&key (debug-view nil)
	 (jvv (instantiate float-vector (length joint-list))) ;; [rad/s] or [m/s]
	 (jav (instantiate float-vector (length joint-list))) ;; [rad/s^2] or [m/s^2]
	 ;; buffers for computation
	 (tmp-va (float-vector 0 0 0))
	 (tmp-vb (float-vector 0 0 0))
	 (tmp-vc (float-vector 0 0 0))
	 (tmp-ma (make-matrix 3 3))
	 (tmp-mb (make-matrix 3 3))
	 (tmp-mc (make-matrix 3 3))
	 (tmp-md (make-matrix 3 3)))
   (let ((torque-vector (instantiate float-vector (length joint-list))))
     (dolist (l links)
       (send l :put :spacial-velocity-jacobian (float-vector 0 0 0))
       (send l :put :angular-velocity-jacobian (float-vector 0 0 0)))
     ;; joint-angle update
     (dotimes (i (length joint-list))
       (let* ((jnt (elt joint-list i)))
	 (send jnt :joint-velocity (elt jvv i))
	 (send jnt :joint-acceleration (elt jav i))
	 ))

     ;; recursive calculation from root-link
     (send (car (send self :links)) :forward-all-kinematics
           :debug-view debug-view :tmp-va tmp-va)
     (send (car (send self :links)) :inverse-dynamics
           :debug-view debug-view
	   :tmp-va tmp-va :tmp-vb tmp-vb :tmp-vc tmp-vc
	   :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc :tmp-md tmp-md)
     ;; reset ext force and ext moment
     (send-all (send self :links) :ext-force (float-vector 0 0 0))
     (send-all (send self :links) :ext-moment (float-vector 0 0 0))
     (dotimes (i (length torque-vector))
       (setf (elt torque-vector i) (send (elt joint-list i) :joint-torque)))
     torque-vector))

  ;; calculate Zero Moment Point based on Inverse Dynamics
  ;;   necessary arguments -> av and root-coords
  (:calc-zmp
    (&optional (av (send self :angle-vector))
               (root-coords (send (car (send self :links)) :copy-worldcoords))
     &key (pZMPz 0.0) ;; ZMP height [mm]
          (dt 0.005) ;; dt [s]
          (update t) (debug-view)
          (calc-torque-method :calc-torque)
          (calc-torque-args))
    (unless update (return-from :calc-zmp (send self :get :zmp)))

    ;; under no external forces and moments
    (dolist (l links)
      (send l :put :ext-force (float-vector 0 0 0))
      (send l :put :ext-moment (float-vector 0 0 0)))

    ;; set spacial-acceleration and angular-acceleration of root-link
    (let* ((dt-1 (/ 1.0 dt))
           (proot-coords
            (if (send self :get :prev-root-coords)
                (send self :get :prev-root-coords)
              root-coords))
           (rv (scale (* 1e-3 dt-1) (v- (send root-coords :worldpos) (send proot-coords :worldpos)))) ;; [m/s]
           (rw (scale dt-1 (send proot-coords :rotate-vector (send proot-coords :difference-rotation root-coords)))) ;; [rad/s]
           (prv (if (send self :get :prev-root-v)
                    (send self :get :prev-root-v)
                  rv)) ;; [m/s]
           (prw (if (send self :get :prev-root-w)
                    (send self :get :prev-root-w)
                  rw)) ;; [rad/s]
           (rva (scale dt-1 (v- rv prv))) ;; [m/s^2]
           (rwa (scale dt-1 (v- rw prw)))) ;; [rad/s^2]
      (send (car (send self :links)) :spacial-acceleration ;; [m/s^2]
            (v- rva (v+ (v* rwa (scale 1e-3 (send root-coords :worldpos))) ;; [rad/s^2] * [m]
                        (v* rw rv)))) ;; [rad/s] * [m/s]
      (send (car (send self :links)) :angular-acceleration rwa) ;; [rad/s^2]
      (send self :put :prev-root-coords root-coords)
      (send self :put :prev-root-v rv)
      (send self :put :prev-root-w rw))

    ;; set joint velocities and joint accelerations
    (let* ((dt-1 (/ 1.0 dt))
           (pav (if (send self :get :prev-av)
                    (send self :get :prev-av)
                  av))
           (jvv (scale dt-1 (map float-vector #'deg2rad (v- av pav)))) ;; [rad/s]
           (pjvv (if (send self :get :prev-jvv)
                     (send self :get :prev-jvv)
                   jvv)) ;; [rad/s]
           (jav (scale dt-1 (v- jvv pjvv)))) ;; [rad/s^2]
      (send* self calc-torque-method
             :dt dt :jvv jvv :jav jav
             :debug-view debug-view
             calc-torque-args)
      (send self :put :prev-av av)
      (send self :put :prev-jvv jvv))

    ;; calculate ZMP from force and moment of root-link
    (let* ((f0 (send (car (send self :links)) :force)) ;; [N]
           (n0 (send (car (send self :links)) :moment)) ;; [Nm], this moment is around the world origin, not around root link's origin.
           (f0x (elt f0 0)) (f0y (elt f0 1)) (f0z (elt f0 2))
           (n0x (elt n0 0)) (n0y (elt n0 1)) (n0z (elt n0 2))
           (pZMPz (* 1e-3 pZMPz)) ;; [m]
           (zmp (float-vector
                 (/ (+ (- n0y) (* pZMPz f0x)) f0z)
                 (/ (+ n0x (* pZMPz f0y)) f0z)
                 pZMPz))) ;; [m]
      (send self :put :zmp (scale 1e3 zmp))
      (send self :put :zmp-moment ;; yaw moment [Nm]
            (float-vector 0 0
                          (+ n0z
                             (* (* 1e-3 (elt (send self :get :zmp) 0)) f0y -1.0)
                             (* (* 1e-3 (elt (send self :get :zmp) 1)) f0x)))))
    (send self :get :zmp))

  ;; method to calculate fullbody mass properties
  (:weight ;; total weight
   (&optional (update-mass-properties t))
   (if update-mass-properties (send self :calc-inertia-matrix-from-link-list))
   (send (car (send self :links)) :get :m-til))
  (:centroid ;; total centroid
   (&optional (update-mass-properties t))
   (if update-mass-properties (send self :calc-inertia-matrix-from-link-list))
   (send (car (send self :links)) :get :c-til))
  )

(in-package "GEOMETRY")
(provide :irtdyna)
