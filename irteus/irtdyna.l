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
	(dotimes (i (length rv)) (setf (aref mat (+ row i (length tv)) column) (elt rv i)))
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
   (if (eps= (float new-weight) 0.0)
       self-centroid ;; if new-weight is 0, return self-centroid ;; in this case, this centroid value is not used latter calculation.
     (scale
      (/ 1.0 new-weight)
      (let ((ret (scale (send self :weight) self-centroid tmp-vb)))
	(dotimes (i len)
	  (v+ ret (scale (elt additional-weights i) (elt additional-centroids i) tmp-va) ret))
	ret)
      tmp-vb)))
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
	  (tmp-mc (make-matrix 3 3)))
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
        (m* (m* (send self :worldrot) (send self :inertia-tensor) tmp-ma)
            (transpose (send self :worldrot) tmp-mb)
            (send self :get :I-til))
	))
    (if debug-view
	(warn ";; joint = ~A ;; m-til = ~A[g], c-til = ~A[mm]~%"
	      (send self :name) (send self :get :m-til) (send self :get :c-til)))
    t)
  ;; calculation of one column inertia matrix
  (:calc-inertia-matrix-column
   (column
    &rest args
    &key (rotation-axis nil)
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
  (:update-mass-properties
   ;; buffers for calculation
   (&key (tmp-va (instantiate float-vector 3))
	 (tmp-vb (instantiate float-vector 3))
	 (tmp-ma (make-matrix 3 3))
	 (tmp-mb (make-matrix 3 3))
	 (tmp-mc (make-matrix 3 3)))
   ;; parameter initialization
   (all-child-links
    (car (send self :links))
    #'(lambda (l)
	(send l :put :m-til 0)
	(dotimes (i 3)
	  (setf (elt (send l :get :c-til) i) 0.0))
	(dotimes (j 3)
	  (dotimes (i 3)
	    (setf (aref (send l :get :I-til) i j) 0.0)))))
   ;; calc all links' mass properties
   (send (car (send self :links)) :propagate-mass-properties
	 :tmp-va tmp-va :tmp-vb tmp-vb :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc)
   )
  ;; return   ;; inertia-matrix [m * kg]
  (:calc-inertia-matrix-from-link-list
    (&rest args
     &key (link-list (send-all joint-list :child-link))
          (rotation-axis nil) (translation-axis t)
          (axis-dim (send self :calc-target-axis-dimension rotation-axis translation-axis))
	  (inertia-matrix
	   (make-matrix axis-dim (send self :calc-target-joint-dimension link-list)))
	  (update-mass-properties t)
          (axis-for-angular)
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
      (send self :update-mass-properties
	    :tmp-va tmp-va :tmp-vb tmp-vb :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc))
    ;; calc inertia matrix
    (let ((j))
      (do ((column 0 (+ column (send j :joint-dof)))
	   (l 0 (+ l 1)))
	  ((>= l (length link-list)))
	  (setq j (send (elt link-list l) :joint))
	  (send* (elt link-list l) :calc-inertia-matrix-column column :inertia-matrix inertia-matrix
                 ;; axis-for-angular is the axis for calculation of angular variables
                 ;; for example, we calculate angular momentum around the whole-body Center Of Mass as follows.
		 :axis-for-angular (if axis-for-angular axis-for-angular (send self :centroid nil))
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
          (update-mass-properties t)
     &allow-other-keys)
    (let ((all-M (* 0.001 (send self :weight update-mass-properties)))) ;; [kg]
      (send* self :calc-inertia-matrix-from-link-list
             :update-mass-properties nil
	     :inertia-matrix inertia-matrix :link-list link-list args)
      ;; calc cog jacobian from inertia matrix
      (dotimes (i axis-dim)
	(dotimes (j (cadr (array-dimensions inertia-matrix)))
	  (setf (aref inertia-matrix i j) (/ (aref inertia-matrix i j) all-M))))
      inertia-matrix))
  ;; cog jacobian methods   : :cog-jacobian-balance-nspace, :difference-cog-position, and :cog-convergence-check
  ;; return                 : angle velocity for COG tracking based on COG jacobian ;; (length dav) => (send self :calc-target-joint-dimension link-list))
  ;; arguments
  ;;   translation-axis     : use X and Y components of cog vector by default
  ;;   target-centroid-pos  : 3D vector of target centroid position
  ;;   centroid-offset-func : offset for centroid
  (:cog-jacobian-balance-nspace
    (link-list
     &rest args
     &key (cog-gain 1.0)
          (translation-axis :z)
          (target-centroid-pos)
	  (centroid-offset-func)
          (update-mass-properties t)
     &allow-other-keys)
    (transform
     (send* self :calc-inverse-jacobian
            (send* self :calc-cog-jacobian-from-link-list
                   :update-mass-properties update-mass-properties
		   :link-list link-list
		   :translation-axis translation-axis args)
	    args)
     (let ((cvel
            (send self :calc-vel-for-cog cog-gain translation-axis target-centroid-pos
                  :centroid-offset-func centroid-offset-func
                  :update-mass-properties nil)))
       (when (send self :get :ik-target-error)
         (push cvel (cdr (assoc :centroid (send self :get :ik-target-error)))))
       cvel)
     )
    )
  (:calc-vel-for-cog
   (cog-gain
    translation-axis
    target-centroid-pos
    &key (centroid-offset-func)
         (update-mass-properties t))
   (scale (* 0.001 cog-gain) ;; [mm] -> [m]
          (send self :difference-cog-position
                target-centroid-pos
                :centroid-offset-func centroid-offset-func
                :translation-axis translation-axis
                :add-draw-on-param t
                :update-mass-properties update-mass-properties)))
  (:difference-cog-position
    (target-centroid-pos &key (centroid-offset-func) (translation-axis :z) (add-draw-on-param) (update-mass-properties t))
    (let ((current-centroid-pos
           (if (functionp centroid-offset-func)
               (funcall centroid-offset-func)
             (send self :centroid update-mass-properties))))
      (if add-draw-on-param
          (send self :put :ik-draw-on-params
                (append (send self :get :ik-draw-on-params)
                        (list (list target-centroid-pos :color #f(0 0 1))
                              (list (float-vector (elt current-centroid-pos 0) (elt current-centroid-pos 1) (elt target-centroid-pos 2)) :size 100 :color #f(0 1 0))))))
      (calc-dif-with-axis
       (v- target-centroid-pos current-centroid-pos)
       translation-axis)))
  (:cog-convergence-check
   (centroid-thre target-centroid-pos &key (centroid-offset-func) (translation-axis :z) (update-mass-properties t))
   (let ((cdiff
            (send self :difference-cog-position target-centroid-pos
                  :centroid-offset-func centroid-offset-func :translation-axis translation-axis :update-mass-properties update-mass-properties)))
     (cond
      ((numberp centroid-thre) (> centroid-thre (norm cdiff)))
      ((functionp centroid-thre) (funcall centroid-thre cdiff))
      ((vectorp centroid-thre) (v< (map float-vector #'abs cdiff) centroid-thre)))))
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
  (:angular-velocity (&optional aa) (if aa (setq angular-velocity aa) angular-velocity))
  (:angular-acceleration (&optional aa) (if aa (setq angular-acceleration aa) angular-acceleration))
  (:spacial-velocity (&optional aa) (if aa (setq spacial-velocity aa) spacial-velocity))
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
   (when (and parent-link (derivedp (send self :parent) bodyset-link)) ;; exclude if root-link
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

   (when (and joint parent-link (derivedp (send self :parent) bodyset-link)) ;; exclude if root-link
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
  (:torque-ratio-vector
    (&rest args &key (torque (send* self :torque-vector args)))
    (let* ((mtq (send self :max-torque-vector))
           (ret (instantiate float-vector (length mtq))))
      (dotimes (i (length mtq))
        (setf (elt ret i) (/ (elt torque i) (elt mtq i))))
      ret))
  (:calc-torque-buffer-args
    ()
    ;; buffers for computation
    (list :tmp-va (float-vector 0 0 0)
          :tmp-vb (float-vector 0 0 0)
          :tmp-vc (float-vector 0 0 0)
          :tmp-ma (make-matrix 3 3)
          :tmp-mb (make-matrix 3 3)
          :tmp-mc (make-matrix 3 3)
          :tmp-md (make-matrix 3 3))
    )
  (:calc-torque
   (&key (debug-view nil)
         ;; statics or dynamics
         ;;   if t -> Calc statics. Do not update vel and acc for av and root-coords.
         ;;   otherwise -> Calc dynamics. Update vel and acc from current av and root-coords.
         (calc-statics-p t)
         (dt 0.005) ;; [sec]
         (av (send self :angle-vector))
         (root-coords (send (car (send self :links)) :copy-worldcoords))
         (force-list) (moment-list) (target-coords)
         (calc-torque-buffer-args (send self :calc-torque-buffer-args)))
   ;; check length
   (unless (= (length force-list) (length moment-list) (length target-coords))
     (error ";; ERROR : list length differ in :calc-torque : force-list ~A moment-list ~A target-coords ~A~%"
            (length force-list) (length moment-list) (length target-coords)))
   ;; set ext wrench
   (mapcar #'(lambda (fv mv tc)
               (send (send tc :parent) :ext-force fv)
               ;; calc moment-offset caused by fv ;
               ;; current irtdyna.l's requires moment around the origin
               (let* ((moment-offset (v* (scale 1e-3 (send tc :worldpos)) fv)))
                 (send (send tc :parent) :ext-moment (v+ mv moment-offset))))
           force-list moment-list target-coords)
   (send self :calc-torque-without-ext-wrench
              :debug-view debug-view
              :calc-statics-p calc-statics-p
              :av av :root-coords root-coords :dt dt
              :calc-torque-buffer-args calc-torque-buffer-args)
   )
  (:calc-torque-without-ext-wrench
   (&key (debug-view nil)
         (calc-statics-p t)
         (dt 0.005) ;; [sec]
         (av (send self :angle-vector))
         (root-coords (send (car (send self :links)) :copy-worldcoords))
         (calc-torque-buffer-args (send self :calc-torque-buffer-args)))
   (send* self :calc-torque-from-vel-acc
          :debug-view debug-view
          :calc-torque-buffer-args calc-torque-buffer-args
          (unless calc-statics-p
            (let* ((ret-rc (send self :calc-root-coords-vel-acc-from-pos dt root-coords))
                   (ret-av (send self :calc-av-vel-acc-from-pos dt av)))
              (list
               :jvv (cadr (memq :joint-velocity-vector ret-av))
               :jav (cadr (memq :joint-acceleration-vector ret-av))
               :root-spacial-velocity (cadr (memq :root-spacial-velocity ret-rc))
               :root-angular-velocity (cadr (memq :root-angular-velocity ret-rc))
               :root-spacial-acceleration (cadr (memq :root-spacial-acceleration ret-rc))
               :root-angular-acceleration (cadr (memq :root-angular-acceleration ret-rc))
               )))))
  (:calc-torque-from-vel-acc
   (&key (debug-view nil)
	 (jvv (instantiate float-vector (length joint-list))) ;; [rad/s] or [m/s]
	 (jav (instantiate float-vector (length joint-list))) ;; [rad/s^2] or [m/s^2]
         (root-spacial-velocity (float-vector 0 0 0))
         (root-angular-velocity (float-vector 0 0 0))
         (root-spacial-acceleration (float-vector 0 0 0))
         (root-angular-acceleration (float-vector 0 0 0))
         (calc-torque-buffer-args (send self :calc-torque-buffer-args)))
   (let ((torque-vector (instantiate float-vector (length joint-list)))
         (analysis-level-org (mapcar #'(lambda (l) (send l :analysis-level)) (send self :links))))
     (send-all (send self :links) :analysis-level :coords)
     (all-child-links
      (car (send self :links))
      #'(lambda (l)
          (send l :put :spacial-velocity-jacobian (float-vector 0 0 0))
          (send l :put :angular-velocity-jacobian (float-vector 0 0 0))))
     ;; joint-angle update
     (dotimes (i (length joint-list))
       (let* ((jnt (elt joint-list i)))
	 (send jnt :joint-velocity (elt jvv i))
	 (send jnt :joint-acceleration (elt jav i))
	 ))
     (send (car (send self :links)) :spacial-acceleration ;; [m/s^2]
           root-spacial-acceleration)
     (send (car (send self :links)) :angular-acceleration
           root-angular-acceleration) ;; [rad/s^2]
     (send (car (send self :links)) :spacial-velocity ;; [m/s]
           root-spacial-velocity)
     (send (car (send self :links)) :angular-velocity ;; [rad/s]
           root-angular-velocity)

     ;; recursive calculation from root-link
     (send* (car (send self :links)) :forward-all-kinematics
            :debug-view debug-view (subseq calc-torque-buffer-args 0 2))
     (send* (car (send self :links)) :inverse-dynamics
            :debug-view debug-view calc-torque-buffer-args)
     ;; reset ext force and ext moment
     (all-child-links
      (car (send self :links))
      #'(lambda (l)
          (send l :ext-force (float-vector 0 0 0))
          (send l :ext-moment (float-vector 0 0 0))))
     (dotimes (i (length torque-vector))
       (setf (elt torque-vector i) (send (elt joint-list i) :joint-torque)))
     (mapcar #'(lambda (l org) (send l :analysis-level org)) (send self :links) analysis-level-org)
     torque-vector))
  (:calc-root-coords-vel-acc-from-pos
   (dt root-coords)
   ;; set spacial-acceleration and angular-acceleration of root-link
   (labels ((omegafromrot
             (rot)
             (let ((alpha (* 0.5 (+ (aref rot 0 0) (aref rot 1 1) (aref rot 2 2) -1))))
               (if (eps= (abs (- alpha 1)) 0.0 1e-20)
                   (float-vector 0 0 0)
                 (let* ((th (acos alpha)) (s (sin th)))
                   (if (< s 1e-20)
                       (float-vector (sqrt (* th (* 0.5 (+ (aref rot 0 0) 1))))
                                     (sqrt (* th (* 0.5 (+ (aref rot 1 1) 1))))
                                     (sqrt (* th (* 0.5 (+ (aref rot 2 2) 1)))))
                     (let ((k (/ (* -0.5 th) s)))
                       (float-vector (* k (- (aref rot 1 2) (aref rot 2 1)))
                                     (* k (- (aref rot 2 0) (aref rot 0 2)))
                                     (* k (- (aref rot 0 1) (aref rot 1 0))))
                       ))
                   )))))
   (let* ((dt-1 (/ 1.0 dt))
          (proot-coords
           (if (send self :get :prev-root-coords)
               (send self :get :prev-root-coords)
             root-coords))
          ;;(rw (scale dt-1 (send proot-coords :rotate-vector (send proot-coords :difference-rotation root-coords)))) ;; [rad/s]
          (rw (scale dt-1 (send proot-coords :rotate-vector (omegafromrot (m* (transpose (send proot-coords :worldrot)) (send root-coords :worldrot)))))) ;; [rad/s]
          (rp (scale 1e-3 (send root-coords :worldpos)))
          (prp (scale 1e-3 (send proot-coords :worldpos)))
          (rv (if (eps= (norm rw) 0.0 5e-3)
                  (scale dt-1 (v- rp prp)) ;; [m/s]
                (v+
                 (v* rw rp)
                 (scale (norm rw)
                        (transform
                         (inverse-matrix
                          (m+ (m* (m- (unit-matrix 3) (matrix-exponent (scale dt rw))) (outer-product-matrix (normalize-vector rw)))
                              (scale-matrix (/ dt (norm rw))
                                            (m*
                                             (transpose (make-matrix 1 3 (list rw)))
                                             (make-matrix 1 3 (list rw))))))
                         (v- rp (transform (matrix-exponent (scale dt rw)) prp)))))))
          (prv (if (send self :get :prev-root-v)
                   (send self :get :prev-root-v)
                 rv)) ;; [m/s]
          (prw (if (send self :get :prev-root-w)
                   (send self :get :prev-root-w)
                 rw)) ;; [rad/s]
          (rwa (scale dt-1 (v- rw prw))) ;; [rad/s^2]
          ;; first order approximation
          (sp-rva (v- (scale dt-1 (v- rv (transform (matrix-exponent (scale dt rw)) prv)))
                      (v* rwa rp)))
          ;;(sp-rva (v- (v- (scale dt-1 (v- rv prv)) (v* rwa rp)) (v* rw rv)))
          ) ;; [m/s^2]
     (send self :put :prev-root-coords (send root-coords :copy-worldcoords))
     (send self :put :prev-root-v rv)
     (send self :put :prev-root-w rw)
     (list :root-spacial-velocity (v- rv (v* rw (scale 1e-3 (send root-coords :worldpos)))) ;; [m/s]
           :root-angular-velocity rw ;; [rad/s]
           :root-spacial-acceleration sp-rva ;; [m/s^2]
           :root-angular-acceleration rwa) ;; [rad/s^2]
     )))
  (:calc-av-vel-acc-from-pos
   (dt av)
   ;; set joint velocities and joint accelerations
   (let* ((dt-1 (/ 1.0 dt))
          (pav (if (send self :get :prev-av)
                   (send self :get :prev-av)
                 av))
          (jvv (let ((jr (instantiate float-vector (length av))))
                 (dotimes (i (length joint-list))
                   (setf (elt jr i) (* dt-1 (send (elt joint-list i) :angle-to-speed (- (elt av i) (elt pav i))))))
                 jr))
          (pjvv (if (send self :get :prev-jvv)
                    (send self :get :prev-jvv)
                  jvv)) ;; [rad/s]
          (jav (scale dt-1 (v- jvv pjvv)))) ;; [rad/s^2]
     (send self :put :prev-av av)
     (send self :put :prev-jvv jvv)
     (list :joint-velocity-vector jvv
           :joint-acceleration-vector jav)
     ))
  (:calc-torque-from-ext-wrenches
   (&key (force-list) (moment-list) (target-coords) ((:jacobi tmp-jacobi)))
   (let* ((link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords))
          (jacobi
           (if tmp-jacobi
               tmp-jacobi
             (send self :calc-jacobian-from-link-list
                   link-list
                   :move-target target-coords
                   :transform-coords (mapcar #'(lambda (x) (make-coords)) target-coords)
                   :rotation-axis (make-list (length target-coords) :initial-element t)
                   :translation-axis (make-list (length target-coords) :initial-element t))
             ))
          (wrenches
           (mapcar #'(lambda (f m) (concatenate float-vector f m)) force-list moment-list))
          (tq (transform (transpose jacobi)
                         (apply #'concatenate float-vector wrenches)))
          (ret-tq (instantiate float-vector (length joint-list)))
          (ul (send self :calc-union-link-list link-list)))
     ;; tq = torque vector included in link-list
     ;; ret-tq = all torque vector in joint-list
     (dotimes (i (length ul))
       (setf (elt ret-tq (position (send (elt ul i) :joint) joint-list)) (elt tq i)))
     ret-tq))
  (:calc-zmp
    (&optional (av (send self :angle-vector))
               (root-coords (send (car (send self :links)) :copy-worldcoords))
     &key (pZMPz 0.0)
          (dt 0.005)
          (update t) (debug-view)
          (calc-torque-buffer-args (send self :calc-torque-buffer-args)))
    "Calculate Zero Moment Point based on Inverse Dynamics.
     The output is expressed by the world coordinates, 
     and depends on historical robot states of the past 3 steps. Step is incremented when this method is called.
     After solving Inverse Dynamics, ZMP is calculated from total root-link force and moment.
     necessary arguments -> av and root-coords.
     If update is t, call inverse dynamics, otherwise, just return zmp from total root-link force and moment.
     dt [s] is time step used only when update is t.
     pZMPz is ZMP height [mm].
     After this method, (send robot :get :zmp) is ZMP and (send robot :get :zmp-moment) is moment around ZMP."
    (when update
      ;; under no external forces and moments
      (all-child-links
       (car (send self :links))
       #'(lambda (l)
           (send l :ext-force (float-vector 0 0 0))
           (send l :ext-moment (float-vector 0 0 0))))

      (send self :calc-torque
            :dt dt :av av :root-coords root-coords
            :debug-view debug-view
            :calc-statics-p nil
            :calc-torque-buffer-args calc-torque-buffer-args))

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
  (:calc-cop-from-force-moment ;; calc cop at ee-coords
   (force moment ;; [N] [Nm], global force & moment
    sensor-coords cop-coords
    &key (fz-thre 1) (return-all-values))
   ;; cop-coords : reference coords for COP, pos,z=COP pos z, c^p_{c,z}=0, rot, z=normal vector
   ;; c^p_{c,x}=(c^f_{s,z} c^p_{s,x} - c^p_{s,z} c^f_{s,x} - c^n_{s,y})/c^f_{s,z}
   ;; c^p_{c,y}=(c^f_{s,z} c^p_{s,y} - c^p_{s,z} c^f_{s,y} + c^n_{s,x})/c^f_{s,z}
   ;; cop-coords relative values
   (let* ((cfs (send cop-coords :inverse-rotate-vector force)) ;; [N]
          (cms (send cop-coords :inverse-rotate-vector moment)) ;; [Nm]
          (cps (scale 1e-3 (send cop-coords :inverse-transform-vector
                                 (send sensor-coords :worldpos)))) ;; [mm]->[m]
          (tmpcopx (+ (* (elt cfs 2) (elt cps 0))
                      (- (* (elt cps 2) (elt cfs 0)))
                      (- (elt cms 1))))
          (tmpcopy (+ (* (elt cfs 2) (elt cps 1))
                      (- (* (elt cps 2) (elt cfs 1)))
                      (elt cms 0)))
          (tmpfz (elt cfs 2)))
     (cond
      ((< tmpfz fz-thre) nil)
      (return-all-values
       (list :cop
             (send cop-coords :transform-vector
                   (scale 1e3 (float-vector (/ tmpcopx tmpfz) (/ tmpcopy tmpfz) 0))) ;; [m]->[mm]
             :fz tmpfz))
      (t (send cop-coords :transform-vector
               (scale 1e3 (float-vector (/ tmpcopx tmpfz) (/ tmpcopy tmpfz) 0)))) ;; [m]->[mm]
      )))
  ;; get list of N x force and N x moment from 6 x N wrench vector
  ;;   (list (list f1 ... fN) (list n1 ... nN)) <= [f1 n1 ... fN nN]
  (:wrench-vector->wrench-list
   (wrench-vector)
   (let ((f-list) (m-list))
     (dotimes (i (/ (length wrench-vector) 6))
       (push (subseq wrench-vector (* 6 i) (+ 3 (* 6 i))) f-list)
       (push (subseq wrench-vector (+ 3 (* 6 i)) (+ 6 (* 6 i))) m-list)
       )
     (list (reverse f-list) (reverse m-list))
     ))
  ;; get 6 x N wrench vector from list of N x force and N x moment
  ;;   [f1 n1 ... fN nN] <= (list (list f1 ... fN) (list n1 ... nN))
  (:wrench-list->wrench-vector
   (wrench-list)
   (apply
    #'concatenate
    float-vector
    (mapcar #'(lambda (f m)
                (concatenate float-vector f m))
            (car wrench-list) (cadr wrench-list))
    ))
  ;; calculate contact force from total wrench
  ;;  total wrench = total force[N] + total moment[Nm] around the origin
  ;;  return = contact wrenches at each contact points
  ;;           contact force [N] and contact moment [Nm] at contact points
  (:calc-contact-wrenches-from-total-wrench
   (target-pos-list ;; target-pos-list [mm]
    &key (total-wrench) (weight (fill (instantiate float-vector (* 6 (length target-pos-list))) 1)))
   (unless total-wrench ;; if total-wrench is not set, set default value
     (setq total-wrench ;; default wrench = wrench derived from weight-force
           (let* ((weight-force (* (send self :weight) 1e-6 (elt *g-vec* 2)))) ;; weight[g] * 1e-6 * gvec[mm/s^2] = force[N]
             (float-vector 0 0 weight-force
                           (* 1e-3 (elt (send self :centroid nil) 1) weight-force)
                           (* -1e-3 (elt (send self :centroid nil) 0) weight-force) 0))))
   ;; set default force & moment by solving mimimum internal forces
   (let ((ret-f
          (transform
           (send self :calc-inverse-jacobian
                 (send self :calc-grasp-matrix target-pos-list)
                 :weight weight)
           total-wrench))
         (force-list) (moment-list))
     (dotimes (i (length target-pos-list))
       (push (subseq ret-f (* i 6) (+ (* i 6) 3)) force-list)
       (push (subseq ret-f (+ (* i 6) 3) (+ (* i 6) 6)) moment-list)
       )
     (list (reverse force-list) (reverse moment-list))
     ))

  (:draw-torque
   (vwer &key flush (width 2) (size 100) (color (float-vector 1 0.3 0)) (warning-color (float-vector 1 0 0)) (torque-threshold nil)
              (torque-vector (send self :torque-vector)) ((:joint-list jlist) (send self :joint-list)))
   (mapcar
    #'(lambda (j r)
	(let* ((rtorque (/ r (send j :max-joint-torque)))
	       (paxis (send j :child-link :rotate-vector
			    (case (j . axis)
			      (:x #f(1 0 0)) (:y #f(0 1 0))
			      (:z #f(0 0 1)) (:-x #f(-1 0 0))
			      (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
			      (t (j . axis)))))
	       (rot-th (acos (v. (float-vector 0 0 1) paxis)))
	       (rot
		(if (eps= rot-th 0.0)
		    (unit-matrix 3)
		  (rotation-matrix
		   rot-th
		   (v* (float-vector 0 0 (if (> rtorque 0.0) 1.0 -1.0)) paxis))))
	       (default-width width) (default-color color))
	  (unless rot (setq rot (unit-matrix 3)))
	  (when (and torque-threshold (> (abs rtorque) torque-threshold))
	    (setq width (* 2 width) color warning-color))
	  (send vwer :viewsurface :line-width width)
	  (send vwer :viewsurface :color color)
	  (send vwer :draw-circle
		(make-coords :pos (send j :child-link :worldpos) :rot rot)
		:radius (* size (abs rtorque)) :arrow t :arc (deg2rad 330))
	  ))
    jlist
    (coerce torque-vector cons))
   (if flush (send vwer :viewsurface :flush))
   );; draw-torque

  ;; method to calculate fullbody mass properties
  (:weight ;; total weight
   (&optional (update-mass-properties t))
   "Calculate total robot weight [g].
    If update-mass-properties argument is t, propagate total mass prop calculation for all links and returns total robot weight.
    Otherwise, do not calculate total weight, just returns pre-computed total robot weight."
   (if update-mass-properties (send self :update-mass-properties))
   (send (car (send self :links)) :get :m-til))
  (:centroid ;; total centroid
   (&optional (update-mass-properties t))
   "Calculate total robot centroid (Center Of Gravity, COG) [mm] in euslisp world coordinates.
    If update-mass-properties argument is t, propagate total mass prop calculation for all links and returns total robot centroid.
    Otherwise, do not calculate total mass prop, just returns pre-computed total robot centroid."
   (if update-mass-properties (send self :update-mass-properties))
   (send (car (send self :links)) :get :c-til))
  (:inertia-tensor ;; total inertia tensor
   (&optional (update-mass-properties t))
   "Calculate total robot inertia tensor [g $mm^2$] around total robot centroid in euslisp world coordinates.
    If update-mass-properties argument is t, propagate total mass prop calculation for all links and returns total robot inertia tensor.
    Otherwise, do not calculate total mass prop, just returns pre-computed total robot inertia tensor."
   (if update-mass-properties (send self :update-mass-properties))
   (send (car (send self :links)) :get :I-til))

  ;; preview filter
  (:preview-control-dynamics-filter
   (dt ;; [s]
    avs ;; input motion : (list (list :angle-vector av :root-coords rc) ....)
    &key (preview-controller-class preview-controller)
         (cog-method :move-base-pos)
         (delay 0.8)) ;; [s]
   ;; generate av-list
   (labels ((update-robot-state
             (av)
             (send self :angle-vector (cadr (memq :angle-vector av)))
             (send self :move-coords (cadr (memq :root-coords av)) (car (send self :links)))))
     (update-robot-state (car avs))
     (let ((df (instance preview-control-cart-table-cog-trajectory-generator
                         :init dt (elt (send self :centroid) 2) :delay delay :initialize-queue-p t
                         :preview-controller-class preview-controller-class))
           (r))
       (dotimes (i 2) (send self :calc-zmp))
       (let ((ret-list) (last-av) (ret-avs) (last-zmp) (av)
             (avs (copy-object avs)))
         (while (not (send df :finishedp))
           (setq av (pop avs))
           (if av
               (progn
                 (update-robot-state av)
                 (setq last-av av))
             (update-robot-state last-av))
           (let* ((zmp (send self :calc-zmp
                             (send self :angle-vector)
                             (send self :copy-worldcoords)
                             :dt dt))
                  (rzmp (let ((tmp-zmp (cadr (memq :zmp av))))
                          (if tmp-zmp (setq last-zmp tmp-zmp) last-zmp)))
                  (dzmp (v- zmp rzmp)))
             (setq r (send df :update-xk (if av dzmp nil) (list av zmp (copy-object (send self :centroid)) rzmp)))
             (when r
               (push (list (send df :current-additional-data) r) ret-list)
               (let ((pav (send self :angle-vector))
                     (prc (send self :copy-worldcoords)))
                 (update-robot-state (car (send df :current-additional-data)))
                 (case cog-method
                   (:move-centroid-on-foot
                    (send self :move-centroid-on-foot :both '(:lleg :rleg)
                          :centroid-thre 0.1
                          :target-centroid-pos (v- (send self :centroid) (send df :refcog))))
                   (:move-base-pos
                    (let ((tc (mapcar #'(lambda (l) (send self l :end-coords :copy-worldcoords)) '(:rleg :lleg))))
                      (send self :translate (float-vector (- (elt (send df :refcog) 0)) (- (elt (send df :refcog) 1)) 0) :world)
                      (mapcar #'(lambda (l ttc)
                                  (send self l :inverse-kinematics ttc :thre 0.1 :rthre (deg2rad 0.1)))
                              '(:rleg :lleg) tc)))
                   )
                 (push (list :angle-vector (send self :angle-vector)
                             :root-coords (send (car (send self :links)) :copy-worldcoords))
                       ret-avs)
                 (send self :angle-vector pav)
                 (send self :newcoords prc)
                 )
               )
             ))
         (setq ret-avs (reverse ret-avs) ret-list (reverse ret-list))
         (update-robot-state (car ret-avs))
         (dotimes (i 2) (send self :calc-zmp))
         (objects (list self))
         (let* ((tm (- dt))
                (data
                 (mapcar
                  #'(lambda (av ret)
                      (update-robot-state av)
                      (let ((zmp (send self :calc-zmp
                                       (send self :angle-vector)
                                       (send self :copy-worldcoords)
                                       :dt dt)))
                        (send *viewer* :draw-objects :flush nil)
                        (send (caddr (car ret)) :draw-on :flush nil :size 400)
                        (send (send self :centroid) :draw-on :flush nil :size 300 :color #f(1 0 0))
                        (send (cadr (car ret)) :draw-on :flush nil :size 400)
                        (send zmp :draw-on :flush t :size 300 :color #f(1 0 0))
                        (x::window-main-one)
                        (unix:usleep 10000)
                        (setq tm (+ tm dt))
                        (list :time tm :output-zmp zmp :input-zmp (cadr (car ret))
                              :output-cog (send self :centroid nil)
                              :input-cog (cadddr (car ret)))
                        ))
                  ret-avs ret-list)))
           (mapcar #'(lambda (x y) (append x y)) ret-avs data)
           )))))
  )

;;;
;;; preview control and preview filter class
;;; written by {nozawa,kanzaki,k-okada}@jsk.t.u-tokyo.ac.jp
;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; riccati equation class
;;;  solve steady solution of riccati equation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass riccati-equation
  :super propertied-object
  :slots (A b c P Q R K
            A-bKt R+btPb-1 ;; for buffer
            ))
(defmethod riccati-equation
  (:init (AA bb cc QQ RR) 
    (setq A AA b BB c CC Q QQ R RR)
    (setq P (make-matrix (array-dimension c 1) (array-dimension c 1))))
  (:solve ()
    (let (diffP
	  (At (transpose A))
	  (bt (transpose b))
	  (ct (transpose c))
	  (zero-matrix (make-matrix (array-dimension c 1) (array-dimension c 1))))
      (do ((loop 0 (1+ loop)))
	  ((>= loop 10000))
	(let* ((tmp-PA (m* P A)))
          (setq R+btPb-1 (inverse-matrix (m+ (scale-matrix R (unit-matrix (array-dimension b 1))) (m* bt (m* P b))))
                K (m* R+btPb-1 (m* bt tmp-PA)))
          (let ((prevP (m- (m+ (m* At tmp-pa)
                               (m* ct (scale-matrix Q c)))
                           (m* (m* (m* At P) b) K))))
            (setq diffP (m- P prevP))
            (when ;;(matrix-eps= diffP zero-matrix (* *epsilon* 0.01))
                (eps-v= (array-entity diffP) (array-entity zero-matrix) (* *epsilon* 0.01))
              (setq A-bKt (transpose (m- A (m* b K))))
              (return-from :solve (list P K)))
          (setq P prevP)
          )))
      (warn ";; ERROR in (reccati (:solve)) : Counld not convergence ~A~%" diffP)
      ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; preview-controller class
;;;  A, b, c -> system configuration matrices
;;;  uk -> input for controller at the time "k"
;;;  xk -> state variable
;;;  delay -> preview step calculated from predictive time (delay = N)
;;;  y1-n -> reference queue (time is k, k+1, ... k+N)
;;;  state equation of this system is
;;;   xk+1 = A xk + b uk
;;;   yk   = c xk
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass preview-controller
  :super riccati-equation
  :slots (xk uk delay f1-n y1-n queue-index initialize-queue-p additional-data-queue finishedp initialized-p output-dim input-dim))
(defmethod preview-controller
  (:init
    (dt
     &key (q) (r)
          ((:delay d))
          ((:A _A))
          ((:B _B))
          ((:C _C))
          (state-dim (array-dimension _A 0))
          ((:output-dim odim) (array-dimension _C 0))
          ((:input-dim idim) (array-dimension _B 1))
          (init-xk (instantiate float-vector (array-dimension _A 0)))
          (init-uk (make-matrix (array-dimension _B 1) 1))
          ((:initialize-queue-p iqp)))
    "Initialize preview-controller.
     Q is weighting of output error and R is weighting of input.
     dt is sampling time [s].
     delay is preview time [s].
     init-xk is initial state value.
     A, B, C are state eq matrices.
     If initialize-queue-p is t, fill all queue by the first input at the begenning, otherwise, do not fill queue at the first."
    (setq delay (round (/ d dt)) initialize-queue-p iqp output-dim odim input-dim idim)
    (send-super :init _A _B _C q r)
    (setq queue-index 0)
    (when (null xk)
      (setq xk (make-matrix state-dim 1))
      (dotimes (i state-dim)
        (setf (aref xk i 0) (elt init-xk i))))
    (setq uk init-uk)
    (send self :solve)
    (send self :calc-f)
    self)
  (:calc-f () ;; calculate preview gain for k+1, ... k+N.
    (setq f1-n (make-matrix input-dim (* output-dim (1+ delay)))) ;; (matrix-row f1-n 0) is dummy to being mutiplied by p1-n
    (let* ((gsi (unit-matrix (array-dimension c 1)))
	   (bt (transpose b))
	   (ct (transpose c)))
      (dotimes (i delay)
	(let* ((qt (scale-matrix Q ct))
	       (fa (m* R+btPb-1 (m* bt (m* gsi qt)))))
          (setq gsi (m* A-bKt gsi))
          (dotimes (ii input-dim)
            (dotimes (jj output-dim)
              (setf (aref f1-n ii (+ (* output-dim (1+ i)) jj)) (aref fa ii jj))))
          ))))
  (:calc-u () (setq uk (m- (m* f1-n y1-n) (m* K xk))))
  (:calc-xk () ;; update xk+1
    (setq xk (m+ (m* A xk) (m* b (send self :calc-u)))))
  ;;
  (:update-xk
   (p &optional (add-data))
   "Update xk by inputting reference output.
    Return value : nil (initializing) => return values (middle) => nil (finished)
    If p is nil, automatically the last value in queue is used as input and preview controller starts finishing."
    ;; initialize
    (if p
        (progn
          (unless initialized-p
            (incf queue-index)
            (setq initialized-p (>= queue-index (1+ delay))))
          (when (null y1-n)
            (setq y1-n (make-matrix (* output-dim (1+ delay)) 1))
            (setq additional-data-queue (make-list (1+ delay)))
            (if initialize-queue-p
                (dotimes (i (1+ delay))
                  (dotimes (j output-dim)
                    (setf (aref y1-n (+ (* output-dim i) j) 0) (elt p j)))
                  (setf (elt additional-data-queue i) add-data)))
            )
          (unless initialize-queue-p
            (when (< queue-index (1+ delay))
              (dotimes (j output-dim)
                (setf (aref y1-n (+ (* output-dim queue-index) j) 0) (elt p j)))
              (setf (elt additional-data-queue queue-index) add-data)
              (return-from :update-xk nil))))
      (progn
        (decf queue-index)
        (setq p (send self :last-reference-output-vector))
        (if (<= queue-index 0) (setq finishedp t))))
    ;; update y1-n
    (dotimes (i delay)
      (dotimes (j output-dim)
        (setf (aref y1-n (+ (* output-dim i) j) 0) (aref y1-n (+ (* output-dim (1+ i)) j) 0)))
      (setf (elt additional-data-queue i) (elt additional-data-queue (1+ i))))
    (dotimes (j output-dim)
      (setf (aref y1-n (+ (* output-dim delay) j) 0) (elt p j)))
    (setf (elt additional-data-queue delay) add-data)
    ;;
    (send self :calc-xk)
    ;; return
    (if (and initialized-p (not finishedp))
        (list (send self :current-reference-output-vector)
              (send self :current-state-vector)
              (send self :current-output-vector)))
    )
  (:finishedp
   ()
   "Finished or not."
   finishedp)
  (:last-reference-output-vector
   ()
   "Last value of reference output queue vector (y_k+N_ref).
    Last value is latest future value."
   (subseq y1-n (* output-dim delay) (+ output-dim (* output-dim delay)))) ;; tail of queue
  (:current-reference-output-vector
   ()
   "First value of reference output queue vector (y_k_ref).
    First value is oldest future value and it can be used as current reference value."
   (subseq y1-n 0 output-dim)) ;; head of queue
  (:current-state-vector
   ()
   "Current state value (xk)."
   (matrix-column xk 0)
   )
  (:current-output-vector
   ()
   "Current output value (yk)."
   (matrix-column (m* c xk) 0))
  (:current-additional-data
   ()
   "Current additional data value.
    First value of additional-data-queue."
   (car additional-data-queue)
   )
  (:pass-preview-controller
   (reference-output-vector-list)
   "Get preview controller results from reference-output-vector-list and returns list."
   (let ((r) (ret))
     ;; Wait until initialize
     (while (not (setq r (send self :update-xk (pop reference-output-vector-list) nil))))
     (push r ret)
     ;; Get value until finish
     (while (setq r (send self :update-xk (pop reference-output-vector-list) nil))
       (push r ret))
     (reverse ret)
     ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; extended-preview-controller class
;;;  this class is error system of preview-control class
;;;  (refer to "Digital Preview Control (in Japanese)", SangyoToshoKabusikiGaisha, Takeshi Tsuchiya and Tadashi Egami)
;;;  orgA, orgb, orgc -> system configuration matrices
;;;  xk* -> [y_k^T, dxk^T]^T
;;;  state equation of this system is
;;;   xk+1* = A xk* + b duk
;;;   yk   = c xk*
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass extended-preview-controller
  :super preview-controller
  :slots (orgA orgB orgC xk*))

(defmethod extended-preview-controller
  (:init
    (dt
     &key (q) (r) ((:delay d))
          ((:A _orgA)) ((:B _orgB)) ((:C _orgC))
          (init-xk (instantiate float-vector (array-dimension _orgA 0)))
          (init-uk (make-matrix (array-dimension _orgB 1) 1))
          (state-dim (array-dimension _orgA 0))
          ((:initialize-queue-p iqp))
          (q-mat))
    "Initialize preview-controller in extended system (error system).
     Q is weighting of output error and R is weighting of input.
     dt is sampling time [s].
     delay is preview time [s].
     init-xk is initial state value.
     A, B, C are state eq matrices for original system and slot variables A,B,C are used for error system matrices.
     If initialize-queue-p is t, fill all queue by the first input at the begenning, otherwise, do not fill queue at the first."
    (setq orgA _orgA orgB _orgB orgC _orgC)
    (setq output-dim (array-dimension orgC 0))
    (setq input-dim (array-dimension orgB 1))
    (let* ((org-state-dim (array-dimension orgA 0))
           (new-state-dim (+ output-dim org-state-dim))
           (tmpcA (m* orgc orgA))
           (newA (make-matrix new-state-dim new-state-dim))
           (tmpcb (m* orgc orgB))
           (newb (make-matrix new-state-dim (array-dimension orgB 1)))
           (newC (make-matrix output-dim new-state-dim)))
      (dotimes (i output-dim)
        (setf (aref newA i i) 1.0))
      (dotimes (i output-dim)
        (dotimes (j org-state-dim)
          (setf (aref newA i (+ output-dim j)) (aref tmpcA i j))))
      (dotimes (i org-state-dim)
        (dotimes (j org-state-dim)
          (setf (aref newA (+ output-dim i) (+ output-dim j)) (aref orgA i j))))
      (dotimes (i output-dim)
        (dotimes (j input-dim)
          (setf (aref newb i j) (aref tmpcb i j))))
      (dotimes (i org-state-dim)
        (dotimes (j input-dim)
          (setf (aref newb (+ output-dim i) j) (aref orgb i j))))
      (if q-mat
          (dotimes (i output-dim)
            (setf (aref newC i i) (aref q-mat i i)))
        (dotimes (i output-dim)
          (setf (aref newC i i) 1.0)))
      (send-super :init
                  dt
                  :A newA :b newb :c newC :state-dim state-dim
                  :q q :r r :delay d :init-xk init-xk :init-uk init-uk :initialize-queue-p iqp :output-dim output-dim :input-dim input-dim)
      (when (null xk*)
        (setq xk* (make-matrix new-state-dim 1))
        (let ((init-y (m* orgC xk)))
          (dotimes (i output-dim)
            (setf (aref xk* i 0) (aref init-y i 0))))
        )
      self))
  ;;
  (:calc-f ()
    (setq f1-n (make-matrix input-dim (* output-dim (1+ delay))))
    (let* ((gsi (unit-matrix (array-dimension c 1)))
	   (bt (transpose b))
	   (ct (transpose c))
	   (R+btPb-1bt (m* R+btPb-1 bt))
	   (qt (scale-matrix Q ct)))
      (dotimes (i delay)
	(let* ((qt (if (= i (1- delay)) (m* P qt) qt))
               (fa (m* R+btPb-1 (m* bt (m* gsi qt)))))
          (setq gsi (m* A-bKt gsi))
          (dotimes (ii input-dim)
            (dotimes (jj output-dim)
              (setf (aref f1-n ii (+ (* output-dim (1+ i)) jj)) (aref fa ii jj))))
          ))))
  (:calc-u
   ()
   (let ((uk* (m- (m* f1-n y1-n) (m* K xk*))))
     (setq uk (m+ uk* uK))
     uk*))
  (:calc-xk ()
    (setq xk* (m+ (m* A xk*) (m* b (send self :calc-u))))
    (dotimes (i (array-dimension orgA 0))
      (setf (aref xk i 0)
            (+ (aref xk i 0)
               (aref xk* (+ output-dim i) 0))))
    )
  (:current-output-vector
   ()
   "Current additional data value.
    First value of additional-data-queue."
   (subseq (matrix-column xk* 0) 0 output-dim)
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; COG (xy) trajectory generator using preview-control convert reference ZMP from reference COG
;;;  (refer to "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point", ICRA 2003, p.1620-1626, Shuuji KAJITA, Fumio KANEHIRO, Kenji KANEKO, Kiyoshi FUJIWARA, Kensuke HARADA, Kazuhito YOKOI and Hirohisa HIRUKAWA
;;;            "Humanoid Robot (in Japanese)", Ohmsha, 2005, Shuji Kajita, ISBN 4-274-20058-2)
;;; A,B,C -> cart-table system
;;; xk -> 3 x 2 matrix (3 = COG pos, vel, acc, 2 = x, y)
;;; uk -> 1 x 2 matrix (COG jark of x and y)
;;  yk -> zmp
;;; unit system -> [mm], [s]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass preview-control-cart-table-cog-trajectory-generator
  :super propertied-object
  :slots (pcs cog-z zmp-z))
(defmethod preview-control-cart-table-cog-trajectory-generator
  (:init
    (dt _zc
     &key (q 1) (r 1e-6)
          ((:delay d) 1.6)
          (init-xk (float-vector 0 0 0))
          ((:A _A) (make-matrix 3 3 (list (list 1 dt (* 0.5 dt dt))
                                          (list 0 1 dt)
                                          (list 0 0 1))))
          ((:B _B) (make-matrix 3 1 (list (list (* (/ 1.0 6.0) dt dt dt))
                                          (list (* 0.5 dt dt))
                                          (list dt))))
          ((:C _C) (make-matrix 1 3 (list (list 1.0 0.0 (- (/ _zc (elt *g-vec* 2)))))))
          ((:initialize-queue-p iqp))
          (preview-controller-class extended-preview-controller))
    "COG (xy) trajectory generator using preview-control convert reference ZMP from reference COG.
     dt -> sampling time[s], _zc is height of COG [mm].
     preview-controller-class is preview controller class (extended-preview-controller by default).
     For other arguments, please see preview-controller and extended-preview-controller :init documentation."
    (setq pcs
          (list (instance preview-controller-class
                          :init dt :q q :r r :delay d
                          :init-xk (float-vector (elt init-xk 0) 0 0)
                          :A _A :B _B :C _C :state-dim 3 :initialize-queue-p iqp)
                (instance preview-controller-class
                          :init dt :q q :r r :delay d
                          :init-xk (float-vector (elt init-xk 1) 0 0)
                          :A _A :B _B :C _C :state-dim 3 :initialize-queue-p iqp)))
    (setq cog-z _zc)
    self)
  ;; getter methods
  (:refcog
   ()
   "Reference COG [mm]."
   (float-vector (elt (send (car pcs) :current-state-vector) 0) (elt (send (cadr pcs) :current-state-vector) 0) cog-z))
  (:cart-zmp ()
   "Cart-table system ZMP[mm] as an output variable."
   (float-vector (elt (send (car pcs) :current-output-vector) 0)
                 (elt (send (cadr pcs) :current-output-vector) 0)
                 zmp-z))
  (:last-refzmp ()
   "Reference zmp at the last of queue."
   (float-vector (elt (send (car pcs) :last-reference-output-vector) 0)
                 (elt (send (cadr pcs) :last-reference-output-vector) 0)
                 zmp-z))
  (:current-refzmp ()
   "Current reference zmp at the first of queue."
   (float-vector (elt (send (car pcs) :current-reference-output-vector) 0)
                 (elt (send (cadr pcs) :current-reference-output-vector) 0)
                 zmp-z))
  ;;
  (:update-xk
   (p &optional (add-data))
   "Update xk and returns zmp and cog values.
    For arguments, please see preview-controller and extended-preview-controller :update-xk."
    (if p (setq zmp-z (elt p 2)))
    (let ((ret (list (send (car pcs) :update-xk (subseq p 0 1) add-data)
                     (send (cadr pcs) :update-xk (subseq p 1 2) add-data))))
      (setq ret (and (car ret) (cadr ret)))
      (if ret
          (list
           (send self :current-refzmp)
           (send self :refcog)
           (send self :cart-zmp)))))
  (:finishedp () "Finished or not." (and (send (car pcs) :finishedp) (send (cadr pcs) :finishedp)))
  (:current-additional-data () "Current additional data value." (send (car pcs) :current-additional-data))
  (:pass-preview-controller
   (reference-output-vector-list)
   "Get preview controller results from reference-output-vector-list and returns list."
   (let ((r) (ret))
     ;; Wait until initialize
     (while (not (setq r (send self :update-xk (pop reference-output-vector-list) nil))))
     (push r ret)
     ;; Get value until finish
     (while (setq r (send self :update-xk (pop reference-output-vector-list) nil))
       (push r ret))
     (reverse ret)
     ))
  (:cog-z () "COG Z [mm]." cog-z)
  (:update-cog-z (zc)
   (mapcar #'(lambda (pc)
               (setf (aref (pc . c) 0 2) (- (/ zc (elt *g-vec* 2))))
               (send pc :solve)
               (send pc :calc-f))
           pcs))
  )

;;
;; gait generator
;;   currently biped robots are supported
;; definition
;;   step  -> one step
;;   index -> one control loop in one step
(defclass gait-generator
  :super propertied-object
  :slots (robot dt
          ;; list for parameters (here, call "gait paramters")
          footstep-node-list ;; list of footstep <- step-node class instance
          support-leg-list ;; support leg list
          support-leg-coords-list ;; support leg coords list
          swing-leg-dst-coords-list ;; destination of swing leg
          swing-leg-src-coords ;; source of swing leg = previous suppor leg coords
          refzmp-cur-list ;; current reference zmp
          refzmp-next ;; next reference zmp
          refzmp-prev ;; previous reference zmp
          step-height-list ;; step height list
          ;; other parameters
          one-step-len ;; division number for one step
          index-count ;; counter for one step
          default-step-height ;; default step height
          default-double-support-ratio ;; default double support ratio ;; (- 1 default-double-support-ratio) is single support ratio
          default-zmp-offsets ;; zmp offset list in the legs' end-coords ;; (list rleg-zmp-offset lleg-zmp-offset)
          finalize-p ;; Are all footstep-node-list executed? finalize flag for make-gait-parameter. Is this necessary?
          ;; preview controller parameters
          apreview-controller ;; preview controller for ZMP -> COG
          all-limbs
          end-with-double-support
          ik-thre ;; position thre for ik
          ik-rthre ;; rotaiotn thre fo rik
          ;; Smooth interpolation for swing-leg-proj-ratio
          swing-leg-proj-ratio-interpolation-acc
          swing-leg-proj-ratio-interpolation-vel
          swing-leg-proj-ratio-interpolation-pos
          ;; Smooth interpolation for swing-rot-ratio
          swing-rot-ratio-interpolation-acc
          swing-rot-ratio-interpolation-vel
          swing-rot-ratio-interpolation-pos
          )
  )

;; methods
;; footstep generation process
;;  footstep (input) -> support-leg-coords, swing-leg-coords -> refzmp -> cog -> angle-vector (output)
;;  step-node class -> geometrical footstep parameters per one step
;;  step-height, step-time -> real robot execution per one step sequence
;;  robot, dt -> robot independent
(defmethod gait-generator
  (:init
    (rb _dt)
    (setq robot rb dt _dt)
    ;; gen append methods
    (dolist (list-name '(footstep-node-list support-leg-list
                         support-leg-coords-list swing-leg-dst-coords-list
                         swing-leg-src-coords refzmp-cur-list
                         step-height-list))
      (eval `(defmethod gait-generator
               (,(read-from-string (format nil ":append-~A" list-name))
                (_args)
                (setq ,list-name (append ,list-name (list _args)))))))
    self)
  (:get-footstep-limbs
   (fs)
   (mapcar #'(lambda (tmpfs)
               (if (find-method tmpfs :l/r)
                   (send tmpfs :l/r)
                 (send tmpfs :name)))
           fs))
  (:get-counter-footstep-limbs
   (fs)
   (if (symbolp (car fs))
       (remove-if
        #'(lambda (l)
            (memq l (remove-if-not #'(lambda (x) (memq x fs)) all-limbs)))
        all-limbs)
     (send self :get-counter-footstep-limbs (send self :get-footstep-limbs fs)))
   )
  (:get-limbs-zmp-list
   (limb-coords limb-names)
   (mapcar #'(lambda (lc ln)
               (v+ (send lc :worldpos)
                   (send lc :rotate-vector
                         (cadr (memq ln default-zmp-offsets)))))
           limb-coords limb-names))
  (:get-limbs-zmp
   (limb-coords limb-names)
   (scale (/ 1.0 (length limb-names))
          (reduce
           #'v+
           (send self :get-limbs-zmp-list limb-coords limb-names)
           :initial-value (float-vector 0 0 0)
           ))
   )
  (:get-swing-limbs
   (limbs)
   (remove-if #'(lambda (x) (memq x limbs)) all-limbs)
   )
  ;; initialize parameter and generate first gait parameter
  (:initialize-gait-parameter
    (fsl time cog ;; time is [s], cog is [mm]
     &key ((:default-step-height dsh) 50) ;; height [mm]
          ((:default-double-support-ratio ddsr) 0.2)
          (delay 1.6) ;; delay [s]
          ((:all-limbs al) '(:rleg :lleg))
          ((:default-zmp-offsets dzo)
           (mapcan #'(lambda (x) (list x (float-vector 0 0 0))) al)) ;; [mm]
          (q 1.0) (r 1e-6)
          (thre 1) (rthre (deg2rad 1)) ;; see :move-centroid-on-foot (thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) 1 5)) fix-limbs))
          (start-with-double-support t)
          ((:end-with-double-support ewds) t)
          )
    (setq footstep-node-list (mapcar #'(lambda (l) (if (consp l) l (list l))) fsl)
          one-step-len (round (/ time dt))
          support-leg-coords-list nil
          support-leg-list nil
          swing-leg-dst-coords-list nil
          refzmp-cur-list nil
          step-height-list nil
          default-step-height dsh
          default-double-support-ratio ddsr
          default-zmp-offsets dzo
          index-count one-step-len
          finalize-p nil
          all-limbs al
          ik-thre thre ik-rthre rthre
          end-with-double-support ewds)
    (setq swing-leg-proj-ratio-interpolation-pos 0
          swing-leg-proj-ratio-interpolation-vel 0
          swing-leg-proj-ratio-interpolation-acc 0
          swing-rot-ratio-interpolation-pos 0
          swing-rot-ratio-interpolation-vel 0
          swing-rot-ratio-interpolation-acc 0)
    (let ((current-swing-limbs
           (send self :get-footstep-limbs (car footstep-node-list))))
      (send self :append-support-leg-list
            (send self :get-counter-footstep-limbs (car footstep-node-list)))
      (let ((current-support-limbs (car support-leg-list)))
        (send self :append-support-leg-coords-list
              (mapcar #'(lambda (l) (send robot l :end-coords :copy-worldcoords)) current-support-limbs))
        (send self :append-swing-leg-dst-coords-list
              (mapcar #'(lambda (l) (send robot l :end-coords :copy-worldcoords)) current-swing-limbs))
        (let ((current-support-leg-coords (car support-leg-coords-list))
              (current-swing-leg-dst-coords (car swing-leg-dst-coords-list)))
          (send self :append-refzmp-cur-list
                (send self :get-limbs-zmp
                      (if start-with-double-support
                          (append current-swing-leg-dst-coords current-support-leg-coords)
                        current-swing-leg-dst-coords)
                      (if start-with-double-support
                          (append current-swing-limbs current-support-limbs)
                        current-swing-limbs)
                      ))
    (send self :append-step-height-list 0.0)
    (pop footstep-node-list)
    ;; currently, not error system
    (setq apreview-controller (instance preview-control-cart-table-cog-trajectory-generator :init
                                       dt (- (elt cog 2) (elt (car refzmp-cur-list) 2))
                                       :delay delay
                                       :q q :r r
                                       :init-xk cog
                                       :preview-controller-class extended-preview-controller
                                       )
          swing-leg-src-coords current-swing-leg-dst-coords
          refzmp-next (send self :get-limbs-zmp
                            (mapcar #'(lambda (l) (send robot l :end-coords :copy-worldcoords)) (send self :get-counter-footstep-limbs (send-all (car footstep-node-list) :name)))
                            (send self :get-counter-footstep-limbs (send-all (car footstep-node-list) :name)))
          refzmp-prev (car refzmp-cur-list))
    (send self :make-gait-parameter)
    t))))
  ;; append final gait parameters
  (:finalize-gait-parameter
    ()
    (let ((dst-swing-coords (car (last support-leg-coords-list)))
          (prev-support-limbs (car (last support-leg-list))))
      (send self :append-support-leg-list
            (send self :get-counter-footstep-limbs
                  (send self :get-swing-limbs (cadr (reverse support-leg-list)))))
      (send self :append-support-leg-coords-list
            (mapcar #'(lambda (x)
                        (if (memq x prev-support-limbs)
                            (elt (car (last support-leg-coords-list)) (position x prev-support-limbs))
                          (elt (car (last swing-leg-dst-coords-list))
                               (position x (send self :get-swing-limbs prev-support-limbs)))))
                    (car (last support-leg-list))))
      (send self :append-swing-leg-dst-coords-list
            (mapcar #'(lambda (x)
                        (elt dst-swing-coords (position x prev-support-limbs)))
                    (send self :get-swing-limbs (car (last support-leg-list)))))
      (send self :append-refzmp-cur-list
            (send self :get-limbs-zmp
                  (if end-with-double-support
                      (append (car (last support-leg-coords-list)) (car (last swing-leg-dst-coords-list)))
                    (car (last swing-leg-dst-coords-list)))
                  (if end-with-double-support
                      (append (car (last support-leg-list)) (send self :get-counter-footstep-limbs (car (last support-leg-list))))
                    (send self :get-counter-footstep-limbs (car (last support-leg-list))))
                  ))
      (send self :append-step-height-list 0.0)
      (setq refzmp-next (car (last refzmp-cur-list)))
      t))
  ;; generate gait parameters
  (:make-gait-parameter
    ()
    (let ((fs (pop footstep-node-list))
          (prev-support-limbs (car (last support-leg-list))))
      (send self :append-support-leg-list
            (send self :get-counter-footstep-limbs fs))
      (send self :append-support-leg-coords-list
            (mapcar #'(lambda (x)
                        (if (memq x prev-support-limbs)
                            (elt (car (last support-leg-coords-list)) (position x prev-support-limbs))
                          (elt (car (last swing-leg-dst-coords-list))
                               (position x (send self :get-swing-limbs prev-support-limbs)))))
                    (car (last support-leg-list))))
      (send self :append-swing-leg-dst-coords-list (send-all fs :worldcoords))
      (send self :append-refzmp-cur-list
            (send self :get-limbs-zmp (car (last support-leg-coords-list)) (car (last support-leg-list))))
      (send self :append-step-height-list default-step-height)
      t))
  (:calc-hoffarbib-pos-vel-acc
   (tmp-remain-time tmp-goal old-acc old-vel old-pos)
   (let* ((jerk (+ (+ (* (/ -9.0 tmp-remain-time) old-acc)
                      (* (/ -36.0 (expt tmp-remain-time 2)) old-vel))
                   (* (/ 60.0 (expt tmp-remain-time 3)) (- tmp-goal old-pos))))
          (new-acc (+ old-acc (* dt jerk)))
          (new-vel (+ old-vel (* dt new-acc)))
          (new-pos (+ old-pos (* dt new-vel))))
     (list new-acc new-vel new-pos)
     ))
  ;; calculate swing leg coords
  (:calc-current-swing-leg-coords
    (ratio src dst &key (type :shuffling) (step-height default-step-height))
    ;; Calculate swing leg coords
    (case type
      (:shuffling
        (midcoords ratio src dst))
      (:cycloid
        (send self :cycloid-midcoords ratio src dst step-height :rot-ratio swing-rot-ratio-interpolation-pos))
      )
    )
  (:calc-ratio-from-double-support-ratio
   ()
   (let* (;; narrowed one-step-len according to double-support-ratio
          (narrow-one-step-len (* (- 1.0 default-double-support-ratio) one-step-len))
          ;; ratio = current count / narrow-one-len
          (tmp-ratio (- 1.0 (/ (- (float index-count) (* 0.5 default-double-support-ratio one-step-len))
                               narrow-one-step-len))))
     ;; ratio <- [0.0 1.0]
     (cond
      ((< tmp-ratio 0.0) 0.0)
      ((> tmp-ratio 1.0) 1.0)
      (t tmp-ratio))))
  (:calc-current-refzmp
   (prev cur next)
   (let* ((cnt (- one-step-len index-count))
          (margin-count (* 0.5 default-double-support-ratio one-step-len)))
     (cond
      ((< cnt margin-count)
       (midpoint (* (/ -0.5 margin-count) (- cnt margin-count))
                 cur prev))
      ((> cnt (- one-step-len margin-count))
       (midpoint (* (/ 0.5 margin-count) (- cnt (- one-step-len margin-count)))
                 cur next))
      (t cur))
     ))
  (:calc-one-tick-gait-parameter
   (type)
   ;; Calculate swing leg rot ratio
   (let ((default-half-double-support-count (truncate (* 0.5 default-double-support-ratio one-step-len))))
     (cond
      ((> index-count (- one-step-len default-half-double-support-count)) ;; initial double support period
       (setq swing-rot-ratio-interpolation-pos 0
             swing-rot-ratio-interpolation-vel 0
             swing-rot-ratio-interpolation-acc 0))
      ((< index-count default-half-double-support-count) ;; final double support period
       (setq swing-rot-ratio-interpolation-pos 1.0
             swing-rot-ratio-interpolation-vel 0
             swing-rot-ratio-interpolation-acc 0))
      (t ;; swing period
       (multiple-value-setq
        (swing-rot-ratio-interpolation-acc swing-rot-ratio-interpolation-vel swing-rot-ratio-interpolation-pos)
        (send self :calc-hoffarbib-pos-vel-acc
              (* (1+ (- index-count default-half-double-support-count)) dt) 1.0 swing-rot-ratio-interpolation-acc swing-rot-ratio-interpolation-vel swing-rot-ratio-interpolation-pos))
       )))
   (list (car support-leg-list)
         (car support-leg-coords-list)
         (mapcar #'(lambda (sw-sc sw-dc)
                     (send self :calc-current-swing-leg-coords
                           (send self :calc-ratio-from-double-support-ratio)
                           sw-sc sw-dc
                           :type type :step-height (car step-height-list)))
                 swing-leg-src-coords (car swing-leg-dst-coords-list))
         (send self :calc-current-refzmp
               refzmp-prev (car refzmp-cur-list) refzmp-next)
         (let* ((swing-leg-proj-ratio
                 (progn
                   (if (= index-count one-step-len)
                       (setq swing-leg-proj-ratio-interpolation-pos 0
                             swing-leg-proj-ratio-interpolation-vel 0
                             swing-leg-proj-ratio-interpolation-acc 0))
                   (multiple-value-setq
                    (swing-leg-proj-ratio-interpolation-acc swing-leg-proj-ratio-interpolation-vel swing-leg-proj-ratio-interpolation-pos)
                    (send self :calc-hoffarbib-pos-vel-acc
                          (* index-count dt) 1.0 swing-leg-proj-ratio-interpolation-acc swing-leg-proj-ratio-interpolation-vel swing-leg-proj-ratio-interpolation-pos))
                   swing-leg-proj-ratio-interpolation-pos))
                (swing-leg-proj-coords
                 (mapcar #'(lambda (sw-sc sw-dc)
                             (midcoords swing-leg-proj-ratio sw-sc sw-dc))
                        swing-leg-src-coords (car swing-leg-dst-coords-list)))
               (limbs
                (append (send self :get-counter-footstep-limbs (car support-leg-list))
                        (car support-leg-list))))
           (+ (send apreview-controller :cog-z)
              (elt (send self :get-limbs-zmp
                         (append swing-leg-proj-coords (car support-leg-coords-list))
                         limbs) 2))
           )
         (let* ((cnt (- one-step-len index-count))
                (margin-count (* 0.5 default-double-support-ratio one-step-len)))
           (if (or (< cnt margin-count)
                   (> cnt (- one-step-len margin-count))
                   (eps= (float (car step-height-list)) 0.0))
               (mapcar #'(lambda (x) :support) all-limbs)
             (mapcar #'(lambda (x) (if (memq x (car support-leg-list)) :support :swing)) all-limbs)
             )
           ))
   )
  ;; generate one tick parameter
  ;;  in detail, calculate swing leg coords, cog and angle-vector
  (:proc-one-tick
    (&key (type :shuffling) (solve-angle-vector :solve-av-by-move-centroid-on-foot)
          (solve-angle-vector-args) (debug nil))
    (let* ((preview-data ;; data to push to preview-controller's que
            (if support-leg-coords-list
                (send self :calc-one-tick-gait-parameter type)))
           (dd (send apreview-controller :update-xk ;; return value from preview-controller
                     (if preview-data (elt preview-data 3)) preview-data))
           (gp (send apreview-controller :current-additional-data))) ;; gait parameter
      (if dd (setf (elt (cadr dd) 2) (elt gp 4))) ;; overwrite 
      (let ((ret (if (and dd solve-angle-vector)
                     (append (send self :solve-angle-vector
                                   (elt gp 0) (elt gp 1) (elt gp 2) (cadr dd)
                                   :solve-angle-vector solve-angle-vector
                                   :solve-angle-vector-args solve-angle-vector-args)
                             (if debug (append (butlast gp 2) (list (cadr dd)
                                                                  (send apreview-controller :cart-zmp))
                                               (last gp)
                                               ))))))
        (when (>= 0 (decf index-count))
          (send self :update-current-gait-parameter))
        ret)))
  ;; update gait parameter
  (:update-current-gait-parameter
   ()
   (setq refzmp-prev (car refzmp-cur-list))
   (let ((prev-support-limbs2 (car support-leg-list)))
   (cond
    (footstep-node-list
     (send self :make-gait-parameter)
     )
    ((null finalize-p)
     (send self :finalize-gait-parameter)
     (setq finalize-p t)
     ))
   (setq index-count one-step-len)
   (pop support-leg-list)
   (setq swing-leg-src-coords
         (if support-leg-list
             (mapcar #'(lambda (x)
                         (elt (car support-leg-coords-list) (position x prev-support-limbs2)))
                     (send self :get-swing-limbs (car support-leg-list)))))
   (pop support-leg-coords-list)
   )
   (pop swing-leg-dst-coords-list)
   (pop refzmp-cur-list)
   (pop step-height-list)
   (if (cadr refzmp-cur-list)
       (setq refzmp-next (cadr refzmp-cur-list))
     (setq refzmp-next (car refzmp-cur-list)))
   )
  (:solve-angle-vector
    (support-leg support-leg-coords swing-leg-coords cog
     &key (solve-angle-vector :solve-av-by-move-centroid-on-foot)
          (solve-angle-vector-args))
    (let ((ik-ret
           (cond
            ((functionp solve-angle-vector)
             (apply solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            ((and (symbolp solve-angle-vector) (find-method self solve-angle-vector))
             (send* self solve-angle-vector
                    support-leg support-leg-coords
                    swing-leg-coords cog robot solve-angle-vector-args))
            (t (error ";; in :solve-angle-vector, invalid function or method~%!")))))
      (list ik-ret (send (car (send robot :links)) :copy-worldcoords))))

  ;; solve-angle-vector methods
  (:solve-av-by-move-centroid-on-foot
   (support-leg support-leg-coords swing-leg-coords cog robot
   &rest args &key (cog-gain 3.5) (stop 100) (additional-nspace-list) &allow-other-keys)
   (let* ((legs (append (send self :get-counter-footstep-limbs support-leg)
                        support-leg))
          (leg-order (mapcar #'(lambda (l) (position l legs)) all-limbs))
          (fix-coords (append swing-leg-coords support-leg-coords)))
     (unless (memq :thre args)
       (append args (list :thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) ik-thre (* 5 ik-thre))) all-limbs))))
     (unless (memq :rthre args)
       (append args (list :rthre (mapcar #'(lambda (x) (deg2rad (if (memq x '(:rleg :lleg)) ik-rthre (* 5 ik-rthre)))) all-limbs))))
     (send* robot :move-centroid-on-foot
            :both all-limbs :target-centroid-pos cog
            :fix-limbs-target-coords (mapcar #'(lambda (idx) (elt fix-coords idx)) leg-order)
            :cog-gain cog-gain :stop stop
            :additional-nspace-list
            (append
             additional-nspace-list
             (list (list (car (send robot :links))
                         #'(lambda ()
                             (let* ((fcoords (apply #'midcoords 0.5
                                                    (mapcar #'(lambda (x) (elt fix-coords (position x legs))) '(:rleg :lleg))))
                                    (xvr (send robot :rotate-vector (case (length legs) (2 #f(1 0 0)) (4 #f(0 0 1)))))
                                    (xvf (send fcoords :rotate-vector #f(1 0 0))))
                               (if (> (elt (send fcoords :inverse-rotate-vector xvr) 0) 0) (setq xvr (scale -1 xvr)))
                               (dolist (xv (list xvr xvf)) (setf (elt xv 2) 0.0))
                               (let ((dth (if (and (eps= (norm xvf) 0.0) (eps= (norm xvr) 0.0))
                                              0.0 (asin (elt (v* (normalize-vector xvf) (normalize-vector xvr)) 2)))))
                                 (float-vector 0 0 0 0 0 dth)
                                 ))))))
            args)))
  ;; calculate midpoint using cycloig orbit
  ;;   ratio  -> midpoint ratio
  ;;   start  -> start pos
  ;;   goal   -> goal pos
  ;;   height -> max height of step
  (:cycloid-midpoint
   (ratio start goal height &key (top-ratio 0.5))
   (let* ((mc (v+ (float-vector 0 0 height)
                  (midpoint top-ratio start goal)))
          (mv (if (> ratio 0.5) (v- goal mc) (v- mc start)))
          (u (v- goal start))
          (w (scale (abs (elt mv 2)) (float-vector 0 0 1.0))))
     (setf (elt mv 2) 0)
     (setf (elt u 2) 0)
     (let* ((u (scale (/ (* 2.0 (norm mv)) pi) (normalize-vector u)))
            (v (normalize-vector (v* w u)))
            (th (* 2pi ratio))
            (cycloid-point
             (float-vector (+ (if (> ratio 0.5) -pi/2 0.0) (* 0.5 (- th (sin th))))
                           0.0
                           (+ (if (> ratio 0.5) -1 0.0) (* 0.5 (- 1.0 (cos th))))))
            (dv (transform cycloid-point
                           (matrix u v w))))
       (v+ dv (if (> ratio 0.5) mc start))
       )))
  ;; calculate midcoords using cycloig orbit
  ;;   ratio  -> midpoint ratio
  ;;   start  -> start pos
  ;;   goal   -> goal pos
  ;;   height -> max height of step
  (:cycloid-midcoords
   (ratio start goal height &key (top-ratio 0.5) (rot-ratio ratio))
   (let ((cp (send self :cycloid-midpoint ratio (send start :worldpos) (send goal :worldpos)
                   height :top-ratio top-ratio)))
     (make-coords :pos cp
                  :rot (send (midcoords rot-ratio start goal) :worldrot))))
  )

(in-package "GEOMETRY")
(provide :irtdyna)
