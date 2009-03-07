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
  :slots (parent-link child-link joint-angle min-angle max-angle))

(defmethod joint
  (:init (&key (name :joint) ((:child-link clink)) ((:parent-link plink))
	       (min -90) (max 90) &allow-other-keys)
	 (send self :name name)
	 (setq parent-link plink child-link clink
	       min-angle min max-angle max)
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
   (let (relang)
     (when v
       (if relative (setq v (+ v joint-angle)))
       (when (>= v max-angle)
	 (setq v max-angle))
       (when (<= v min-angle)
	 (setq v min-angle))
       (setq relang (- v joint-angle) joint-angle v)
       (send child-link :rotate (deg2rad relang) axis))
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
   (let (relang)
     (when v
       (if relative (setq v (+ v joint-angle)))
       (when (>= v max-angle)
	 (setq v max-angle))
       (when (<= v min-angle)
	 (setq v min-angle))
       (setq relang (- v joint-angle) joint-angle v)
       (send child-link :translate (scale relang axis)))
     joint-angle))
  )

(defclass bodyset-link
  :super bodyset
  :slots (joint parent-link child-links analysis-level))
(defmethod bodyset-link
  (:init 
   (coords
    &rest args
    &key
    ((:analysis-level level) :body)
    &allow-other-keys)
   (setq analysis-level level)
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
  (:joint (&rest args) (user::forward-message-to joint args))
  (:add-joint (j) (setq joint j))

  (:parent-link () parent-link)
  (:child-links () child-links)

  (:add-child-links (l) (unless (or (member l child-links) (not l))(push l child-links)))
  (:add-parent-link (l) (setq parent-link l))

  (:del-child-link  (l) (setq child-links (delete l child-links)))
  (:del-parent-link ()  (setq parent-link nil))
  )

(defclass cascaded-link
  :super cascaded-coords
  :slots (links joint-list bodies
		collision-avoidance-link-pair))

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
     (case translation-axis
	   ((:x :y :z) (decf dim 1))
	   ((:xy :yx :yz :zy :zx :zy) (decf dim 2))
	   (nil (decf dim 3)))
     (case rotation-axis ((:x :y :z) (decf dim 1)) (nil (decf dim 3)))
     dim))
  (:calc-inverse-jacobian
   (jacobi &rest args
    &key
    ((:manipulability-limit ml) 0.1)
    ((:manipulability-gain mg) 0.001)
    weight debug-view
    ret wmat tmat umat umat2 mat-tmp
    tmp-mcc tmp-mrc tmp-mrr tmp-mrr2
    &allow-other-keys)
   (let (jacobi# m m2 (k 0))
     ;; m : manipulability
     (setq m (manipulability jacobi tmp-mrr tmat))
     (if (< m ml) (setq k (* mg (expt (- 1.0 (/ m  ml)) 2))))
     (when debug-view
       (warn "k     :~7,3f (manipulability:~7,3f, gain:~7,3f, limit:~7,3f, len:~d)~%" k m mg ml (cadr (array-dimensions jacobi))))

     ;; calc weighted SR-inverse
     (setq jacobi# (sr-inverse jacobi k weight 
			       ret wmat tmat umat umat2 mat-tmp
			       tmp-mcc tmp-mrc tmp-mrr tmp-mrr2
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
	      &key move-target (rotation-axis nil) (translation-axis t)
	      (dim (send self :calc-target-axis-dimension rotation-axis translation-axis))
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
     (unless fik (setq fik (make-matrix dim len)))
     (dotimes (i len)
       (setq j (send (elt link-list i) :joint))
       (cond ((and (< (+ i 1) len)
		   (not (eq (send j :child-link)
			    (send (elt link-list (+ i 1)) :parent-link))))
	      (setq child-reverse t)
	      (setq child-link (send j :parent-link)))
	     (t
	      (setq child-reverse nil)
	      (setq child-link (send j :child-link))))
	(setq paxis (case (j . axis)
		      (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
		      (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
		      (t (j . axis))))

	(setq tmp-v3 (normalize-vector (send child-link :rotate-vector paxis tmp-v3) tmp-v3))
	(setq tmp-v3
	 (v- (send move-target :inverse-transform-vector tmp-v3 tmp-v3 tmp-v3b tmp-m33)
	     (send move-target :inverse-transform-vector #f(0 0 0) tmp-v3a tmp-v3b tmp-m33)  tmp-v3))

	(cond 
	 ((derivedp j linear-joint)
	  ;(setq pos-r (scale 0.001 pos-r pos-r));; scale [mm] -> [m]
	  (setq pos-r (calc-dif-with-translation-axis tmp-v3 translation-axis tmp-v0 tmp-v1 tmp-v2))
	  (dotimes (j (length pos-r)) (setf (aref fik j i) (elt pos-r j)))
	  (setq pos-i (calc-dif-with-rotation-axis #f(0 0 0) rotation-axis tmp-v0 tmp-v2))
	  (dotimes (j (length pos-i)) (setf (aref fik (+ j (length pos-r)) i) (elt pos-i j)))
	  )
	 (t
	  (setq pos-i (scale (if child-reverse 1 -1)
			     (send move-target :inverse-transform-vector
				   (send child-link :worldpos) tmp-v3a tmp-v3b tmp-m33) tmp-v3a))
	  (setq pos-i (scale 0.001 pos-i pos-i));; scale [mm] -> [m]

	  (setq pos-r (v* tmp-v3 pos-i tmp-v3b))
	  (setq pos-r (calc-dif-with-translation-axis pos-r translation-axis tmp-v0 tmp-v1 tmp-v2))
	  (dotimes (j (length pos-r)) (setf (aref fik j i) (elt pos-r j)))
	  (setq pos-i (calc-dif-with-rotation-axis tmp-v3 rotation-axis tmp-v0 tmp-v2))
	  (dotimes (j (length pos-i)) (setf (aref fik (+ j (length pos-r)) i) (elt pos-i j)))
	  ))
	;;
	)
     fik))
  ;;
  ;; calc joint angle speed from velocity vector (pos, rot)
  ;; q = J#x + (I - J#J)y
  ;;
  (:calc-joint-angle-speed
   (vel-pos vel-rot
    &rest args
    &key
    link-list
    move-target jacobi jacobi# null-space
    (rotation-axis nil) (translation-axis t)
    p-limit r-limit
    (weight (fill (instantiate float-vector (length link-list)) 1))
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
   (let (vel-p vel-r J#x I-J#J (len (length link-list)))
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

     (setq vel-p (calc-dif-with-translation-axis
		  (v- (send move-target :inverse-transform-vector vel-pos
			    tmp-v3 tmp-v3b tmp-m33)
		      (send move-target :inverse-transform-vector #f(0 0 0)
			    tmp-v3a tmp-v3b tmp-m33) tmp-v3c)
		  translation-axis
		  tmp-v0 tmp-v1 tmp-v2))
     (dotimes (i (length vel-p)) (setf (elt tmp-dim i) (elt vel-p i)))

     (setq vel-r (calc-dif-with-rotation-axis
		  (v- (send move-target :inverse-transform-vector vel-rot 
			    tmp-v3 tmp-v3b tmp-m33)
		      (send move-target :inverse-transform-vector #f(0 0 0)
			    tmp-v3a tmp-v3b tmp-m33) tmp-v3)
		  rotation-axis
		  tmp-v0 tmp-v2))
     (dotimes (i (length vel-r)) (setf (elt tmp-dim (+ i (length vel-p))) (elt vel-r i)))
     
     (if (null tmp-len) (setq tmp-len (instantiate float-vector len)))
     (if (null tmp-len2) (setq tmp-len2 (instantiate float-vector len)))
     (setq J#x (transform jacobi# tmp-dim tmp-len))
     (when debug-view
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
       (v+ J#x (transform I-J#J null-space tmp-len2) J#x))
     J#x))
  (:collision-avoidance-link-pair () collision-avoidance-link-pair)
  (:collision-avoidance
   (&rest args
	  &key link-list (warnp t)
	  &allow-other-keys)
   (let* ((pair-list (send self :collision-avoidance-link-pair))
	  (pair-len (length pair-list))
	  (ret (make-list pair-len))
	  (col-list (send self :get :collision-distance))
	  (distance-limit 1.0)
	  (sc 0.0) np collidep pair)
     (dotimes (i pair-len) (setf (elt ret i) (float-vector 0 0 0)))
     (when (or (null link-list) (= pair-len 0))
       (print (send self :collision-avoidance-link-pair))
       (warn ";; ERROR: :link-list ~A / pair-len ~A required~%" link-list pair-len)
       (return-from :collision-avoidance ret))
     (if (or (null col-list) (/= (length col-list) pair-len))
	 (setq col-list (make-list pair-len)))
     ;; pair (part of this limb . part of another limb)
     (dotimes (i pair-len)
       (setq pair (elt pair-list i))
       (if (not (memq (car pair) link-list)) 
	   (progn
	     (setf (elt col-list i) nil)
	     )
	 (progn
	   (setq np (pqp-collision-distance (car pair) (cadr pair) :qsize 2))
	   (if (<= (car np) distance-limit)
	       (progn
		 (if warnp
		     (warn ";; !!WARNING!! collision detected~%;; (~a . ~a . ~a)~%"
			   (send (car pair) :name) (send (cadr pair) :name) np))
		 (if (elt col-list i)
		     (setq np (elt col-list i))
		   (setf (elt col-list i) np))
		 (setf (elt (elt col-list i) 0) distance-limit)
		 (setq collidep t))
	     (progn
	       (setf (elt col-list i) np)
	       (setq collidep nil)))
	   (setf (elt ret i)
		 (normalize-vector (v- (elt np 1) (elt np 2) (elt ret i)) (elt ret i)))
	   ))) ;; dotimes
     (send self :put :collision-distance col-list)
     ret))
  (:move-joints
   (vel-pos vel-rot &rest args
    &key
    link-list
    move-target
    (debug-view nil)
    &allow-other-keys)
   (let (dav dtheta)
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     (if (and debug-view (not (equal debug-view :no-clear)) (boundp '*viewer*))
	 (send *viewer* :viewsurface :clear))

     (setq dav (send* self :calc-joint-angle-speed vel-pos vel-rot args))

     ;; update body
     (dotimes (i (length link-list))
       (setq dtheta
	 (cond ((derivedp (send (elt link-list i) :joint) linear-joint)
		(* 1000.0 (elt dav i)))
	       ((derivedp (send (elt link-list i) :joint) rotational-joint)
		(rad2deg (elt dav i)))))
       (send (elt link-list i) :joint :joint-angle dtheta :relative t))
     ;;
     ;; done
     (when (and debug-view *viewer*)
       (send (send *viewer* :viewsurface :parent) :draw-objects
	     :clear nil :flush nil)
       (if move-target
	   (send-message move-target cascaded-link :draw-on :flush nil :size 100)))
     t))
  (:move-joints-avoidance
   (vel-pos vel-rot &rest args
	    &key
	    link-list
	    move-target
	    (weight (fill (instantiate float-vector (length link-list)) 1))
	    (null-space)
	    (avoid-nspace-gain 0.1)
	    (avoid-weight-gain 1.0)
	    (avoid-collision-null-gain 1.0)
	    (avoid-collision-joint-gain 0.3)
	    (avoid-collision-nspace-distance 50)
	    (avoid-collision-joint-distance 500)
	    (tmp-len) (tmp-len2) (debug-view)
	    &allow-other-keys)
   (let* ((len (length link-list))
	  (wmat (make-matrix len len))
	  joint-angle-limit-nspace
	  joint-angle-limit-weight
	  joint-angle-limit-weight-old
	  la dav dtheta 
	  (nul-col (instantiate float-vector len))
	  (dav-col (instantiate float-vector len)))
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

     ;; weighting joint angle weight : W(dH/dq)
     (setq joint-angle-limit-weight 
       (scale avoid-weight-gain
	      (joint-angle-limit-weight (send-all link-list :joint) tmp-len) tmp-len))
     (when debug-view
       (warn "angle :")
       (dolist (j (send-all link-list :joint)) (warn "~7,1f " (send j :joint-angle))) (warn "~%")
       (warn "cost  :")
       (dotimes (i len) (warn "~7,3f " (elt joint-angle-limit-weight i))) (warn "~%")
       )
     (dotimes (i len)
       (setf (elt weight i)
	     (* (elt weight i) 
		(if (> (elt joint-angle-limit-weight i)
		       (elt joint-angle-limit-weight-old i))
		    (/ 1.0 (+ 1.0 (elt joint-angle-limit-weight i)))
		  1.0))))
     (dotimes (i len) (setf (elt joint-angle-limit-weight-old i)
			    (elt joint-angle-limit-weight i)))
     (dotimes (i len) (setf (aref wmat i i) (aref weight i)))
     (when debug-view
       (warn "weight:")
       (dotimes (i  len) (warn "~7,3f " (elt weight i))) (warn "~%"))

     ;; avoiding joint angle limit : a * W(dH/dq)
     (setq joint-angle-limit-nspace (joint-angle-limit-nspace (send-all link-list :joint) tmp-len2))
     (setq la
       (scale avoid-nspace-gain
	      (transform wmat joint-angle-limit-nspace tmp-len2) tmp-len2))
     (when debug-view
       (warn "nspace:")
       (dotimes (i  len) (warn "~7,3f " (elt la i))) (warn "~%"))
     ;;
     (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
     ;; avoiding self collision
     (if (or (> avoid-collision-null-gain 0.0)
	     (> avoid-collision-joint-gain 0.0))
       (let* ((ret (send* self :collision-avoidance args))
	      (collision-limit (min avoid-collision-joint-distance 
				    (* 0.5 (norm vel-pos))))
	      (pair-list (send self :collision-avoidance-link-pair))
	      (pair-len (length pair-list)) pair 
	      (col-args-list (send self :get :collision-avoidance-args-list))
	      col-args
	      (p-limit (cadr (memq :p-limit args)))
	      (r-limit (cadr (memq :r-limit args)))
	      col-link-list col-link-len col-scale 
	      vel-col (vel-rot (float-vector 0 0 0)) 
	      (tmp-vel-col (float-vector 0 0 0))
	      (ddav-len 0)
	      drag-coords np jacobi jacobi# nnul-col ddav-col)
	 (dotimes (i pair-len)
	   (setq vel-col (elt ret i)
		 pair (elt pair-list i)
		 np (elt (send self :get :collision-distance) i)
		 col-link-list (send self :link-list (car pair) (car link-list))
		 col-link-len (length col-link-list)
		 )
	   (if (> col-link-len (length col-args-list))
	       (setq col-args-list
		 (nconc col-args-list 
			(make-list (- col-link-len
				      (length col-args-list))))))
	   (setq col-args (elt col-args-list (1- col-link-len)))
	   (when np
	     (when debug-view
	       (warn "col   : ~6,2f ~A ~A~%" (car np) (send (car pair) :name) (send (cadr pair) :name)))
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
		 (send self :put :collision-avoidance-args-list col-args-list)
		 ))
	     (if (eq (car pair) (send move-target :parent))
		 (progn
		   (setq col-scale (* collision-limit (/ collision-limit (car np))))
		   (setq vel-pos (v+ vel-pos (scale col-scale vel-col vel-col) vel-pos)))
	       (progn
		 (setq drag-coords (make-cascoords :pos (elt np 1)))
		 (send (car pair) :assoc drag-coords)
		 (setq col-link-list (send self :link-list (car pair) (car link-list)))
		 (setq jacobi (send* self :calc-jacobian-from-link-list
				     col-link-list :move-target drag-coords
				     col-args))
		 (setq jacobi# (send* self :calc-inverse-jacobian jacobi col-args))
		 (when (> avoid-collision-null-gain 0.0)
		   (setq col-scale (* avoid-collision-nspace-distance (/ avoid-collision-nspace-distance (car np))))
		   (setq nnul-col
		     (send* self :calc-joint-angle-speed
			    (scale col-scale vel-col tmp-vel-col) vel-rot
			    :link-list col-link-list
			    :move-target drag-coords
			    :jacobi jacobi :jacobi# jacobi#
			    :p-limit p-limit :p-limit r-limit
			    col-args
			    ))
		   (when debug-view
		     (warn "nspace:")
		     (dotimes (i col-link-len) (warn "~7,3f " (elt nnul-col i))) (warn "~%"))

		   (dotimes (i (length nnul-col))
		     (setf (elt nul-col i) (+ (elt nul-col i) (elt nnul-col i))))
		   )
		 (when (> avoid-collision-joint-gain 0.0)
		   (setq col-scale (* collision-limit (/ collision-limit (car np))))
		   (setq ddav-col
		     (send* self :calc-joint-angle-speed
			    (scale col-scale vel-col tmp-vel-col) vel-rot
			    :link-list col-link-list
			    :move-target drag-coords
			    :jacobi jacobi :jacobi# jacobi#
			    :p-limit p-limit :p-limit r-limit
			    col-args))
		   (when debug-view
		     (warn "joint :")
		     (dotimes (i col-link-len) (warn "~7,3f " (elt ddav-col i))) (warn "~%"))

		   (dotimes (i (length ddav-col))
		     (setf (elt dav-col i) (+ (elt dav-col i) (elt ddav-col i))))
		   )
		 (incf ddav-len)
		 )))
	   )
	 (if (> ddav-len 0)
	     (setq dav-col (scale (/ 1.0 ddav-len) dav-col dav-col)
		   nul-col (scale (/ 1.0 ddav-len) nul-col nul-col)))
	 ))
     ;;
     (when (> avoid-collision-null-gain 0)
       (setq nul-col (scale avoid-collision-null-gain (transform wmat nul-col nul-col) nul-col))
       (when debug-view
	 (warn "nspace:")
	 (dotimes (i len) (warn "~7,3f " (elt nul-col i))) (warn "~%")))

     (send-message* self cascaded-link
		    :move-joints vel-pos vel-rot
		    :null-space (if null-space (v+ null-space (v+ la nul-col la) la) (v+ la nul-col la))
		    :angle-speed (if (> avoid-collision-joint-gain 0) (scale avoid-collision-joint-gain (transform wmat dav-col dav-col) dav-col))
		    :weight weight
		    args)
     ;;
     ;; done
     (when (and debug-view *viewer*)
       (if (or (> avoid-collision-null-gain 0.0)
	       (> avoid-collision-joint-gain 0.0))
	 (let ((pwidth (send *viewer* :viewsurface :line-width))
	       (psize (send *viewer* :viewsurface :point-size))
	       (pcolor (send *viewer* :viewsurface :color))
	       (pair-list (send self :collision-avoidance-link-pair))
	       np)
	   (send *viewer* :viewsurface :point-size 5)
	   (send *viewer* :viewsurface :line-width 1)
	   (send *viewer* :viewsurface :color #f(1 0 0))
	   (dotimes (i (length pair-list))
	     (setq np (elt (send self :get :collision-distance) i))
	     (unless np (return))
	     (if (<= (car np) -1.0)
		 (dolist (obj (elt pair-list i))
		   (dolist (e (cdr (send obj :edges)))
		     (send *viewer* :viewsurface :3d-line
			   (e . pvert) (e . nvert) :depth-test t))))
	     (send *viewer* :viewsurface :3d-line (elt np 1)(elt np 2) :depth-test t)
	     (send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
	     (send *viewer* :viewsurface :3d-point (elt np 2) :depth-test t))
	   (send *viewer* :viewsurface :line-width pwidth)
	   (send *viewer* :viewsurface :point-size psize)
	   (send *viewer* :viewsurface :color pcolor)
	   ))
       (if (not (memq :no-flush debug-view))
	   (send *viewer* :viewsurface :flush))
       )
     ))
  (:inverse-kinematics
   (target-coords &rest args 
		  &key (stop 50)
		  (link-list)
		  (move-target)
		  (debug-view) (warnp t)
		  (rotation-axis t)
		  (translation-axis t)
		  (look-at-target)
		  (thre 1) (rthre (deg2rad 1))
		  &allow-other-keys)
   (let* ((loop 0) (av0 (send-all (send-all link-list :joint) :joint-angle))
	  dif-pos dif-rot need-clear need-flush
	  ;;
	  (c (length link-list))
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
	  )
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
       (when debug-view
	 (warn ";; ~3d: dif-pos : ~a/(~a/~A), dif-rot : ~a/(~a/~A)~%"
	       loop dif-pos (norm dif-pos) thre dif-rot (norm dif-rot) rthre))
       ;; check loop end
       (if (and (> loop (/ stop 10))
		(< (norm dif-pos) thre)
		(if rotation-axis (< (norm dif-rot) rthre) t))
	   (return nil))
       ;; ik
       (when (and need-clear debug-view *viewer*)
	 (if need-clear (send *viewer* :viewsurface :clear)))
       (send-message* self cascaded-link
		      :move-joints-avoidance
		      dif-pos dif-rot :link-list link-list
		      :rotation-axis rotation-axis :translation-axis translation-axis
		      :warnp warnp :debug-view debug-view
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
		      args)
       (when (and debug-view  *viewer*)
	 (send target-coords :draw-on :flush nil :color #f(1 0 0))
	 (if need-flush (send *viewer* :viewsurface :flush)))
       (if look-at-target
	   (send self :head :look-at (send target-coords :worldpos)))
       )
     ;; update difference
     (setq dif-pos (send move-target :difference-position target-coords)
	   dif-rot (send move-target :difference-rotation target-coords :rotation-axis rotation-axis))
     (setq dif-pos (calc-dif-with-translation-axis
		    (v- (send move-target :inverse-transform-vector dif-pos)
			(send move-target :inverse-transform-vector (float-vector 0 0 0)))
		    translation-axis))
     (setq dif-rot (calc-dif-with-rotation-axis
		    (v- (send move-target :inverse-transform-vector dif-rot)
			(send move-target :inverse-transform-vector (float-vector 0 0 0)))
		    rotation-axis))

     ;; check solved or not
     (if (and (< (norm dif-pos) thre)  (< (norm dif-rot) rthre))
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
	 nil))
     ))
  )

(defun calc-dif-with-translation-axis (dif translation-axis &optional tmp-v0 tmp-v1 tmp-v2)
  (case translation-axis
	(:x (if tmp-v2 
		(progn
		  (setf (elt tmp-v2 0) (elt dif 1) (elt tmp-v2 1) (elt dif 2))
		  tmp-v2)
	      (float-vector (elt dif 1) (elt dif 2))))
	(:y (if tmp-v2
		(progn
		  (setf (elt tmp-v2 0) (elt dif 0) (elt tmp-v2 1) (elt dif 2))
		  tmp-v2)
	      (float-vector (elt dif 0) (elt dif 2))))
	(:z (if tmp-v2
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

(defun calc-dif-with-rotation-axis (dif rotation-axis &optional tmp-v0 tmp-v2)
  (case rotation-axis
	(:x (if tmp-v2
		(progn
		  (setf (elt tmp-v2 0) (elt dif 1) (elt tmp-v2 1) (elt dif 2))
		  tmp-v2)
		(float-vector (elt dif 1) (elt dif 2))))
	(:y (if tmp-v2
		(progn
		  (setf (elt tmp-v2 0) (elt dif 0) (elt tmp-v2 1) (elt dif 2))
		  tmp-v2)
	      (float-vector (elt dif 0) (elt dif 2))))
	(:z (if tmp-v2
		(progn
		  (setf (elt tmp-v2 0) (elt dif 0) (elt tmp-v2 1) (elt dif 1))
		  tmp-v2)
	      (float-vector (elt dif 0) (elt dif 1))))
	(nil (if tmp-v0 tmp-v0 (float-vector)))
	(t dif)))

(defun joint-angle-limit-weight (j-l &optional (res (instantiate float-vector (length j-l))))
  (let (j jang jmax jmin nang wang (e (deg2rad 1)) r)
    (dotimes (i (length j-l))
      (setq j (elt j-l i))
      (setq jang (deg2rad (send j :joint-angle))
	    jmax (deg2rad (send j :max-angle))
	    jmin (deg2rad (send j :min-angle)))
      (cond 
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
       (t
	(setq r
	      (abs (/ (* (expt (- jmax jmin) 2) (- (* 2 jang) jmax jmin))
		      (* 4 (expt (- jmax jang) 2) (expt (- jang jmin) 2)))))
	(if (and (< r 0.0) (> r 0.0)) (setq r 0.0))
	(setf (elt res i) r))
       )
      )
    res))

(defun joint-angle-limit-nspace (j-l &optional (res (instantiate float-vector (length j-l))))
  (let (j jang jmax jmin nang r)
    (dotimes (i (length j-l))
      (setq j (elt j-l i))
      (setq jang (send j :joint-angle)
	    jmax (send j :max-angle)
	    jmin (send j :min-angle))
      (cond
       ((or (eq (send j :name) :torso-waist-p)
	    (eq (send j :name) :waist-p))
	(setq nang (- jmin (j . min-angle)))
	(cond
	 ((= nang jmax) (setq nang (- jmax 1)))
	 ((= nang jmin) (setq nang (+ jmin 1))))
	(setf (elt res i)
	      (if (> jang nang)
		  (/ (- nang jang) (- jmax nang))
		(/ (- nang jang) (* 5.0(- nang jmin))))))
       (t
	(setf (elt res i) (/ (- (/ (+ jmax jmin) 2.0) jang)
			     (/ (- jmax jmin) 2.0)))))
      (setq r (* (if (plusp (elt res i)) 1 -1) (expt (elt res i) 2)))
      (if (and (< r 0.0) (> r 0.0)) (setq r 0.0))
      (setf (elt res i) r))
    res))

(in-package "GEOMETRY")

(provide :irtmodel "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.15  2009-03-07 01:53:00  k-okada
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
