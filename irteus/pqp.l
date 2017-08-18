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

(in-package "GEOMETRY")

(export '(pqp-collision-check pqp-collision-distance pqp-collision-check-objects))

(defconstant PQP_ALL_CONTACTS	1)
(defconstant PQP_FIRST_CONTACT 2)

(defmethod cascaded-coords
  (:make-pqpmodel
   (&key (fat 0) ((:faces fs) (send self :faces)))
   (let ((m (pqpmakemodel))
	 vs v1 v2 v3 (id 0))
     (setf (get self :pqpmodel) m)
     (pqpbeginmodel m)
     (dolist (f fs)
       (dolist (poly (face-to-triangle-aux f))
	 (setq vs (send poly :vertices)
	       v1 (send self :inverse-transform-vector (first vs))
	       v2 (send self :inverse-transform-vector (second vs))
	       v3 (send self :inverse-transform-vector (third vs)))
	 (when (not (= fat 0))
	   (setq v1 (v+ v1 (scale fat (normalize-vector v1)))
		 v2 (v+ v2 (scale fat (normalize-vector v2)))
		 v3 (v+ v3 (scale fat (normalize-vector v3)))))
	 (pqpaddtri m v1 v2 v3 id)
	 (incf id)
	 )
       )
     (pqpendmodel m)
     m))
  )

(defun pqp-collision-check (model1 model2
				       &optional (flag PQP_FIRST_CONTACT) &key (fat 0) (fat2 nil))
  "Check collision between model1 and model2 using PQP.
   If return value is 0, no collision.
   Otherwise (return value is 1), collision."
  (let ((m1 (get model1 :pqpmodel))
	(m2 (get model2 :pqpmodel))
	(r1 (send model1 :worldrot))
	(t1 (send model1 :worldpos))
	(r2 (send model2 :worldrot))
	(t2 (send model2 :worldpos)))
    (if (null fat2) (setq fat2 fat))
    (if (null m1) (setq m1 (send model1 :make-pqpmodel :fat fat)))
    (if (null m2) (setq m2 (send model2 :make-pqpmodel :fat fat2)))
    (pqpcollide r1 t1 m1 r2 t2 m2 flag)
    ))

(defun pqp-collision-distance (model1 model2
				      &key (fat 0) (fat2 nil) (qsize 2))
  "Calculate collision distance between model1 and model2 using PQP.
   Return value is (list [distance] [nearest point on model1] [nearest point on model2]).
   If collision occurs, [distance] is 0 and nearest points are insignificant values."
  (let ((m1 (get model1 :pqpmodel))
	(m2 (get model2 :pqpmodel))
	(r1 (send model1 :worldrot))
	(t1 (send model1 :worldpos))
	(r2 (send model2 :worldrot))
	(t2 (send model2 :worldpos))
	(p1 (float-vector 0 0 0))
	(p2 (float-vector 0 0 0))
	r)
    (if (null fat2) (setq fat2 fat))
    (if (null m1) (setq m1 (send model1 :make-pqpmodel :fat fat)))
    (if (null m2) (setq m2 (send model2 :make-pqpmodel :fat fat2)))
    (setq r (pqpdistance r1 t1 m1 r2 t2 m2 p1 p2 qsize))
    (list r p1 p2)
    ))

(defun pqp-collision-check-objects (obj1 obj2 &key (fat 0.2))
  "return nil or t"
  (dolist (o1 obj1)
    (dolist (o2 obj2)
      (if (> (pqp-collision-check o1 o2 PQP_FIRST_CONTACT :fat fat) 0)
	  (return-from pqp-collision-check-objects t))))
  nil)

#|
(setq c1 (make-cube 100 100 100))
(setq c2 (make-cube 100 100 100))
(send c1 :set-color :red)
(send c2 :set-color :blue)
(send c1 :locate #f(300 0 0))
(if (not (boundp '*irtview*)) (make-irtviewer))
(objects (list c1 c2))
(setq i 0)
(do-until-key
  (send c1 :locate (float-vector (* 200 (sin (/ i 100.0))) 0 0) :world)
  (setq r (pqp-collision-distance c1 c2))
  (warn "num of contacts ~3d / ~7,3f~%" (pqp-collision-check c1 c2) (car r))
  (print (send c1 :vertices))
  (send c2 :worldcoords)
  (send *irtviewer* :draw-objects)
  (send *irtviewer* :viewer :viewsurface :line-width 4)
  (send *irtviewer* :viewer :viewsurface :color #f(1 1 1))
  (send *irtviewer* :viewer :draw-line (elt r 1) (elt r 2))
  (send *irtviewer* :viewer :flush)
  (incf i)
  )
|#

(provide :pqp "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.3  2009-08-05 16:22:58  ueda
;;; change polygon -> poly for thread safe
;;;
;;; Revision 1.2  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
