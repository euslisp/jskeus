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

(require :irtmath)

(in-package "GEOMETRY")

(export '(midcoords bodyset))

(defun midcoords (p c1 c2)
  (let ()
    (make-coords :pos (midpoint p (send c1 :worldpos) (send c2 :worldpos))
		 :rot (user::midrot p (send c1 :worldrot) (send c2 :worldrot)))
    ))

(defmethod coordinates
  (:axis (axis)
	 (send self :rotate-vector
	       (case axis
		     (:x #f(1 0 0))
		     (:y #f(0 1 0))
		     (:z #f(0 0 1))))
	 )
  (:difference-position 
   (coords &key (translation-axis t))
   (let ((dif-pos
	  (send self :inverse-transform-vector (send coords :worldpos))))
     (case
      translation-axis
      ((:x :xx) (setf (elt dif-pos 0) 0))
      ((:y :yy) (setf (elt dif-pos 1) 0))
      ((:z :zz) (setf (elt dif-pos 2) 0))
      ((:xy :yx) (setf (elt dif-pos 0) 0) (setf (elt dif-pos 1) 0))
      ((:yz :zy) (setf (elt dif-pos 1) 0) (setf (elt dif-pos 2) 0))
      ((:zx :xz) (setf (elt dif-pos 2) 0) (setf (elt dif-pos 0) 0)))
     dif-pos))
  (:difference-rotation
   (coords &key (rotation-axis t))
   (let (dif-rotmatrix dif-rot a0 a1)
     (case
      rotation-axis
      ((:x :y :z)
       (setq a0 (send self :axis rotation-axis)
	     a1 (send coords :axis rotation-axis))
       (setq dif-rot 
             (transform (transpose (send self :worldrot))
                        (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))))
      ((:xx :yy :zz)
       (let ((axis (case rotation-axis (:xx :x) (:yy :y) (:zz :z))) a2)
	 (setq a0 (send self :axis axis)
	       a1 (send coords :axis axis)
	       a2 (v- (send coords :axis axis)))
	 (setq a1 (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))
	 (setq a2 (scale (acos (v. a0 a2)) (normalize-vector (v* a0 a2))))
	 (if (< (norm a1) (norm a2)) (setq dif-rot a1) (setq dif-rot a2))
         (setq dif-rot (transform (transpose (send self :worldrot)) dif-rot))))
      (nil
       (setq dif-rot (float-vector 0 0 0)))
      (t
       (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
       (setq dif-rot (user::matrix-log dif-rotmatrix))
       ))
     dif-rot))
  )


;; BEGIN:: Redefine for rotm3*
(defmethod coordinates
  (:move-to (c &optional (wrt :local) &aux cc)
     (unless (coordinates-p c) (error "coordinates expected for :move-to"))
     (cond ((or (memq wrt '(:local local)) (eq wrt self))
	    (setq cc (transform-coords self c))
	    (send self :newcoords cc))
	   ((or (memq wrt '(:parent parent :world world))
		(equal wrt *world-coords*))
	    (send self :newcoords c))
	   ((coordinates-p wrt)
	    (setq cc (transform-coords (send wrt :worldcoords) c))
	    (transform-coords (send (send self :parentcoords)
				    :inverse-transformation) cc cc)
	    (send self :newcoords cc))))
  (:transformation   (c2 &optional (wrt :local))
   (setq c2 (send c2 :worldcoords)) 
   (let* ((c1 (send self :worldcoords)) 
	  (inv (send c1 :inverse-transformation))
	  xw)
      (cond
         ((or (memq wrt '(:local local)) (eq wrt self))
	   (transform-coords inv c2 inv))
         ((or (memq wrt '(:parent parent :world world))
	      (eq wrt *world-coords*))
	   (transform-coords c2 inv inv))
         ((coordinates-p wrt)
	   (setq xw (send wrt :worldcoords))
	   (transform-coords c2 inv inv)
	   (transform-coords (send xw :inverse-transformation) inv inv)
	   (transform-coords inv xw inv))
         (t (send self :error ":transform wrt?" wrt)))
      inv))
  (:transform  (c &optional (wrt :local))
   (cond
      ((or (memq wrt '(local :local)) (eq wrt self))	;multiply c from the right
	(transform-coords self c self))
      ((or (memq wrt '(parent :parent world :world))
	   (eq wrt *world-coords*))
	(transform-coords c self self))	;;multiply c from the left
      ((coordinates-p wrt)
	(transform-coords (send wrt :inverse-transformation) self self)
	(transform-coords c self self)
	(transform-coords (send wrt :worldcoords) self self))
      (t (send self :error ":transform wrt?" wrt)))
   (send self :newcoords rot pos))
  )

(defmethod cascaded-coords 
 (:worldcoords  ()	;calculate rot and pos in the world
   (when changed
      (if parent
	  (transform-coords (send parent :worldcoords) self worldcoords)
	  (send worldcoords :replace-coords self))
      (send self :update)
      (setf changed nil))
   worldcoords)
 (:transformation (c2 &optional (wrt :local))
   (let* ((w2 (send c2 :worldcoords))
	  (w1 (send self :worldcoords))
	  (w1inv (send w1 :inverse-transformation))
	  (c1inv) (xw))
      (cond
        ((or (memq wrt '(:local local)) (eq wrt self))
	 (transform-coords w1inv w2 w1inv))
	((or (memq wrt '(:parent parent)) (eq wrt parent))	;nil?
	 (setq c1inv (send-super :inverse-transformation))
	 (transform-coords w2 c1inv c1inv)
	 (transform-coords w1inv c1inv c1inv)
	 (transform-coords self c1inv c1inv))
	((or (memq wrt '(world :world))  (equal wrt *world-coords*))
	 (transform-coords w2 w1inv w1inv))
        ((coordinates-p wrt)
	 (setq xw (send wrt :worldcoords))
	 (transform-coords w1inv xw w1inv)
	 (transform-coords w2 w1inv w1inv)
	 (transform-coords (send xw :inverse-transformation) w1inv w1inv))
      (t (send self :error ":transform wrt?" wrt)))))
 (:transform  (c &optional (wrt :local))
   (cond
    ((or (memq wrt '(:local local)) (eq wrt self))	;multiply c from the right
	(transform-coords self c self))
    ((or (memq wrt '(:parent parent)) (eq wrt parent))	;nil?
	(transform-coords c self self))	;;multiply c from the left
    ((or (memq wrt '(world :world))  (equal wrt *world-coords*))
	(let ((pc (send self :parentcoords)))
	   (transform-coords pc self self)
	   (transform-coords c self self)
	   (transform-coords (send pc :inverse-transformation) self self)))
    ((coordinates-p wrt)
	(let ((pc (send self :parentcoords)))
	   (transform-coords pc self self)
	   (transform-coords (send wrt :inverse-transformation) self self)
	   (transform-coords c self self)
	   (transform-coords (send wrt :worldcoords) self self)
	   (transform-coords (send pc :inverse-transformation) self self)))
    (t (send self :error ":transform wrt?" wrt)))
   (send self :newcoords rot pos))
 (:move-to (c &optional (wrt :local) &aux cc)
     (unless (coordinates-p c) (error "coordinates expected for :move-to"))
     (cond ((or (memq wrt '(:local local)) (eq wrt self))
	    (setq cc (transform-coords self c))
	    (send self :newcoords cc))
	   ((or (memq wrt '(:parent parent)) (eq wrt parent))
	    (send self :newcoords c))
	   ((or (memq wrt '(:world world)) (equal wrt *world-coords*))
	    (setq cc (transform-coords
			(send (send self :parentcoords)
				:inverse-transformation) c))
	    (send self :newcoords cc))
	   ((coordinates-p wrt)
	    (setq cc (transform-coords (send wrt :worldcoords) c))
	    (transform-coords (send (send self :parentcoords)
				    :inverse-transformation) cc cc)
	    (send self :newcoords cc))))
  )

(defun transform-coords (c1 c2 &optional
			    (c3 (let ((dim (send c1 :dimension)))
				  (instance coordinates 
				    :newcoords (unit-matrix dim)
				    (instantiate float-vector dim)))))
  (if (eq c1 c3)
      (v+ (coordinates-pos c1)
	  (transform (coordinates-rot c1) (coordinates-pos c2))
	  (coordinates-pos c3))
    (v+ (coordinates-pos c1)
	(transform (coordinates-rot c1) (coordinates-pos c2) (coordinates-pos c3))
	(coordinates-pos c3)))
  (user::rotm3* (coordinates-rot c1) (coordinates-rot c2) (coordinates-rot c3))
  c3)


;; END:: Redefine for rotm3*

;;;
(defmethod coordinates
  (:rotate-vector (v &optional r) 
		  (if r (transform rot v r) (transform rot v))))
(defmethod cascaded-coords 
  (:rotate-vector (v &optional r) 
		  (send (send self :worldcoords) :rotate-vector v r)))

(defmethod coordinates
  (:inverse-transform-vector
   (vec &optional v3a v3b m33)	;vec in world coordinates->local
   (let ((inv-rot (if m33 (transpose rot m33) (transpose rot))))
     (if (and v3a v3b m33)
	 (v- (transform inv-rot vec v3a) (transform inv-rot pos v3b) v3a)
       (v- (transform inv-rot vec) (transform inv-rot pos))))))
(defmethod cascaded-coords 
  (:inverse-transform-vector
   (v &optional v3a v3b m33)
   (send (send self :worldcoords) :inverse-transform-vector v v3a v3b m33)))


(defclass bodyset
  :super cascaded-coords
  :slots 
  ((bodies :type cons)
  ))

(defmethod bodyset
  (:init 
   (coords
    &rest args
    &key (name (intern (format nil "bodyset~A" (sys::address self)) "KEYWORD"))
    ((:bodies bs))
    &allow-other-keys)
   (let ()
     (send-super* :init :name name args)
     (send coords :copy-worldcoords self)
     (setq bodies bs)
     (when bodies
       ;; モデリングにおける制約：bodiesの先頭にはツリーのルートを入れておく
       (send self :assoc (car bodies)))
     (send self :worldcoords)
     self))
  (:bodies (&rest args) (user::forward-message-to-all bodies args))
  (:faces () (flatten (send-all bodies :faces)))
  (:worldcoords
   ()
   (when changed
     (send-super :worldcoords)
     (dolist (b bodies) (send b :worldcoords)))
   worldcoords)
  (:draw-on (&rest args) (dolist (b bodies) (send* b :draw-on args)))
  )

(defun face-to-triangle-aux (f)
  (cond
   ((= (length (send f :edges)) 3) (list f))
   (t (face-to-triangle f))))

(defun face-to-triangle (f)
  (let* ((start 0) 
	 (poly (face-to-tessel-triangle f start)))
    (while (null poly)
      (if (>= start (length (send f :vertices)))
	  (return-from face-to-triangle nil))
      (incf start)
      (setq poly (face-to-tessel-triangle f start)))
    (cons poly 
	  (face-to-triangle-aux (face-to-triangle-rest-polygon f start)))))

;; return polygon if triangable ,unless return nil.
;; this function = sharpp + trianglable + insidep
(defun face-to-tessel-triangle (f num)
  (let* ((vers (send f :vertices))
	 (len (length (cdr vers)))
	 (v0 (nth (mod (1- (+ num len)) len) vers))
	 (v1 (nth (mod num len) vers))
	 (v2 (nth (mod (1+  num) len) vers)))
    (unless (> (v. (v* (v- v1 v0) (v- v2 v1)) (send f :normal)) 0)
	    (return-from face-to-tessel-triangle nil))
    (let ((poly (instance polygon :init :vertices (list v0 v1 v2)))
	  (rvers (remove v0 (remove v1 (remove v2 vers)))))
      (if (every #'(lambda (x) (eq ':outside (send poly :insidep x))) rvers)
	  poly
	nil))))

(defun face-to-triangle-rest-polygon (f num)
  (let* ((vers (butlast (send f :vertices)))
	 (len (length vers))
	 (rvers (remove (nth (mod num len) vers) vers)))
    (instance polygon :init :vertices rvers)))

(provide :irtgeo "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.10  2009-09-24 10:07:34  k-okada
;;; fix differnece-position/rotation to calculate local difference
;;;
;;; Revision 1.9  2009/09/10 16:02:39  nozawa
;;; fix calculation of dif-pos (in :difference-position)
;;;
;;; Revision 1.8  2009/09/10 15:24:04  k-okada
;;; update :difference-translation
;;;
;;; Revision 1.7  2009/08/07 12:13:20  k-okada
;;; support rotation-axis :xx :yy :zz
;;;
;;; Revision 1.6  2009/05/15 07:45:48  eus
;;; remove :box from bodyset
;;;
;;; Revision 1.5  2009/05/15 01:02:27  nozawa
;;; add :box to bodyset
;;;
;;; Revision 1.4  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.3  2009/01/05 03:48:31  k-okada
;;; move defmethod polygon :reset-normal to jskgeo.l
;;;
;;; Revision 1.2  2008/11/11 14:01:54  k-okada
;;; error handling when normalize-vector #f(0 0 0), again
;;;
;;; Revision 1.1  2008/09/18 18:11:00  k-okada
;;; add irteus
;;;
;;;
