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

(require :irtmodel)

(defun make-default-robot-link (len radius axis name &optional extbody)
  (let (bs b0 b1 c a l (r/2 (/ radius 2)))
    (setq b0 (make-cylinder r/2 radius))
    (setq b1 (make-cube r/2 r/2 len))

    (setq c (make-cascoords))
    (case axis
      (:x (setq a #f(1 0 0)))
      (:y (setq a #f(0 1 0)))
      (:z (setq a #f(0 0 1)))
      (:-x (setq a #f(-1 0 0)))
      (:-y (setq a #f(0 -1 0)))
      (:-z (setq a #f(0 0 -1)))
      (t  (setq a axis)))
    (if (> (norm (v* a #f(0 0 -1))) 0)
	(send c :orient (acos (v. a #f(0 0 -1))) (v* a #f(0 0 -1)) :world))
    (send b0 :transform c)
    (send b0 :translate (float-vector 0 0 (- r/2)))
    (send b1 :translate (float-vector 0 0 (/ len -2)) :locate)
    (send b0 :assoc b1)
    (send b0 :set-color :red)
    (send b1 :set-color :green)
    (setq bs (list b0 b1))
    (when extbody 
      (dolist (b extbody) (send b1 :assoc b))
      (setq bs (append bs extbody)))
    (setq l (instance bodyset-link :init (make-cascoords) :bodies bs :name name))
    l))

(defclass robot-model
  :super cascaded-link
  :slots (larm-end-coords rarm-end-coords
	  lleg-end-coords rleg-end-coords
	  head-end-coords torso-end-coords
	  larm-root-link rarm-root-link
	  lleg-root-link rleg-root-link
	  head-root-link torso-root-link
	  larm rarm lleg rleg torso head))

(defmethod robot-model
  (:limb
   (limb method &rest args)
   (let (ret)
     (case method
       (:end-coords 
	(user::forward-message-to 
	 (cdr (assoc (intern (format nil "~A-END-COORDS" (string-upcase limb)))
		     (send self :slots)))
	 args))
       (:root-link
	(user::forward-message-to 
	 (cdr (assoc (intern (format nil "~A-ROOT-LINK" (string-upcase limb))) (send self :slots)))
	 args))
       (:angle-vector
	(if args
	    (progn
	      (mapcar #'(lambda (l a)
			  (send l :joint :joint-angle a))
		      (send self limb) (coerce (car args) cons))
	      (send self limb :angle-vector))
	  (coerce (mapcar #'(lambda (l) (send l :joint :joint-angle))
			  (send self limb)) float-vector)))
       (:inverse-kinematics
	(send* self :inverse-kinematics (car args)
	       :move-target (if (memq :move-target args)
			      (cadr (memq :move-target args))
			      (send self limb :end-coords))
	       :link-list (if (memq :link-list args)
			      (cadr (memq :link-list args))
			    (send self :link-list
				  (send self limb :end-coords :parent)
				  (send self limb :root-link)))
	       (cdr args)))
       (:move-end
	(send* self limb :inverse-kinematics args))
       (:move-end-rot
	(let ((coords (send self limb :end-coords :copy-worldcoords))
	      (angle (pop args)) (axis (pop args)) (wrt (pop args)))
	  (send* self limb :move-end
		 (send coords :rotate (deg2rad angle) axis wrt) args)))
       (:move-end-pos
	(let ((coords (send self limb :end-coords :copy-worldcoords))
	      (pos (pop args)) (wrt (pop args)))
	  (unless wrt (setq wrt :local))
	  (send* self limb :move-end (send coords :translate pos wrt) args)))
       (:look-at
	(let (dif-pos dif-rot target v p-dif-rot (count 0))
	  (setq target (make-coords :pos (car args)))
	  (setq v (normalize-vector (v- (send target :worldpos)
					(send head-end-coords :worldpos))))
	  (send target :rotate
		(acos (v. (matrix-column (send target :worldrot) 2) v))
		(v* (matrix-column (send target :worldrot) 2) v))
	  (while (and (< (incf count) 10)
		      (if p-dif-rot (> (norm (v- p-dif-rot dif-rot)) 1e-3) t))
	    (setq p-dif-rot dif-rot)
	    (setq dif-pos (send head-end-coords :difference-position target)
		  dif-rot (send head-end-coords :difference-rotation target :rotation-axis :z))
	    (send self :move-joints #f(0 0 0) dif-rot
		  :move-target head-end-coords
		  :rotation-axis :z
		  :translation-axis nil
		  :link-list (if (memq :link-list args)
				 (cadr (memq :link-list args))
			       (send self :link-list
				     (send self limb :end-coords :parent)
				     (send self limb :root-link)))
		  :debug-view nil))
	  ))
       (t
	(cond
	 ((or (null method) (send bodyset-link :method method))
	  (if method
	      (send-all (cdr (assoc (intern (string-upcase limb)) (send self :slots))) method)
	    (cdr (assoc (intern (string-upcase limb)) (send self :slots)))))
	 (t
	  (setq ret (member (format nil "~A-~A" (string limb) (string method))
			    (send-all (cdr (assoc (intern (string-upcase limb))
						  (send self :slots))) :joint)
			    :key #'(lambda (a) (symbol-name (send a :name)))
			    :test #'string=))
	  (if ret (user::forward-message-to (car ret) args)
	    (progn (warn ";; error: cannot find method ~A~%" method)))
	  )))
       ) ;; case method
     )) ;; defmethod
  (:larm (&rest args) 
	 (unless args (setq args (list nil))) (send* self :limb :larm args))
  (:rarm (&rest args)
	 (unless args (setq args (list nil))) (send* self :limb :rarm args))
  (:lleg (&rest args)
	 (unless args (setq args (list nil))) (send* self :limb :lleg args))
  (:rleg (&rest args)
	 (unless args (setq args (list nil))) (send* self :limb :rleg args))
  (:head (&rest args)
	 (unless args (setq args (list nil))) (send* self :limb :head args))
  (:torso (&rest args)
	 (unless args (setq args (list nil))) (send* self :limb :torso args))
  (:arms (&rest args) (list (send* self :larm args) (send* self :rarm args)))
  (:legs (&rest args) (list (send* self :lleg args) (send* self :rleg args)))
  (:look-at-hand (l/r) (send self :head :look-at (send self l/r :end-coords :worldpos)))
		 
  )

(in-package "GEOMETRY")

(provide :irtrobot "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.2  2009-02-17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
