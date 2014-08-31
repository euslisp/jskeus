;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: $
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

(defclass scene-model
  :super cascaded-coords
  :slots (name objs))

(defmethod scene-model
  (:init
   (&rest args &key ((:name n) "scene") ((:objects o)))
   "Create scene model"
   (send-super* :init args)
   (setq name :name)
   (setq objs o)
   (dolist (obj objs)
     (send self :assoc obj)
     (when (eq (class obj) cascaded-coords)
       (setf (get obj :width) 2)
       (setf (get obj :color) #f(0.294118 0.572549 0.858823))
       (setf (get obj :size) 300)))
   self)
  (:objects () "Returns objects in the scene." objs)
  (:find-object
   (name)
   "Returns objects with given name."
   (append
    (mapcan
     #'(lambda(x)(if (derivedp x scene-model) (send x :find-object name) nil))
     objs)
    (mapcan #'(lambda (o)
		(if (string= name (send o :name)) (list o)))
	    objs)))
  (:spots
   (&optional name)
   "Returns spots in the scene. If name is given returns spot of given name."
   (append
    (mapcan
     #'(lambda(x)(if (derivedp x scene-model) (send x :spots name) nil))
     objs)
    (mapcan #'(lambda (o)
		(if (and (eq (class o) cascaded-coords)
			 (or (null name) (string= name (send o :name))))
		    (list o)))
	    objs)))
  ;;
  (:object
   (name)
   "Returns object of given name."
   (let ((r (send self :find-object name)))
     (case (length r)
       (0 (warning-message 1 "could not found object(~A)" name) nil)
       (1 (car r))
       (t (warning-message 1 "found multiple object ~A for given name(~A)" r name) (car r)))))
  (:spot
   (name)
   "Returns scene of given name."
   (let ((r (send self :spots name)))
     (case (length r)
       (0 (warning-message 1 "could not found spot(~A)" name) nil)
       (1 (car r))
       (t (warning-message 1 "found multiple spot ~A for given name(~A)" r name) (car r)))))
  ;;
  (:bodies
   ()
   (flatten (mapcar #'(lambda (x) (if (find-method x :bodies) (send x :bodies) x)) objs)))
  )


(in-package "GEOMETRY")

(provide :irtscene "$Id: $")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: $
;;;
;;; $Log$
