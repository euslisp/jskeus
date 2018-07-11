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
   (&rest args &key ((:name n) "scene") ((:objects o)) ((:remove-wall w)) &allow-other-keys)
   "Create scene model"
   (send-super* :init args)
   (setq name :name)
   (setq objs o)
   (if w (delete-if
          #'(lambda (x)
              (and (stringp (send x :name))
                   (substringp "wall" (send x :name)))) o)
        objs)
   (dolist (obj objs)
     (send self :assoc obj)
     (when (eq (class obj) cascaded-coords)
       (setf (get obj :width) 2)
       (setf (get obj :color) #f(0.294118 0.572549 0.858823))
       (setf (get obj :size) 300)))
   self)
  (:objects () "Returns objects in the scene." objs)
  (:add-objects
   (objects)
   "Add objects to scene with identifiable names. Returns all objects."
   (dolist (obj objects)
     (when (send self :find-object (send obj :name))
       (error "object with name ~A already exists." (send obj :name))
       (return-from :add-objects nil)))
   (nconc objs objects))
  (:add-object (obj)
   "Add object to scene with identifiable name. Returns all objects."
   (send self :add-objects (list obj)))
  (:remove-objects
   (objs-or-names)
   "Remove objects or objects with given names from scene. Returns removed objects."
   (let (remove-objs)
     (dolist (o objs-or-names)
       (let ((obj?
              (if (stringp o) (send self :object o)
                (send self :object (send o :name)))))
         (unless obj?
           (error "object ~A not found in scene" o)
           (return-from :remove-objects nil))
         (push obj? remove-objs)))
     (setq objs (set-difference objs remove-objs))
     remove-objs))
  (:remove-object
   (obj-or-name)
   "Remove object or object with given name from scene. Returns removed object."
   (car (send self :remove-objects (list obj-or-name))))
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
  (:add-spots
   (spots)
   "Add spots to scene with identifiable names. All spots will be :assoc with this scene. Returns T if added spots successfly, otherwise returns NIL."
   (dolist (spot spots)
     (unless (and (stringp (send spot :name)) (not (null-string-p (send spot :name))))
       (error "spot name must not be empty!: ~A" (send spot :name))
       (return-from :add-spots nil))
     (unless (eq (class spot) cascaded-coords)
       (error "class of spot ~A must be cascaded-coords" (send (class spot) :name))
       (return-from :add-spots nil))
     (when (and (send spot :parent)
                (not (eq (send self :name) (send (send spot :parent) :name))))
       (error "parent of spot ~A must be ~A. please dissoc at first." (send spot :parent) (send self :name))
       (return-from :add-spots nil))
     (when (send self :spot (send spot :name))
       (error "spot with name ~A already exists." (send spot :name))
       (return-from :add-spots nil)))
   (dolist (s spots) (send self :assoc s))
   (nconc objs spots)
   t)
  (:add-spot
   (spot)
   "Add spot to scene with identifiable name. The spot will be :assoc with this scene. Returns T if spot is added successfly, otherwise returns NIL."
   (send self :add-spots (list spot)))
  (:remove-spots
   (spots)
   "Remove spots from this scene. All spots will be :dissoc with this scene. Returns removed spots."
   (dolist (spot spots)
     (unless (eq (class spot) cascaded-coords)
       (error "class of spot ~A must be cascaded-coords. To remove object(s) from scene, use :remove-object(s)." (send (class spot) :name))
       (return-from :remove-spots nil)))
   (let ((removed-spots (send self :remove-objects spots)))
     (dolist (s removed-spots)
       (send (send s :parent) :dissoc s))
     removed-spots))
  (:remove-spot
   (spot)
   "Remove spot from scene. the spot will be :dissoc with this scene. Returns removed spot."
   (car (send self :remove-spots (list spot))))
  (:spots
   ()
   "Return all spots in the scene."
   (append
    (mapcan #'(lambda(x)(if (derivedp x scene-model) (send x :spots) nil)) objs)
    (remove-if-not #'(lambda (o) (eq (class o) cascaded-coords)) objs)))
  ;;
  (:object
   (name)
   "Return an object of given name."
   (let ((r (send self :find-object name)))
     (case (length r)
       (0 (warning-message 1 "could not found object(~A)" name) nil)
       (1 (car r))
       (t (warning-message 1 "found multiple object ~A for given name(~A)" r name) (car r)))))
  (:spot
   (name)
   "Return a spot of given name."
   (let ((r (remove-if-not #'(lambda (s) (string= name (send s :name))) (send self :spots))))
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
