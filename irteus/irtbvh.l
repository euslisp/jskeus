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

;(require :irtrobot)

;;
;; bvh joint and link
;;

(defclass bvh-link
  :super bodyset-link
  :slots (type offset channels neutral))

(defmethod bvh-link
  (:init (name typ offst chs parent children)
         "create link for bvh model"
         (let ((s 30) bs)
           (setq type typ offset offst channels chs)

           (setq bs (list (make-cube s s s)
			  (make-cube (* 0.1 s) (* 0.5 s) (* 0.5 s)  :pos (float-vector (* 0.60 s) 0 0))
			  (make-cylinder (* 0.5 s) (* 0.60 s))))
	   (dolist (b (cdr bs)) (send (car bs) :assoc b))
           (dolist (child children)
             (let* ((v (v- (normalize-vector child)))
                    (c (make-cylinder (/ s 3.0) (norm child)))
                    (a (normalize-vector (v* v #f(0 0 1)))) m r)
               (if (eps= (norm a) 0.0) (setq a #f(1 0 0)))
	       (setq m (send (send (make-coords) :rotate (acos (v. v #f(0 0 -1))) a) :rot))
	       (dotimes (i 12)
		 (push (transform m
				  (float-vector (* (/ s 3) (sin (* -2pi (/ i 12.0))))
						(* (/ s 3) (cos (* -2pi (/ i 12.0))))
						0)) r))
	       (setq c (make-prism r child))
               (send (car bs) :assoc c)
               (setq bs (nconc bs (list c)))))
           (send-super :init (make-cascoords) :bodies bs :name name)
           (case type
             ('end  (send self :bodies :set-color :red))
             ('root (send self :bodies :set-color :darkgreen))
             (t     (send self :bodies :set-color :green)))

           (send self :locate
                 (if parent (v+ (send parent :worldpos) offset) offset) :world)
           (if parent (send parent :assoc self))
           (send self :worldcoords)
           self))
  (:type () type)
  (:offset () offset)
  (:channels () channels)
  )

(defclass bvh-sphere-joint
  :super sphere-joint
  :slots (axis-order bvh-offset-rotation))
(defmethod bvh-sphere-joint
  (:init (&rest args &key (order (list :z :x :y)) ((:bvh-offset-rotation bvh-rotation) (unit-matrix 3)) &allow-other-keys)
         "create joint for bvh model"
         (setq axis-order order
	       bvh-offset-rotation bvh-rotation)
         (send-super* :init args))
  (:joint-angle-bvh (&optional v)
   (send self :joint-angle-bvh-impl v (unit-matrix 3)))
  (:joint-angle-bvh-offset (&optional v)
   (send self :joint-angle-bvh-impl v bvh-offset-rotation))
  (:joint-angle-bvh-impl
   (v bvh-offset)
   (let (ja m)
     (when v
       (setq ja (map float-vector #'deg2rad v))
       (setq m (rotation-matrix (elt ja 2) (elt axis-order 2)))
       (rotate-matrix m (elt ja 1) (elt axis-order 1) t m)
       (rotate-matrix m (elt ja 0) (elt axis-order 0) t m)
       (send self :joint-angle (map float-vector #'rad2deg (matrix-log (m* bvh-offset m))))
       )
     (map cons #'rad2deg (matrix-to-euler-angle (m* (transpose bvh-offset) (send child-link :rot)) axis-order))))
  (:axis-order () axis-order)
  (:bvh-offset-rotation () bvh-offset-rotation)
  )

(defclass bvh-6dof-joint
  :super 6dof-joint
  :slots (scale axis-order bvh-offset-rotation))
(defmethod bvh-6dof-joint
  (:init (&rest args &key (order (list :x :y :z :z :x :y)) ((:scale scl)) ((:bvh-offset-rotation bvh-rotation) (unit-matrix 3)) &allow-other-keys)
         (setq axis-order order
	       scale (if scl scl 1.0)
               bvh-offset-rotation bvh-rotation)
         (send-super* :init args))
  (:joint-angle-bvh (&optional v)
   (send self :joint-angle-bvh-impl v (unit-matrix 3)))
  (:joint-angle-bvh-offset (&optional v)
   (send self :joint-angle-bvh-impl v bvh-offset-rotation))
  (:joint-angle-bvh-impl
   (v bvh-offset)
   (let (m r p)
     (when v
       (setq r (map float-vector #'deg2rad (subseq v 3 6))
             p (scale scale (subseq v 0 3)))
       (setq m (rotation-matrix (elt r 2) (elt axis-order 5)))
       (rotate-matrix m (elt r 1) (elt axis-order 4) t m)
       (rotate-matrix m (elt r 0) (elt axis-order 3) t m)
       (send self :joint-angle
             (concatenate float-vector
			  (transform bvh-offset p)
			  (map cons #'rad2deg (matrix-log (m* bvh-offset-rotation m))))))
     (concatenate cons
		  (transform (transpose bvh-offset) (scale 0.001 (send child-link :pos)))
		  (map cons #'rad2deg (matrix-to-euler-angle (m* (transpose bvh-offset) (send child-link :rot)) (subseq axis-order 3 6))))))
  (:axis-order () axis-order)
  (:bvh-offset-rotation () bvh-offset-rotation)
  )

(defclass bvh-robot-model
  :super robot-model
  :slots ())

(defmethod bvh-robot-model
  (:init
   (&rest args &key tree coords ((:scale scl)))
   "create robot model for bvh model"
   (send-super* :init :coords (make-cascoords) args) ;; to avoid pass :coords keyword to upper-class
   (send self :make-bvh-link tree :scale scl)
   (if coords (send self :transform coords))
   (setq links (reverse links)
         joint-list (reverse joint-list))
   (send self :init-ending)
   self)
  (:make-bvh-link
   (tree &key parent ((:scale scl)))
   (let (joint-type joint-name offset channels alink ajoint)
     (setq joint-type (caar tree)
           joint-name (cadr (pop tree))
           offset (cadr (pop tree))
           channels (cdr (pop tree)))
     (if (null scl) (setq scl 1.0))
     (setq alink (instance bvh-link :init joint-name joint-type
                           offset
			   (car channels)
                           parent
                           (mapcar #'(lambda (tr) (cadr (assoc 'offset (cdr tr)))) tree)
                           ))
     (push alink links)
     (unless (eql joint-type 'end)
       (let (joint-cls)
         (when (eql joint-type 'root)
             (setq parent (instance bodyset-link :init (make-cascoords) :name 'root-parent :bodies (list (make-cube 10 10 10))))
	     (send parent :assoc alink)
	     (send self :assoc parent))
         (case (car channels)
           (3 (setq joint-cls bvh-sphere-joint))
           (6 (setq joint-cls bvh-6dof-joint))
           (t (warning-message 1 "unknown joint type ~A~%" channels)))
         (setq ajoint (instance joint-cls :init :name joint-name
                                :child-link alink :parent-link parent
				:bvh-offset-rotation (send self :bvh-offset-rotate joint-name)
                                :order (reduce #'(lambda (x y) (substitute (car y) (cdr y) x)) (list (cdr channels) (cons :x 'xrotation) (cons :y 'yrotation) (cons :z 'zrotation)))
                                :scale scl))
         (push ajoint joint-list)))
     (dolist (tr tree)
       (send self :make-bvh-link tr :parent alink :scale scl))
     parent))
  (:angle-vector
   (&optional vec
              (angle-vector (instantiate float-vector (calc-target-joint-dimension joint-list))))
   (let ((i 0) j)
     (dolist (j joint-list)
       (if vec
           (case (send j :joint-dof)
             (1 (send j :joint-angle (elt vec i)))
             (3 (let ((v (subseq vec i (+ i (send j :joint-dof)))))
		  (send j :joint-angle-bvh-offset v)))
             (t (send j :joint-angle-bvh-offset (subseq vec i (+ i (send j :joint-dof)))))
             ))
       (dotimes (k (send j :joint-dof))
         (setf (elt angle-vector i) (elt (send j :joint-angle-bvh-offset) k))
         (incf i)))
     angle-vector))
  ;;
  (:dump-joints
   (links &key (depth 0) (strm *standard-output*))
   (let ((tabspace (fill (make-string (* depth 2)) #\space))
	 offset channels axis-order)
     (dolist (l links)
       (setq offset (send l :offset)
	     channels (send l :channels)
	     axis-order (send l :joint :axis-order))
       (format strm "~A~A ~A~%" tabspace (string-upcase (send l :type)) (send l :joint :name))
       (format strm "~A{~%" tabspace)
       (format strm "  ~AOFFSET ~0,6f ~0,6f ~0,6f~%" tabspace (elt offset 0) (elt offset 1) (elt offset 2))
       (format strm "  ~ACHANNELS ~A" tabspace channels)
       (dolist (axis axis-order)
	 (case axis
	   (:x (format strm " Xrotation"))
	   (:y (format strm " Yrotation"))
	   (:z (format strm " Zrotation"))
	   (t  (format strm " ~A" axis))))
       (format strm "~%")
       (if (send l :child-links)
	   (send self :dump-joints (reverse (send l :child-links))
		      :depth (1+ depth) :strm strm)
	 (let* ((end (find-if #'(lambda (x) (derivedp x bvh-link)) (send l :descendants)))
		(offset (send end :offset)))
	   (format strm "  ~AEnd Site~%" tabspace)
	   (format strm "  ~A{~%" tabspace)
	   (format strm "    ~A OFFSET ~0,6f ~0,6f ~0,6f~%" tabspace (elt offset 0) (elt offset 1) (elt offset 2))
	   (format strm "  ~A}~%" tabspace)
	   ))
       (format strm "~A}~%" tabspace)
     )))
  (:dump-hierarchy
   (&optional (strm *standard-output*))
   (format strm "HIERARCHY~%")
   (send self :dump-joints (list (car links)) :strm strm)
   (format strm "MOTION~%")
   (format strm "Frames: 100~%")
   (format strm "Frame Time: 0.033333~%")
   )
  (:dump-motion
   (&optional (strm *standard-output*))
   (let (a)
     (dolist (l links)
       (setq a (send l :joint :joint-angle-bvh))
       (dotimes (i (length a))
	 (format strm "~0,6f " (elt a i))))
     (format strm "~%")))
  (:copy-state-to
   (robot)
   (warning-message 1 "subclass's responsibility :copy-state-to~%")
   )
  (:fix-joint-angle
   (i limb joint-name joint-order a)
   a)
  (:fix-joint-order (jo limb)
   jo)
  (:bvh-offset-rotate
   (name)
   (unit-matrix 3))
  ) ; bvh-robot-model

;;;
;;; bvh parser
;;;
;;; https://groups.google.com/group/comp.lang.lisp/browse_thread/thread/e9f2dd4f0a17810e/0be7675a3295f4e6?hl=ky&

(defun parse-bvh-sexp (src &key ((:scale scl)))
  (if (null scl) (setq scl 1.0))
  (if (atom src)
      src
    (let (op result)
      (while (setq op (pop src))
        (case op
          ((root joint end)
           (push (cons (list op (pop src)) (parse-bvh-sexp (pop src) :scale scl)) result))
          (offset
           (push (list op (scale scl (float-vector (pop src) (pop src) (pop src)))) result))
          (channels
           (push (cons op (do* ((i (pop src)) (r (list i)))
                              ((< (decf i) 0) (reverse r))
                            (push (pop src) r))) result))
          (t
           (if (symbolp op)
               (push (cons op (parse-bvh-sexp (pop src) :scale scl)) result)
             (push op result)))))
      (reverse result))))

(defun read-bvh (fname &key scale)
  "read bvh file"
  (let ((*readtable* (copy-readtable))
        (eof (gensym)) hierarchy motions frames frame-time)
    (set-syntax-from-char #\# #\;) ;; commnet
    (set-syntax-from-char #\: #\ ) ;; fake :
    (set-syntax-from-char #\} #\)) ;; delimiter

    (set-macro-character
     #\{
     #'(lambda (strm ch)
         (read-delimited-list #\} strm t)))

    (with-open-file
     (strm fname :direction :input)

     (read strm) ;; check if hierarchy
     (do* ((item nil (read strm nil eof)))
         ((or (eql item eof) (eql item 'motion))
          (setq hierarchy (cdr (nreverse hierarchy))))
       (push item hierarchy))
     (setq hierarchy (car (parse-bvh-sexp hierarchy :scale scale)))

     (read strm) ;; frames
     (setq frames (read strm))

     (read strm) (read strm);; frame time
     (setq frame-time (read strm))
     (do* ((line "" (read-line strm nil nil)))
         ((not line))
       (let ((v (eval (read-from-string (format nil "(float-vector ~A)" line)))))
         (if (> (length v) 0) (push v motions))))
     (setq motions (nreverse motions))

     (warning-message 2 ";; Reading motion capture data from \"~A\"~%" fname)
     (warning-message 2 ";;  ~A joints, ~A frames~%" (length (car motions)) (length motions))

     `((hierarchy ,hierarchy)
       (motions ((frames ,frames) (frame-time ,frame-time) ,motions)))
     ) ;; with-open-file
    )); read-bvh

(defun make-bvh-robot-model (bvh-data &rest args)
  (instance* bvh-robot-model :init :tree bvh-data args))

(defclass motion-capture-data
  :super propertied-object
  :slots (frame model animation))
(defmethod motion-capture-data
  (:init
   (fname &key (coords (make-coords)) ((:scale scl)))
   (let (bvh-data)
     (setq bvh-data (read-bvh fname :scale scl))
     (setq model (make-bvh-robot-model (cadr (assoc 'hierarchy bvh-data)) :coords coords :scale scl))
     (setq animation (car (last (cadr (assoc 'motions bvh-data)))))
     (setq frame 0)
     self))
  (:model (&rest args) (forward-message-to model args))
  (:animation (&rest args) (forward-message-to animation args))
  (:frame (&optional f)
   (if f (setq frame f))
   (let ((av (elt animation frame)) (i 0))
     (dolist (j (send model :joint-list))
       (send j :joint-angle-bvh (subseq av i (incf i (send j :joint-dof))))))
   frame)
  (:frame-length () (length animation))
  (:animate
   (&rest args &key (start 0) (step 1) (end (send self :frame-length)) (interval 20)  &allow-other-keys)
   (let ()
     (send self :frame start)
     (catch :animate
       (while t
	 (send self :frame frame)
	 (send (get *viewer* :pickviewer) :look-all)
         (dolist (obj (objects))
           (if (and (not (derivedp obj bvh-robot-model))
                    (derivedp obj robot-model))
               (send (send self :model) :copy-state-to obj)))
	 (incf frame step)
	 (if (>= frame (send self :frame-length)) (throw :animate nil))
	 (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil))
	 (if interval (unix:usleep (* 1000 interval)))
	 (x::window-main-one))) ;; catch
       ))
  ;;
  ) ; motion-capture-data

(defun bvh2eus (fname &rest args &key ((:objects obj) nil) &allow-other-keys)
  "read bvh file and anmiate robot model in the viewer

for Supported bvh data, such as

- CMU motion capture database
   (https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture/cmu-bvh-conversion)

- The TUM Kitchen Data Set
   (http://ias.cs.tum.edu/download/kitchen-activity-data)

Use

(tum-bvh2eus \"Take 005.bvh\")   ;; tum
(rikiya-bvh2eus \"A01.bvh\")     ;; rikiya
(cmu-bvh2eus \"01_01.bvh\")      ;; cmu


Other Sites are:
 (http://www.mocapdata.com/page.cgi?p=free_motions)
 (http://www.motekentertainment.com/)
 (http://www.mocapclub.com/Pages/Library.htm)

(bvh2eus \"poses.bvh\")
 "
  (let (b)
    (unless (listp obj) (setq obj (list obj)))
    (setq b (apply #'load-mcd fname args))
    (objects (append (list (send b :model)) obj))
    (send b :animate)
    b))

(defmethod bvh-robot-model
  (:init-end-coords ()
   (setq larm-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last larm))))) links)
	 rarm-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last rarm))))) links)
	 lleg-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last lleg))))) links)
	 rleg-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last rleg))))) links)
	 torso-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last torso))))) links)
	 head-end-coords
	 (find-if  #'(lambda (x) (and (eq (send x :name) 'site)
				      (eq (send x :parent) (car (last head))))) links)))
  (:init-root-link ()
   (setq larm-root-link (car larm) rarm-root-link (car rarm)
	 lleg-root-link (car lleg) rleg-root-link (car rleg)
	 torso-root-link (car torso)
	 head-root-link (car head)))
  ) ; defmethod bvh-robot-model


(defclass rikiya-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod rikiya-bvh-robot-model
  (:init (&rest args)
   (send-super* :init :coords (make-coords :rpy (float-vector pi/2 0 pi/2)) args)
   (setq larm (list
	       (find 'leftcollar    links :key #'(lambda (x) (send x :name)))
	       (find 'leftshoulder  links :key #'(lambda (x) (send x :name)))
	       (find 'leftelbow     links :key #'(lambda (x) (send x :name)))
	       (find 'leftwrist     links :key #'(lambda (x) (send x :name))))
	 rarm (list
	       (find 'rightcollar   links :key #'(lambda (x) (send x :name)))
	       (find 'rightshoulder links :key #'(lambda (x) (send x :name)))
	       (find 'rightelbow    links :key #'(lambda (x) (send x :name)))
	       (find 'rightwrist    links :key #'(lambda (x) (send x :name))))
	 lleg (list
	       (find 'lefthip       links :key #'(lambda (x) (send x :name)))
	       (find 'leftknee      links :key #'(lambda (x) (send x :name)))
	       (find 'leftankle     links :key #'(lambda (x) (send x :name))))
	 rleg (list
	       (find 'righthip      links :key #'(lambda (x) (send x :name)))
	       (find 'rightknee     links :key #'(lambda (x) (send x :name)))
	       (find 'rightankle    links :key #'(lambda (x) (send x :name))))
	 torso (list
		(find 'chest        links :key #'(lambda (x) (send x :name))))
	 head  (list
		(find 'neck        links :key #'(lambda (x) (send x :name)))
		(find 'head        links :key #'(lambda (x) (send x :name)))))
   ;;
   (send self :init-end-coords)
   (send self :init-root-link)
   ;;;
   self) ;; init
  (:larm-collar (&rest args)   (forward-message-to (elt larm 0) args))
  (:larm-shoulder (&rest args) (forward-message-to (elt larm 1) args))
  (:larm-elbow (&rest args)    (forward-message-to (elt larm 2) args))
  (:larm-wrist (&rest args)    (forward-message-to (elt larm 3) args))
  (:rarm-collar (&rest args)   (forward-message-to (elt rarm 0) args))
  (:rarm-shoulder (&rest args) (forward-message-to (elt rarm 1) args))
  (:rarm-elbow (&rest args)    (forward-message-to (elt rarm 2) args))
  (:rarm-wrist (&rest args)    (forward-message-to (elt rarm 3) args))
  (:lleg-crotch (&rest args)   (forward-message-to (elt lleg 0) args))
  (:lleg-knee (&rest args)     (forward-message-to (elt lleg 1) args))
  (:lleg-ankle (&rest args)    (forward-message-to (elt lleg 2) args))
  (:rleg-crotch (&rest args)   (forward-message-to (elt rleg 0) args))
  (:rleg-knee (&rest args)     (forward-message-to (elt rleg 1) args))
  (:rleg-ankle (&rest args)    (forward-message-to (elt rleg 2) args))
  (:torso-chest (&rest args)   (forward-message-to (elt torso 0) args))
  (:head-neck (&rest args)     (forward-message-to (elt head 0) args))
  (:copy-joint-to
   (robot limb joint &optional (sign 1))
   (if (find-method robot (intern (format nil "~A-~A-R" (symbol-name limb) (symbol-name joint)) "KEYWORD"))
       (send robot limb (intern (format nil "~A-R" (symbol-name joint)) "KEYWORD") :joint-angle
             (* sign (elt (send self limb joint :joint :joint-angle) 2))))
   (if (find-method robot (intern (format nil "~A-~A-P" (symbol-name limb) (symbol-name joint)) "KEYWORD"))
       (send robot limb (intern (format nil "~A-P" (symbol-name joint)) "KEYWORD") :joint-angle
             (elt (send self limb joint :joint :joint-angle) 0)))
   (if (find-method robot (intern (format nil "~A-~A-Y" (symbol-name limb) (symbol-name joint)) "KEYWORD"))
       (send robot limb (intern (format nil "~A-Y" (symbol-name joint)) "KEYWORD") :joint-angle
             (* sign (elt (send self limb joint :joint :joint-angle) 1)))))
  (:copy-state-to
   (robot)
   ;;(warning-message 2 ";; irteus copy-state-to in irtbvh.l robot:~A~%" robot)
   (let (sign)
     (dolist (arm '(:larm :rarm))
       (dolist (joint '(:collar :shoulder :elbow :wrist))
	 (if (and (boundp 'euscollada-robot)
		  (derivedp robot euscollada-robot))
	     (send self :copy-joint-to robot arm joint)
	   (send self :copy-joint-to robot arm joint (case arm (:rarm -1) (t 1))))))
     (dolist (leg '(:lleg :rleg))
       (dolist (joint '(:crotch :knee :ankle))
	 (if (and (boundp 'euscollada-robot)
		  (derivedp robot euscollada-robot))
	     (send self :copy-joint-to robot leg joint)
	   (send self :copy-joint-to robot leg joint (case leg (:rleg -1) (t 1))))))
     (send self :copy-joint-to robot :torso :chest)
     (send self :copy-joint-to robot :head :neck -1)
     (send robot :newcoords (send self :transformation (car links) :world))
     )) ;; copy-state-to
  ) ;defmethod rikiya-bvh-robot-model

(defclass tum-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod tum-bvh-robot-model
  (:init (&rest args)
   (send-super* :init :coords (make-coords :rpy (float-vector pi/2 0 0)) args)
   (setq larm (list
	       (find 'sb.l   links :key #'(lambda (x) (send x :name)))
	       (find 'oa.l   links :key #'(lambda (x) (send x :name)))
	       (find 'ua.l   links :key #'(lambda (x) (send x :name)))
	       (find 'ha.l   links :key #'(lambda (x) (send x :name)))
	       (find 'fi.l   links :key #'(lambda (x) (send x :name))))
	 rarm (list
	       (find 'sb.r   links :key #'(lambda (x) (send x :name)))
	       (find 'oa.r   links :key #'(lambda (x) (send x :name)))
	       (find 'ua.r   links :key #'(lambda (x) (send x :name)))
	       (find 'ha.r   links :key #'(lambda (x) (send x :name)))
	       (find 'fi.r   links :key #'(lambda (x) (send x :name))))
	 lleg (list
	       (find 'os.l   links :key #'(lambda (x) (send x :name)))
	       (find 'us.l   links :key #'(lambda (x) (send x :name)))
	       (find 'fu.l   links :key #'(lambda (x) (send x :name)))
	       (find 'fb.l   links :key #'(lambda (x) (send x :name))))
	 rleg (list
	       (find 'os.r   links :key #'(lambda (x) (send x :name)))
	       (find 'us.r   links :key #'(lambda (x) (send x :name)))
	       (find 'fu.r   links :key #'(lambda (x) (send x :name)))
	       (find 'fb.r   links :key #'(lambda (x) (send x :name))))
	 torso (list
		(find 'ulw   links :key #'(lambda (x) (send x :name)))
		(find 'olw   links :key #'(lambda (x) (send x :name)))
		(find 'ubw   links :key #'(lambda (x) (send x :name)))
		(find 'obw   links :key #'(lambda (x) (send x :name)))
		(find 'uhw   links :key #'(lambda (x) (send x :name)))
		(find 'brk   links :key #'(lambda (x) (send x :name))))
	 head  (list
		(find 'ohw   links :key #'(lambda (x) (send x :name)))
		(find 'ko    links :key #'(lambda (x) (send x :name)))))

   ;;
   (send self :init-end-coords)
   (send self :init-root-link)
   ;;
   self) ;; init
  ) ;defmethod tum-bvh-robot-model

(defclass cmu-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod cmu-bvh-robot-model
  (:init (&rest args)
   (send-super* :init :coords (make-coords :rpy (float-vector pi/2 0 pi/2)) args)
   (setq larm (list
	       (find 'leftshoulder  links :key #'(lambda (x) (send x :name)))
	       (find 'leftarm       links :key #'(lambda (x) (send x :name)))
	       (find 'leftforearm   links :key #'(lambda (x) (send x :name)))
	       (find 'lefthand      links :key #'(lambda (x) (send x :name)))
	       (find 'lthumb       links :key #'(lambda (x) (send x :name))))
	 rarm (list
	       (find 'rightshoulder links :key #'(lambda (x) (send x :name)))
	       (find 'rightarm      links :key #'(lambda (x) (send x :name)))
	       (find 'rightforearm  links :key #'(lambda (x) (send x :name)))
	       (find 'righthand     links :key #'(lambda (x) (send x :name)))
	       (find 'rthumb       links :key #'(lambda (x) (send x :name))))
	 lleg (list
	       (find 'lhipjoint     links :key #'(lambda (x) (send x :name)))
	       (find 'leftupleg     links :key #'(lambda (x) (send x :name)))
	       (find 'leftleg	    links :key #'(lambda (x) (send x :name)))
	       (find 'leftfoot      links :key #'(lambda (x) (send x :name)))
	       (find 'lefttoebase   links :key #'(lambda (x) (send x :name))))
	 rleg (list
	       (find 'rhipjoint     links :key #'(lambda (x) (send x :name)))
	       (find 'rightupleg    links :key #'(lambda (x) (send x :name)))
	       (find 'rightleg	    links :key #'(lambda (x) (send x :name)))
	       (find 'rightfoot     links :key #'(lambda (x) (send x :name)))
	       (find 'righttoebase  links :key #'(lambda (x) (send x :name))))
	 torso (list
		(find 'lowerback    links :key #'(lambda (x) (send x :name)))
		(find 'spine        links :key #'(lambda (x) (send x :name)))
		(find 'spine1      links :key #'(lambda (x) (send x :name))))
	 head  (list
		(find 'neck        links :key #'(lambda (x) (send x :name)))
		(find 'neck1       links :key #'(lambda (x) (send x :name)))
		(find 'head        links :key #'(lambda (x) (send x :name)))))
   ;;
   (send self :init-end-coords)
   (send self :init-root-link)
   ;;;
   self) ;; init
  ) ;defmethod cmu-bvh-robot-model

(defun load-mcd (fname &key (scale) (coords) (bvh-robot-model-class bvh-robot-model) &allow-other-keys)
  "load motion capture data"
  (let ((bvh-robot-model bvh-robot-model-class))
    (instance motion-capture-data :init fname :coords coords :scale scale)
    ))

(defun rikiya-bvh2eus (fname &rest args)
  "read rikiya bvh file and anmiate robot model in the viewer
(rikiya-bvh2eus \"A01.bvh\")
"
  (apply #'bvh2eus fname :scale 10.0 :bvh-robot-model-class rikiya-bvh-robot-model args))

(defun cmu-bvh2eus (fname &rest args)
  "read cmu bvh file and anmiate robot model in the viewer

CMU motion capture database
 (https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture/cmu-bvh-conversion)

(cmu-bvh2eus \"01_01.bvh\" :scale 100.0)
"
  (apply #'bvh2eus fname :scale 100.0 :bvh-robot-model-class cmu-bvh-robot-model args))

(defun tum-bvh2eus (fname &rest args)
  "read tum file and anmiate robot model in the viewer

The TUM Kitchen Data Set
 (http://ias.cs.tum.edu/download/kitchen-activity-data)

(tum-bvh2eus \"Take 005.bvh\" :scale 10.0)
"
  (apply #'bvh2eus fname :scale 10.0 :bvh-robot-model-class tum-bvh-robot-model args))


(defun rikiya-file (&key (num 1) (cls 'a))
  (format nil "~A/bvh/rikiya/rbvh_~A/~A~0,2d.bvh"
          (unix::getenv "EUSDIR")
          cls
          (string-upcase
           (format nil "~A" cls)) num))

(defun tum-kitchen-file (&key (num 1) (cls 0))
  (format nil "~A/bvh/tum-kitchen/~A-~A/poses.bvh"
          (unix::getenv "EUSDIR")
          cls num))

(defun cmu-mocap-file (&key (num 1) (cls 1))
  (format nil "~A/bvh/cmu-mocap/~0,2d/~0,2d_~0,2d.bvh"
          (unix::getenv "EUSDIR")
          cls cls num))

(in-package "GEOMETRY")

(provide :irtbvh "$Id: ")
