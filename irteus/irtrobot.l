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
  (let (bs b0 b1 c a (r/2 (/ radius 2)))
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
    ;; set a mass center of default-robot-link as a volume center
    (let* ((valid-bodies (remove-if #'(lambda (x)
					(and (> (send x :volume) 0) (< (send x :volume) 0))) ;; nan check
				    bs))
	   (bodies-centroid
	    (if (= (length valid-bodies) 1)
		(send (car valid-bodies) :centroid)
	      (scale (/ 1.0 (reduce #'+ (mapcar #'(lambda (x) (send x :volume)) valid-bodies)))
		     (reduce #'v+ (mapcar #'(lambda (x) (scale (send x :volume) (send x :centroid))) valid-bodies))))))
      (instance bodyset-link :init (make-cascoords)
		:bodies bs :name name :centroid bodies-centroid)
      )))

(defclass robot-model
  :super cascaded-link
  :slots (larm-end-coords rarm-end-coords
	  lleg-end-coords rleg-end-coords
	  head-end-coords torso-end-coords
	  larm-root-link rarm-root-link
	  lleg-root-link rleg-root-link
	  head-root-link torso-root-link
	  larm-collision-avoidance-links
	  rarm-collision-avoidance-links
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
	 (let* ((link-list (if (memq :link-list args)
			       (cadr (memq :link-list args))
			     (send self :link-list
				   (send self limb :end-coords :parent)
				   (send self limb :root-link))))
		(collision-avoidance-link-pair
		 (if (memq :collision-avoidance-link-pair args)
		     (cadr (memq :collision-avoidance-link-pair args))
		   (send self :collision-avoidance-link-pair-from-link-list link-list
			 :collision-avoidance-links (send self limb :collision-avoidance-links)))))
	   (send* self :inverse-kinematics (car args)
		  :move-target (if (memq :move-target args)
				   (cadr (memq :move-target args))
				 (send self limb :end-coords))
		  :collision-avoidance-link-pair collision-avoidance-link-pair
		  :link-list link-list
		  (cdr args))))
       (:move-end
	(send* self limb :inverse-kinematics args))
       (:move-end-rot
	(let ((coords (send self limb :end-coords :copy-worldcoords))
	      (angle (pop args)) (axis (pop args)) (wrt (pop args)))
	  (unless wrt (setq wrt :local))
	  (send* self limb :move-end
		 (send coords :rotate (deg2rad angle) axis wrt) args)))
       (:move-end-pos
	(let ((coords (send self limb :end-coords :copy-worldcoords))
	      (pos (pop args)) (wrt (pop args)))
	  (unless wrt (setq wrt :local))
	  (send* self limb :move-end (send coords :translate pos wrt) args)))
       (:look-at
	(let* ((move-target
                (if (memq :move-target args)
                    (cadr (memq :move-target args))
                  (send self limb :end-coords)))
               (link-list (if (memq :link-list args)
                              (cadr (memq :link-list args))
                            (send self :link-list
                                  (send move-target :parent)
                                  (send self limb :root-link))))
               (stop (if (memq :stop args) (cadr (memq :stop args)) 100))
               (warnp (if (memq :warnp args) (cadr (memq :warnp args)) t))
               dif-pos dif-rot p-dif-rot (count 0))
	  (while (and (< (incf count) stop)
		      (if p-dif-rot (> (norm (v- p-dif-rot dif-rot)) 1e-3) t))
	    (let* ((target-coords
                    (orient-coords-to-axis ;; orient target-coords to look-at direction
                     (make-coords :pos (car args))
                     (v- (car args) (send move-target :worldpos)))))
              (setq p-dif-rot dif-rot
                    dif-pos (send move-target :difference-position target-coords :translation-axis nil)
                    dif-rot (send move-target :difference-rotation target-coords :rotation-axis :z))
              (send self :inverse-kinematics-loop dif-pos dif-rot
                    :move-target move-target
                    :collision-avoidance-link-pair nil
                    :rotation-axis :z
                    :translation-axis nil
                    :link-list link-list
                    :loop count :stop stop ;;:thre nil
                    :target-coords target-coords ;; for debug-view
                    :debug-view (cadr (memq :debug-view args))))
            )
	  (if (and p-dif-rot (<= (norm (v- p-dif-rot dif-rot)) 1e-3))
	      (send self :angle-vector)
	    (when warnp
	      (warn ";; :look-at failed.~%")
	      (warn ";;     count : ~a~%" count)
	      (when p-dif-rot
		(warn ";; p-dif-rot : ~a/(~a)~%" p-dif-rot (norm p-dif-rot))
		(warn ";;   dif-rot : ~a/(~a)~%" dif-rot (norm dif-rot))
		(warn ";;      diff : ~a < ~A~%" (norm (v- p-dif-rot dif-rot)) 1e-3)
		))
	    nil)
	  ))
       (:collision-avoidance-links
	 (user::forward-message-to
	   (cdr (assoc (intern (format nil "~A-COLLISION-AVOIDANCE-LINKS"
				       (string-upcase limb))) (send self :slots)))
	   args))
       (:links (send self :limb limb nil))
       (:joint-list (send-all (send self :limb limb :links) :joint))
       (:gripper (send* self :gripper limb args))
       (:joint-order (send self :joint-order limb))
       (t
	(cond
	 ((or (null method) (send bodyset-link :method method))
	  (if method
	      (send-all (cdr (assoc (intern (string-upcase limb)) (send self :slots))) method)
	    (cdr (assoc (intern (string-upcase limb)) (send self :slots)))))
	 (t
	  (let ((limb-joint-name
		 (intern (format nil "~A-~A" (string-upcase limb) (string-upcase method)) *keyword-package*)))
	    (if (find-method self limb-joint-name)
		(user::forward-message-to (send self limb-joint-name) args)
	      (warn ";; error: cannot find method ~A~%" method))
	    ))))
       ) ;; case method
     )) ;; defmethod
  (:gripper
   (limb &rest args)
   (cond
    ((memq :links args)
     (sort (all-child-links (send self limb :end-coords :parent)) #'(lambda (a b) (string< (string (send a :name)) (string (send b :name))))))
    ((memq :joint-list args)
     (send-all (send self :gripper limb :links) :joint))
    ((memq :angle-vector args)
     (if (null (cdr args))
	 (send-all (send self :gripper limb :joint-list) :joint-angle)
       (map float-vector #'(lambda (x y) (send x :joint-angle y))
	    (send self :gripper limb :joint-list) (cadr args))))))
  ;;
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
  ;; override inverse-kinematics methods for look-at method
  (:inverse-kinematics
   (target-coords &rest args &key look-at-target &allow-other-keys)
   (if (atom target-coords) (setq target-coords (list target-coords)))
   (let ((ret (send-super* :inverse-kinematics target-coords args)))
     (if look-at-target
         (send self :head :look-at (send (car target-coords) :worldpos)))
     ret))
  (:inverse-kinematics-loop
   (dif-pos dif-rot &rest args
            &key target-coords debug-view look-at-target
            &allow-other-keys)
   (if (atom target-coords) (setq target-coords (list target-coords)))
   (let ((ret
          (send-super* :inverse-kinematics-loop dif-pos dif-rot
                       :target-coords target-coords args)))
     (if (and debug-view look-at-target)
         (send self :head :look-at (send (car target-coords) :worldpos)))
     ret))
  (:torque-vector
   (&key (force-list) (moment-list) (target-coords)
         (debug-view nil)
	 (jvv (instantiate float-vector (calc-target-joint-dimension joint-list))) ;; [rad/s] or [m/s]
	 (jav (instantiate float-vector (calc-target-joint-dimension joint-list)))) ;; [rad/s^2] or [m/s^2]
   (unless (every #'null (send self :legs)) ;; for legged robot, set default external force and moment at the end-effectors
     (let ((weight-force (* (send self :weight) 1e-6 (elt *g-vec* 2)))) ;; weight[g] * 1e-6 * gvec[mm/s^2] = force[N]
       (unless target-coords
	 (dolist (limb '(:rleg :lleg))
	   (push (send self limb :end-coords) target-coords)))
       (unless force-list ;; force [N]
	 (let ((total-force/2 (* weight-force 0.5)))
	   ;; hypothesis : lleg force = rleg force, lleg force + rleg force + gravity force = 0
	   (dolist (limb '(:rleg :lleg))
	     (push (float-vector 0.0 0.0 (case limb
					   ((:rleg :lleg) total-force/2)
					   (t 0.0))) force-list))
	   ))
       (unless moment-list ;; moment [Nm]
	 (let ((total-moment (v* (scale 1e-3
					(v- (send self :centroid nil)
					    (apply #'midpoint 0.5 (send-all target-coords :worldpos))))
				 (float-vector 0 0 weight-force))))
	   ;; hypothesis : gravity force moment + rleg moment + lleg moment = 0, minimal internal moments <-> pseudo-inverse <-> lleg moment = rleg moment
	   (dolist (limb '(:rleg :lleg))
	     (push (scale 0.5 total-moment) moment-list))
	   ))))
   (unless (= (length force-list) (length moment-list)
	      (length target-coords))
     (warn ";; ERROR : list length differ : force-list ~A moment-list ~A target-coords ~A~%"
	   (length force-list) (length moment-list) (length target-coords))
     (return-from :torque-vector jvv))
   (mapcar #'(lambda (fv mv tc)
	       (send (send tc :parent) :ext-force fv)
	       ;; calc moment-offset caused by fv ;
	       ;; current irtdyna.l's requires moment around the origin
	       (let* ((c (send (send tc :parent) :centroid))
		      (moment-offset (v* (scale 1e-3 (send tc :worldpos)) fv)))
		 (send (send tc :parent) :ext-moment (v+ mv moment-offset))))
	   force-list moment-list target-coords)
   (send-super :calc-torque
               :debug-view debug-view
               :jvv jvv :jav jav)
   )
  (:calc-force-from-joint-torque
   (limb all-torque &key (move-target (send self limb :end-coords)) (use-torso))
   (let* ((link-list
	   (send self :link-list
		 (send move-target :parent)
		 (unless use-torso (car (send self limb :links)))))
	  (jacobian
	   (send self :calc-jacobian-from-link-list
		 link-list
		 :move-target move-target
		 :rotation-axis (list t)
		 :translation-axis (list t)))
	  (torque (instantiate float-vector (length link-list))))
     (dotimes (i (length link-list))
       (setf (elt torque i)
	     (elt all-torque (position (send (elt link-list i) :joint) (send self :joint-list)))))
     (transform (send self :calc-inverse-jacobian (transpose jacobian))
		torque)))
  (:draw-torque
   (vwer &key flush (width 2) (size 100) (color (float-vector 1 0.3 0)) (warning-color (float-vector 1 0 0)) (torque-threshold nil) (torque-vector (send self :torque-vector)))
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
	  (when (and torque-threshold (> (abs rtorque) torque-threshold))
	    (setq width (* 2 width) color warning-color))
	  (send vwer :viewsurface :line-width width)
	  (send vwer :viewsurface :color color)
	  (send vwer :draw-circle
		(make-coords :pos (send j :child-link :worldpos) :rot rot)
		:radius (* size (abs rtorque)) :arrow t :arc (deg2rad 330))
	  ))
    (send self :joint-list)
    (coerce torque-vector cons))
   (if flush (send vwer :viewsurface :flush))
   );; draw-torque
  ;; fullbody inverse kinematics for legged robot
  ;;   necessary args : target-coords, move-target, and link-list must include legs' (or leg's) parameters
  ;;                    ex. (send *robot* :fullbody-inverse-kinematics (list rarm-tc rleg-tc lleg-tc) :move-target (list rarm-mt rleg-mt lleg-mt) :link-list (list rarm-ll rleg-ll lleg-ll))
  (:fullbody-inverse-kinematics
    (target-coords
     &rest args
     &key (move-target) (link-list)
          ;; default robot root-link 6dof parameters : min, max, root-link-virtual-joint-weight, and weight
          (min (float-vector -500 -500  -500 -20 -20 -10))
          (max (float-vector  500  500   25  20  20  10))
          (root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5))
          ;; default cog-jacobian parameters : target-centroid-pos, cog-gain, and centroid-thre
          (target-centroid-pos (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos))) ;; <- target centroid position.
          (cog-gain 1.0) ;; <- cog gain for null-space calculation. cog-gain should be over zero.
          (centroid-thre 5.0) ;; cog convergence threshould [mm]
     &allow-other-keys)
    (with-append-root-joint ;; inverse-kinematics with base-link
     (link-list-with-robot-6dof self link-list
                                :joint-class 6dof-joint
                                :joint-args (list :min min :max max))
     (send* self :inverse-kinematics target-coords
            :move-target move-target :link-list link-list-with-robot-6dof
            :cog-gain cog-gain :centroid-thre centroid-thre :target-centroid-pos target-centroid-pos
            :weight #'(lambda (union-link-list)
			(let ((tmp-weight (fill (instantiate float-vector (send self :calc-target-joint-dimension union-link-list)) 1.0)))
			  (dotimes (i 6) (setf (elt tmp-weight i) (elt root-link-virtual-joint-weight i)))
			  (if (memq :weight args)
			      (let ((tmp-weight2 (funcall (cadr (memq :weight args)) union-link-list)))
				(dotimes (i (length tmp-weight2)) (setf (elt tmp-weight i) (* (elt tmp-weight i) (elt tmp-weight2 i))))))
			  tmp-weight))
            args)
     ))
  (:joint-order
   (limb &optional jname-list)
   (let ((joint-list (mapcar #'(lambda (x) (cons x (find-if #'(lambda (y) (eq y (send self x))) (send self limb :joint-list)))) (mapcan #'(lambda (x) (if (substringp (format nil "~A-" (symbol-name limb)) (string x)) (list x)))(send self :methods))))
         j (result) name)
     (unless jname-list
       (setq jname-list
	     (case limb
	       ((:larm :rarm) '("collar" "shoulder" "elbow" "wrist"))
	       ((:lleg :rleg) '("crotch" "knee" "ankle"))
	       ((:torso) '("chest" "weist"))
	       ((:head) '("neck" "head")))))
     (dolist (jname jname-list) ;; look for joint corresponds to jname
       (while (substringp (string-upcase jname) (setq name (symbol-name (caar joint-list))))
         (push
          (read-from-string
           (format nil ":~A" (subseq name (1- (length name)))))
          j)
         (pop joint-list))
       (nreverse j)
       (push j result)
       (setq j nil))
     (nreverse result)))
  )

(in-package "GEOMETRY")

(provide :irtrobot "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.4  2009-12-27 08:46:18  ueda
;;; IMPORTANT commit. change arguments of inverse-kinematics-loop from coordinates to position and rotation velicity
;;;
;;; Revision 1.3  2009/10/10 12:50:17  nozawa
;;; replace ik method, :move-joints -> :inverse-kinematics-loop
;;;
;;; Revision 1.2  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
