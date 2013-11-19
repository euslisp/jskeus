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
(require :irtdyna)

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
        (if (send self :head :links) (send* self :inverse-kinematics-loop-for-look-at limb args)))
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
  (:inverse-kinematics-loop-for-look-at
   (limb &rest args)
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
  (:look-at-hand
   (l/r)
   "(l/r) ;; l/r supports :rarm, :larm, :arms, and '(:rarm :larm) ;; "
   (send self :look-at-target
         (mapcar #'(lambda (x) (send self x :end-coords))
                 (cond
                  ((consp l/r) l/r)
                  ((eq l/r :arms) '(:rarm :larm))
                  (t (list l/r))))))
  ;; override inverse-kinematics methods for look-at method
  (:inverse-kinematics
   (target-coords
    &rest args
    &key look-at-target
         (move-target)
         (link-list
          (if (atom move-target)
              (send self :link-list (send move-target :parent))
            (mapcar #'(lambda (mt) (send self :link-list (send mt :parent))) move-target)))
    &allow-other-keys)
   "(target-coords
    &rest args
    &key look-at-target
         (move-target)
         (link-list
          (if (atom move-target)
              (send self :link-list (send move-target :parent))
            (mapcar #'(lambda (mt) (send self :link-list (send mt :parent))) move-target)))
    &allow-other-keys)
    look-at-target suppots t, nil, float-vector, coords, list of float-vector, list of coords
    link-list is set by default based on move-target -> root link link-list"
   (if (atom target-coords) (setq target-coords (list target-coords)))
   (prog1
       (send-super* :inverse-kinematics target-coords
                    :move-target move-target :link-list link-list args)
     (send self :look-at-target look-at-target :target-coords target-coords)
     ))
  (:inverse-kinematics-loop
   (dif-pos dif-rot
    &rest args
    &key target-coords debug-view look-at-target
         (move-target)
         (link-list
          (if (atom move-target)
              (send self :link-list (send move-target :parent))
            (mapcar #'(lambda (mt) (send self :link-list (send mt :parent))) move-target)))
         &allow-other-keys)
   "(dif-pos dif-rot
    &rest args
    &key target-coords debug-view look-at-target
         (move-target)
         (link-list
          (if (atom move-target)
              (send self :link-list (send move-target :parent))
            (mapcar #'(lambda (mt) (send self :link-list (send mt :parent))) move-target)))
         &allow-other-keys)
    look-at-target suppots t, nil, float-vector, coords, list of float-vector, list of coords
    link-list is set by default based on move-target -> root link link-list"
   (if (atom target-coords) (setq target-coords (list target-coords)))
   (prog1
       (send-super* :inverse-kinematics-loop dif-pos dif-rot
                    :link-list link-list :move-target move-target
                    :target-coords target-coords args)
     (if debug-view (send self :look-at-target look-at-target :target-coords target-coords))
     ))
  ;; look-at-target supports -> t, nil, float-vector, coords, list of float-vector, list of coords
  (:look-at-target
   (look-at-target &key (target-coords))
   (cond
    ((float-vector-p look-at-target)
     (send self :head :look-at look-at-target))
    ((coordinates-p look-at-target)
     (send self :head :look-at (send look-at-target :worldpos)))
    ((and (consp look-at-target)
          (every #'identity (mapcar #'float-vector-p look-at-target)))
     (send self :head :look-at
           (scale (/ 1.0 (length look-at-target)) (reduce #'v+ look-at-target :initial-value #f(0 0 0)))))
    ((and (consp look-at-target)
          (every #'identity (mapcar #'coordinates-p look-at-target)))
     (send self :head :look-at
           (scale (/ 1.0 (length look-at-target)) (reduce #'v+ (send-all look-at-target :worldpos) :initial-value #f(0 0 0)))))
    ((null look-at-target))
    (t (send self :head :look-at (send (car target-coords) :worldpos)))
    ))
  ;; init-pose
  (:init-pose () (send self :angle-vector (instantiate float-vector (send self :calc-target-joint-dimension (cdr (send self :links))))))
  (:torque-vector
   (&key (force-list) (moment-list) (target-coords)
         (debug-view nil)
	 (jvv (instantiate float-vector (calc-target-joint-dimension joint-list))) ;; [rad/s] or [m/s]
	 (jav (instantiate float-vector (calc-target-joint-dimension joint-list))) ;; [rad/s^2] or [m/s^2]
         (calc-torque-buffer-args (send self :calc-torque-buffer-args)))
   (unless (every #'null (send self :legs)) ;; for legged robot, set default external force and moment at the end-effectors
     (let ((weight-force (* (send self :weight) 1e-6 (elt *g-vec* 2)))) ;; weight[g] * 1e-6 * gvec[mm/s^2] = force[N]
       (unless target-coords
	 (dolist (limb '(:rleg :lleg))
	   (push (send self limb :end-coords) target-coords)))
       (unless (or force-list moment-list) ;; force [N], moment [Nm]
	 ;; set default force & moment by solving mimimum internal forces
	 (let ((ret-f
		(transform
		 (pseudo-inverse (send self :calc-grasp-matrix (send-all target-coords :worldpos)))
		 (float-vector 0 0 weight-force
			       (* 1e-3 (elt (send self :centroid nil) 1) weight-force)
                               (* -1e-3 (elt (send self :centroid nil) 0) weight-force) 0))))
	   (dotimes (i (length target-coords))
	     (push (subseq ret-f (* i 6) (+ (* i 6) 3)) force-list)
	     (push (subseq ret-f (+ (* i 6) 3) (+ (* i 6) 6)) moment-list)
	     ))
         (setq force-list (reverse force-list))
         (setq moment-list (reverse moment-list))
	 )))
   (unless (= (length force-list) (length moment-list)
	      (length target-coords))
     (warn ";; ERROR : list length differ : force-list ~A moment-list ~A target-coords ~A~%"
	   (length force-list) (length moment-list) (length target-coords))
     (return-from :torque-vector jvv))
   (mapcar #'(lambda (fv mv tc)
	       (send (send tc :parent) :ext-force fv)
	       ;; calc moment-offset caused by fv ;
	       ;; current irtdyna.l's requires moment around the origin
	       (let* ((moment-offset (v* (scale 1e-3 (send tc :worldpos)) fv)))
		 (send (send tc :parent) :ext-moment (v+ mv moment-offset))))
	   force-list moment-list target-coords)
   (send self :calc-torque
              :debug-view debug-view
              :jvv jvv :jav jav :calc-torque-buffer-args calc-torque-buffer-args)
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
          (additional-weight-list)
     &allow-other-keys)
    (with-append-root-joint ;; inverse-kinematics with base-link
     (link-list-with-robot-6dof self link-list
                                :joint-class 6dof-joint
                                :joint-args (list :min min :max max))
     (send* self :inverse-kinematics target-coords
            :move-target move-target :link-list link-list-with-robot-6dof
            :cog-gain cog-gain :centroid-thre centroid-thre :target-centroid-pos target-centroid-pos
            :additional-weight-list
            (append
             additional-weight-list
             (list (list (car (send self :links)) root-link-virtual-joint-weight)))
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
  (:print-vector-for-robot-limb
   (vec)
   (let ((tmp-limbs
          (remove-duplicates
           (mapcar #'(lambda (l)
                       (find-if #'(lambda (tmp-limb) (member l (send self tmp-limb :links))) '(:rarm :larm :rleg :lleg :torso :head)))
                   (send-all (send self :joint-list) :child-link))))
         (ordered-joint-list (send self :joint-list)))
     (format t "    #f(~%      ")
     (dolist (tmp-limb tmp-limbs)
       (dolist (j (send self tmp-limb :joint-list))
         (format t "~6,2f " (elt vec (position j ordered-joint-list))))
       (format t "~%      "))
     (format t ")~%")
     vec))
  ;; Calculate zmp[mm] from sensor local forces and moments
  ;;   If force_z is large, zmp can be defined and returns 3D zmp.
  ;;   Otherwise, zmp cannot be defined and returns nil.
  (:calc-zmp-from-forces-moments
   (forces moments ;; [N] [Nm], sensor local force & moment
    &key (wrt :world)
         ;; :wrt is :local => calc local zmp for robot's root-link coords
         ;; :wrt is :world => calc world zmp for robot
         (limbs '(:rleg :lleg))
         (force-sensors (mapcar #'(lambda (l) (send self :force-sensor l)) limbs))
         (zmp-z (elt (apply #'midpoint 0.5 (mapcar #'(lambda (l) (send self l :end-coords :worldpos)) limbs)) 2)) ;; [mm]
         (fz-thre 50)) ;; [N]
   (let ((tmpzmpx 0) (tmpzmpy 0) (tmpfz 0)
         (tmp-zmp-z (* 1e-3 zmp-z))) ;; [mm]->[m]
     (mapcar #'(lambda (fs f m)
                 (let ((fsp (scale 1e-3 (send fs :worldpos))) ;; [mm]->[m]
                       (nf (send fs :rotate-vector f))
                       (nm (send fs :rotate-vector m)))
                   (setq tmpzmpx (+ tmpzmpx
                                    (* (elt nf 2) (elt fsp 0))
                                    (- (* (- (elt fsp 2) tmp-zmp-z) (elt nf 0)))
                                    (- (elt nm 1)))
                         tmpzmpy (+ tmpzmpy
                                    (* (elt nf 2) (elt fsp 1))
                                    (- (* (- (elt fsp 2) tmp-zmp-z) (elt nf 1)))
                                    (elt nm 0))
                         tmpfz (+ tmpfz (elt nf 2)))))
             force-sensors forces moments)
     (let ((tmpzmp (scale 1e3 (float-vector (/ tmpzmpx tmpfz) (/ tmpzmpy tmpfz) tmp-zmp-z)))) ;; [m]->[mm]
       (cond
        ((< tmpfz fz-thre) nil)
        ((eq wrt :world) tmpzmp)
        ((eq wrt :local) (send (car (send self :links)) :inverse-transform-vector tmpzmp))
        (t )
        ))))
  )

;;; moved from rbrarin libraries
(defmethod robot-model
  ;; :foot-midcoords
  ;;   purpose : to calculate midcoords of :rleg and :lleg
  ;;   relevant euslib codes : defined in and copied from rbrain/basicape.l
  ;;   objection : in the following codes, leged robot is assumed.
  ;;               is this correct?
  (:foot-midcoords
   (&optional (mid 0.5))
   (apply #'midcoords mid (send self :legs :end-coords))
   )
  ;; :fix-leg-to-coords
  ;;   purpose : to fix robot's legs to a coords
  ;;   relevant euslib codes : defined in and copied from rbrain/basicmodel.l
  ;;   objection : in the following codes, leged robot is assumed.
  ;;               is this correct?
  (:fix-leg-to-coords
   (fix-coords &optional (l/r :both) &key (mid 0.5) &allow-other-keys) ;; support-leg
   (unless (not (some #'null (send self :legs :links)))
     (return-from :fix-leg-to-coords nil))
   (let (support-coords tmp-coords move-coords pos rot ra)
     (cond
      ((or (eq l/r :left) (eq l/r :lleg))
       (setq support-coords
             (send self :lleg :end-coords :copy-worldcoords)))
      ((or (eq l/r :right) (eq l/r :rleg))
       (setq support-coords
             (send self :rleg :end-coords :copy-worldcoords)))
      (t
       (setq support-coords
             (midcoords mid
                        (send self :lleg :end-coords :copy-worldcoords)
                        (send self :rleg :end-coords :copy-worldcoords)))))
     (setq tmp-coords (send fix-coords :copy-worldcoords))
     (setq move-coords (send support-coords :transformation self))
     (send tmp-coords :transform move-coords :local)
     (send self :newcoords tmp-coords)
     (send self :worldcoords)
     tmp-coords))
  (:move-centroid-on-foot
   (leg fix-limbs
        &rest args
        &key (thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) 1 5)) fix-limbs))
        (rthre (mapcar #'(lambda (x) (deg2rad (if (memq x '(:rleg :lleg)) 1 5))) fix-limbs))
        (mid 0.5)
        (target-centroid-pos
         (if (eq leg :both)
             (apply
              #'midpoint mid
              (mapcar
               #'(lambda (tmp-leg)
                   (send self tmp-leg :end-coords :worldpos))
               (remove-if-not #'(lambda (x) (memq x '(:rleg :lleg))) fix-limbs)))
           (send self leg :end-coords :worldpos)))
        (fix-limbs-target-coords
         (mapcar #'(lambda (x) (send self x :end-coords :copy-worldcoords)) fix-limbs))
        (root-link-virtual-joint-weight #f(0.1 0.1 0.0 0.0 0.0 0.5)) ;; use only translation x, y and rotation z
        &allow-other-keys)
   "(leg fix-limbs
        &rest args
        &key (thre (mapcar #'(lambda (x) (if (memq x '(:rleg :lleg)) 1 5)) fix-limbs))
        (rthre (mapcar #'(lambda (x) (deg2rad (if (memq x '(:rleg :lleg)) 1 5))) fix-limbs))
        (mid 0.5)
        (target-centroid-pos
         (if (eq leg :both)
             (apply
              #'midpoint mid
              (mapcar
               #'(lambda (tmp-leg)
                   (send self tmp-leg :end-coords :worldpos))
               (remove-if-not #'(lambda (x) (memq x '(:rleg :lleg))) fix-limbs)))
           (send self leg :end-coords :worldpos)))
        (fix-limbs-target-coords
         (mapcar #'(lambda (x) (send self x :end-coords :copy-worldcoords)) fix-limbs))
        &allow-other-keys)
    leg -> legs for target of robot's centroid. :both, :rleg, and :lleg are supported.
    fix-limbs -> limb names which are fixed in this IK."
   (with-move-target-link-list
    (mt ll self fix-limbs)
    (let* ()
      (send* self :fullbody-inverse-kinematics
             fix-limbs-target-coords
             :move-target mt :link-list ll
             :fix-limbs (remove-if-not #'(lambda (x) (memq x '(:rleg :lleg))) fix-limbs)
             :root-link-virtual-joint-weight root-link-virtual-joint-weight
             :thre thre :rthre rthre
             :target-centroid-pos target-centroid-pos
             args)
      )))
  (:calc-walk-pattern-from-footstep-list
   (footstep-list
    &key (default-step-height 50) (dt 0.1) (default-step-time 1.0)
         (default-zmp-offsets (list :rleg (float-vector 0 0 0) :lleg (float-vector 0 0 0)))
         (solve-angle-vector-args) (debug-view nil)
         (init-pose-function #'(lambda () (send self :move-centroid-on-foot :both '(:rleg :lleg)))))
   (let* ((res) (ret) (tm 0)
          (gg (instance gait-generator :init self dt)))
     (funcall init-pose-function)
     ;; initial move centroid on foot
     (send gg :initialize-gait-parameter
           footstep-list default-step-time (send (car (send self :links)) :get :c-til)
           :default-step-height default-step-height :default-double-support-ratio 0.2
           :default-zmp-offsets default-zmp-offsets)

     (while (null (setq ret (send gg :proc-one-tick :type :cycloid :debug t :solve-angle-vector-args solve-angle-vector-args))))
     (dotimes (i 2) (send self :calc-zmp)) ;; for zmp initialization

     ;; following are in control loop
     (while (setq ret (send gg :proc-one-tick :type :cycloid :debug t :solve-angle-vector-args solve-angle-vector-args))
       ;; only for debug view
       (let ((czmp (send self :calc-zmp
                         (send self :angle-vector)
                         (send (car (send self :links)) :copy-worldcoords)
                         :dt dt :pZMPz (elt (elt ret 5) 2))))
         (if debug-view
             (send self :draw-gg-debug-view
                   (elt ret 4) ;; swc
                   (elt ret 3) ;; spc
                   (elt ret 5) ;; rz
                   (elt ret 6) ;; cog
                   (elt ret 7) ;; pz
                   czmp dt))
         (push
          (list :angle-vector (send self :angle-vector)
                :root-coords (send self :copy-worldcoords)
                :czmp czmp :refzmp (elt ret 5) :cog (elt ret 6)
                :swing-leg-coords (elt ret 4)
                :support-leg-coords (elt ret 3)
                :time tm
                :pz (elt ret 7)
                )
          res)
         (setq tm (+ tm dt))
         ))
     (reverse res)
     ))
  (:draw-gg-debug-view
   (swc spc rz cog pz czmp dt)
   (send *viewer* :draw-objects :flush nil)
   (labels ((with-modify-color
             (col func)
             (let ((pc (send *viewer* :viewsurface :color)))
               (send *viewer* :viewsurface :color col)
               (funcall func)
               (send *viewer* :viewsurface :color pc))))
     (send-all spc :draw-on :flush nil :size 300 :color #f(1 0 0))
     (send-all swc :draw-on :flush nil :size 300 :color #f(0 1 0))
     (send rz :draw-on :flush nil :size 300 :color #f(0 0 1))
     (send czmp :draw-on :flush nil :size 200 :width 5)
     (with-modify-color
      #f(1 0 0)
      #'(lambda () (send *viewer* :viewsurface :string 20 20 "red = support leg")))
     (with-modify-color
      #f(0 1 0)
      #'(lambda () (send *viewer* :viewsurface :string 20 50 "green = swing leg")))
     (with-modify-color
      #f(0 0 1)
      #'(lambda () (send *viewer* :viewsurface :string 20 80 "blue = refzmp")))
     (with-modify-color
      #f(1 1 1)
      #'(lambda () (send *viewer* :viewsurface :string 20 110 "white = calc zmp"))))
   (send *viewer* :viewsurface :flush)
   (x::window-main-one)
   )
  ;; generate footstep parameter
  ;; currently only default, forward and outside
  (:gen-footstep-parameter
   (&key (ratio 1.0))
   (warn ";; generating footstep-parameter...~%")
   (let ((pav (send self :angle-vector))
         (pc (send self :copy-worldcoords)))
     (send self :reset-pose)
     (send self :fix-leg-to-coords (make-coords) '(:rleg :lleg))
     (let ((dol (abs (elt (apply #'v- (send self :legs :end-coords :worldpos)) 1))))
       (labels ((ik-test
                 (target-coords-func diff-func)
                 (send self :reset-pose)
                 (send self :fix-leg-to-coords (make-coords) '(:rleg :lleg))
                 (let* ((tc (send self :rleg :end-coords :copy-worldcoords))
                        (init-tc (send tc :copy-worldcoords)))
                   (while (send self :inverse-kinematics
                                (funcall target-coords-func tc)
                                :link-list (send self :link-list (send self :rleg :end-coords :parent))
                                :move-target (send self :rleg :end-coords) :warnp nil))
                   (funcall diff-func tc init-tc))))
         (let ((fol (ik-test #'(lambda (tc) (send tc :translate #f(10 0 0)))
                             #'(lambda (tc init-tc) (abs (elt (v- (send tc :worldpos) (send init-tc :worldpos)) 0)))))
               (ool (ik-test #'(lambda (tc) (send tc :translate #f(0 -10 0)))
                             #'(lambda (tc init-tc) (abs (elt (v- (send tc :worldpos) (send init-tc :worldpos)) 1)))))
               (frr (ik-test #'(lambda (tc) (send tc :rotate (deg2rad -2.5) :z))
                             #'(lambda (tc init-tc) (abs (rad2deg (- (caar (send tc :rpy-angle)) (caar (send init-tc :rpy-angle)))))))))
           (send self :newcoords pc)
           (send self :angle-vector pav)
           (warn ";; generating footstep-parameter... done.~%")
           (send self :put :footstep-parameter
                 (list :default-half-offset (float-vector 0 (* 0.5 dol) 0)
                       :forward-offset-length (* fol 0.5 ratio)
                       :outside-offset-length (* ool 0.5 ratio)
                       :rotate-rad (* frr 0.5 ratio)))
           )))
     ))
  (:footstep-parameter
   ()
   (unless (send self :get :footstep-parameter)
     (send self :gen-footstep-parameter))
   (send self :get :footstep-parameter)
   )
  ;; simple footstep gen
  (:go-pos-params->footstep-list
   (xx yy th ;; [mm] [mm] [deg]
    &key ((:footstep-parameter prm) (send self :footstep-parameter))
         ((:default-half-offset defp) (cadr (memq :default-half-offset prm)))
         ((:forward-offset-length xx-max) (cadr (memq :forward-offset-length prm)))
         ((:outside-offset-length yy-max) (cadr (memq :outside-offset-length prm)))
         ((:rotate-rad th-max) (abs (rad2deg (cadr (memq :rotate-rad prm)))))
         (gen-go-pos-step-node-func
          #'(lambda
              (mc leg leg-translate-pos)
              (let ((cc (send (send mc :copy-worldcoords) :translate
                              (cadr (assoc leg leg-translate-pos)))))
                (send cc :put :l/r leg)
                cc))))
   (let* ((x-sign (if (> xx 0.0) 1.0 -1.0))
          (y-sign (if (> yy 0.0) 1.0 -1.0))
          (th-sign (if (> th 0.0) 1.0 -1.0))
          (dx 0.0) (dy 0.0) (dth 0.0) (cnt 0)
          (leg (if (> yy 0.0) :lleg :rleg))
          (mc (apply #'midcoords 0.5 (send self :legs :end-coords :copy-worldcoords)))
          ;;(mc (make-coords))
          (leg-translate-pos
           (mapcar #'(lambda (l)
                       (list l (scale (case l (:rleg -1) (:lleg 1)) defp)))
                   '(:rleg :lleg)))
          (ret (list (funcall gen-go-pos-step-node-func mc (case leg (:lleg :rleg) (:rleg :lleg)) leg-translate-pos))))
     (labels ((do-push-steps
               (max-cnt func)
               (setq cnt 0)
               (dotimes (i max-cnt)
                 (funcall func)
                 (push (funcall gen-go-pos-step-node-func mc leg leg-translate-pos) ret)
                 (setq leg (case leg (:lleg :rleg) (:rleg :lleg)))
                 (incf cnt)
                 ))
              (calc-max-count
               (val val-max)
               (1+ (round (floor (- (/ (abs val) val-max) 1e-5))))))
       (do-push-steps (max (calc-max-count xx xx-max)
                           (calc-max-count yy yy-max)
                           (calc-max-count th th-max))
                      #'(lambda ()
                          (let ((ddx (if (> xx-max (abs (- dx xx))) (- xx dx) (* xx-max x-sign)))
                                (ddy (cond
                                      ((oddp cnt) 0.0)
                                      ((> yy-max (abs (- dy yy))) (- yy dy))
                                      (t (* yy-max y-sign))))
                                (ddth (cond
                                       ((> th-max (abs (- dth th))) (- th dth))
                                       (t (* th-max th-sign)))))
                            (send mc :translate (float-vector ddx ddy 0.0))
                            (send mc :rotate (deg2rad ddth) :z)
                            (setq dx (+ dx ddx) dy (+ dy ddy) dth (+ dth ddth)))))
       (push (funcall gen-go-pos-step-node-func mc leg leg-translate-pos) ret)
       (reverse ret))))
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
