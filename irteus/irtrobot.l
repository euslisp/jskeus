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
	  larm rarm lleg rleg torso head
          ;; sensor slots
          force-sensors imu-sensors cameras
          ;;
          support-polygons
          ))

(defmethod robot-model
  (:init-ending
   ()
   (prog1
       (send-super :init-ending)
     (if (and (not (every #'null (send self :legs))) ;; for legged robot
              (not (every #'null (send self :legs :end-coords)))) ;; for robot with legs end-coords
       (send self :make-support-polygons))
     (setq end-coords-list (remove nil (append end-coords-list
                                               (list larm-end-coords rarm-end-coords
                                                     lleg-end-coords rleg-end-coords
                                                     head-end-coords torso-end-coords))))
     )
   )
  ;; End-coords accessor
  (:rarm-end-coords () rarm-end-coords)
  (:larm-end-coords () larm-end-coords)
  (:rleg-end-coords () rleg-end-coords)
  (:lleg-end-coords () lleg-end-coords)
  (:head-end-coords () head-end-coords)
  (:torso-end-coords () torso-end-coords)
  ;; Root-link accessor
  (:rarm-root-link () rarm-root-link)
  (:larm-root-link () larm-root-link)
  (:rleg-root-link () rleg-root-link)
  (:lleg-root-link () lleg-root-link)
  (:head-root-link () head-root-link)
  (:torso-root-link () torso-root-link)
  (:limb
   (limb method &rest args)
   (let (ret)
     (case method
       (:end-coords 
	(user::forward-message-to 
         (send self (read-from-string (format nil "~A-END-COORDS" limb)))
	 args))
       (:root-link
	(user::forward-message-to 
         (send self (read-from-string (format nil "~A-ROOT-LINK" limb)))
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
       (:cameras (send self :get-sensors-method-by-limb :cameras limb))
       (:imu-sensors (send self :get-sensors-method-by-limb :imu-sensors limb))
       (:force-sensors (send self :get-sensors-method-by-limb :force-sensors limb))
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
                (v- (car args) (send move-target :worldpos))))
              target-orient move-orient axis-orient axis-orient2 head-weight head-nspace jid)
         ;; this code assuming that the robot does not have continuous rotation joint for neck-y
         (setq p-dif-rot dif-rot
               dif-pos (send move-target :difference-position target-coords :translation-axis nil)
               dif-rot (send move-target :difference-rotation target-coords :rotation-axis :z))
         (setq head-weight (instantiate float-vector (send self :calc-target-joint-dimension link-list))
               head-nspace (instantiate float-vector (send self :calc-target-joint-dimension link-list)))
         (fill head-weight 1)
         (setq target-orient (matrix-column (send target-coords :worldrot) 2)
               move-orient (matrix-column (send move-target :worldrot) 2))
         (setq jid 0)
         (dolist (j (send-all link-list :joint))
           (setq axis-orient (send j :parent-link :rotate-vector (case (j . axis) (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1)) (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1)) (t (j . axis)))))
           ;; direciton of non continuous area
           (setq axis-orient2 (send j :parent-link :rotate-vector #f(-1 0 0)))
           (when  (or (and  (< (v.* axis-orient move-orient target-orient) 0.0)
                            (< (v.* axis-orient move-orient axis-orient2) 0.0)
                            (< (v.* axis-orient axis-orient2 target-orient) 0.0))
                      (and  (> (v.* axis-orient move-orient target-orient) 0.0)
                            (> (v.* axis-orient move-orient axis-orient2) 0.0)
                            (> (v.* axis-orient axis-orient2 target-orient) 0.0)))

             (if (> (v.* axis-orient move-orient target-orient) 0)
                 (setf (elt head-nspace jid) -1)
               (setf (elt head-nspace jid) 1))
             (fill head-weight 0.01)
             (setf (elt head-weight jid) 0.0)
             (return nil))
           (incf jid))
         (send self :inverse-kinematics-loop dif-pos dif-rot
               :move-target move-target
               :collision-avoidance-link-pair nil
               :rotation-axis :z
               :translation-axis nil
               :link-list link-list
               :loop count :stop stop ;;:thre nil
               :target-coords target-coords ;; for debug-view
               :null-space head-nspace
               :weight head-weight
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
	 (concatenate
          float-vector
          (send-all (send self :gripper limb :joint-list) :joint-angle))
       (map float-vector #'(lambda (x y) (send x :joint-angle y))
	    (send self :gripper limb :joint-list) (cadr args))))))
  ;; sensor accessor definitions
  (:camera (sensor-name) "Returns camera with given name" (send self :get-sensor-method :camera sensor-name))
  (:force-sensor (sensor-name) "Returns force sensor with given name" (send self :get-sensor-method :force-sensor sensor-name))
  (:imu-sensor (sensor-name) "Returns imu sensor of given name" (send self :get-sensor-method :imu-sensor sensor-name))
  (:get-sensor-method
   (sensor-type sensor-name)
   (let ((sens (send self (read-from-string (format nil "~As" sensor-type)))))
     (find sensor-name sens :test #'equal :key #'(lambda (x) (send x :name)))))
  (:get-sensors-method-by-limb
   (sensors-type limb)
   (remove-if-not #'(lambda (x) (member (send x :parent) (all-child-links (send self limb :root-link))))
                  (send self sensors-type)))
  ;; sensor accessors
  (:force-sensors () "Returns force sensors." force-sensors)
  (:imu-sensors () "Returns imu sensors." imu-sensors)
  (:cameras () "Returns camera sensors." cameras)
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
   "look at hand position, l/r supports :rarm, :larm, :arms, and '(:rarm :larm)"
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
   "solve inverse kinematics, move move-target to target-coords
    look-at-target suppots t, nil, float-vector, coords, list of float-vector, list of coords
    link-list is set by default based on move-target -> root link link-list"
   (if (or (atom target-coords) (functionp target-coords)) (setq target-coords (list target-coords)))
   (when (null (every #'(lambda (x) (or (coordinates-p x) (functionp x))) target-coords))
     (warn ";; error: all target-coords should be Coordinates or functions, but get ~A~%" target-coords)
     (return-from :inverse-kinematics nil))
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
   "move move-target using dif-pos and dif-rot,
    look-at-target suppots t, nil, float-vector, coords, list of float-vector, list of coords
    link-list is set by default based on move-target -> root link link-list"
   (if (or (atom target-coords) (functionp target-coords)) (setq target-coords (list target-coords)))
   (prog1
       (send-super* :inverse-kinematics-loop dif-pos dif-rot
                    :link-list link-list :move-target move-target
                    :target-coords target-coords args)
     (if debug-view (send self :look-at-target look-at-target :target-coords target-coords))
     ))
  ;; look-at-target supports -> t, nil, float-vector, coords, list of float-vector, list of coords
  (:look-at-target
   (look-at-target &key (target-coords))
   "move robot head to look at targets, look-at-target support t/nil float-vector coordinates, center of list of float-vector or list of coordinates"
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
  (:init-pose () "Set robot to initial posture." (send self :angle-vector (instantiate float-vector (send self :calc-target-joint-dimension (cdr (send self :links))))))
  (:torque-vector
   (&key (force-list) (moment-list) (target-coords)
         (debug-view nil)
         (calc-statics-p t)
         (dt 0.005)
         (av (send self :angle-vector))
         (root-coords (send (car (send self :links)) :copy-worldcoords))
         (calc-torque-buffer-args (send self :calc-torque-buffer-args))
         (distribute-total-wrench-to-torque-method (if (and (not (every #'null (send self :legs))) ;; For legged robots
                                                            (not (and force-list moment-list target-coords))) ;; If no force-list, moment-list, target-coords are specified
                                                       :distribute-total-wrench-to-torque-method-default)))
   "Returns torque vector"
   (let ((ret-tq
          (send self :calc-torque
                :debug-view debug-view
                :calc-statics-p calc-statics-p
                :av av :root-coords root-coords :dt dt
                :force-list force-list :moment-list moment-list :target-coords target-coords
                :calc-torque-buffer-args calc-torque-buffer-args)))
     (when (find-method self distribute-total-wrench-to-torque-method)
       (let ((ext-wrench-tq (send self distribute-total-wrench-to-torque-method)))
         (dotimes (i (length ret-tq))
           (setf (elt ret-tq i) (- (elt ret-tq i) (elt ext-wrench-tq i)))
           (send (elt joint-list i) :joint-torque (elt ret-tq i)))))
     ret-tq
     ))
  (:distribute-total-wrench-to-torque-method-default
   ()
   ;; set default force & moment by solving mimimum internal forces
   (let ((target-coords))
     (dolist (limb '(:rleg :lleg))
       (if (send self limb)
           (push (send self limb :end-coords) target-coords)))
     (let* ((ret-fm (send self :calc-contact-wrenches-from-total-wrench (send-all target-coords :worldpos)
                          :total-wrench (concatenate float-vector (send (car (send self :links)) :force) (send (car (send self :links)) :moment)))))
       (send self :calc-torque-from-ext-wrenches :force-list (car ret-fm) :moment-list (cadr ret-fm) :target-coords target-coords)
       )))
  (:calc-force-from-joint-torque
   (limb all-torque &key (move-target (send self limb :end-coords)) (use-torso))
   "Calculates end-effector force and moment from joint torques."
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
          (cog-translation-axis :z) ;; translation-axis of cog
          (centroid-offset-func nil) ;; offset function for cog
          (centroid-thre 5.0) ;; cog convergence threshould [mm]
          (additional-weight-list)
          (joint-args nil)
          (cog-null-space nil)
          (min-loop 2) ;; 2 is minimum loop count
     &allow-other-keys)
    "fullbody inverse kinematics for legged robot.
     necessary args : target-coords, move-target, and link-list must include legs' (or leg's) parameters
                      ex. (send *robot* :fullbody-inverse-kinematics (list rarm-tc rleg-tc lleg-tc) :move-target (list rarm-mt rleg-mt lleg-mt) :link-list (list rarm-ll rleg-ll lleg-ll))"
    (with-append-root-joint ;; inverse-kinematics with root-link
     (link-list-with-robot-6dof self link-list
                                :joint-class 6dof-joint
                                :joint-args (append (list :min min :max max) joint-args))
     (send* self :inverse-kinematics target-coords
            :move-target move-target :link-list link-list-with-robot-6dof
            :cog-gain cog-gain :centroid-thre centroid-thre :target-centroid-pos target-centroid-pos
            :cog-translation-axis cog-translation-axis :centroid-offset-func centroid-offset-func :cog-null-space cog-null-space
            :min-loop min-loop
            :additional-weight-list
            (append
             additional-weight-list
             (list (list (car (send self :links)) root-link-virtual-joint-weight)))
            args)
     ))
  (:joint-angle-limit-nspace-for-6dof
   (&key (avoid-nspace-gain 0.01) (limbs '(:rleg :lleg)))
   (let* ((ll (mapcar #'(lambda (l) (send self l :links)) limbs))
          (J (send self :calc-jacobian-from-link-list
                   (mapcar #'(lambda (l) (append (list (car (send self :links))) l)) ll)
                   :move-target (mapcar #'(lambda (l) (send self l :end-coords)) limbs)
                   :translation-axis (make-list (length limbs) :initial-element t)
                   :rotation-axis (make-list (length limbs) :initial-element t)))
          (Jb (make-matrix (array-dimension J 0) 6))
          (Jj (make-matrix (array-dimension J 0) (send self :calc-target-joint-dimension ll))))
     (dotimes (i (array-dimension Jb 0))
       (dotimes (ii (array-dimension Jb 1))
         (setf (aref Jb i ii) (aref J i ii))))
     (dotimes (i (array-dimension Jj 0))
       (dotimes (ii (array-dimension Jj 1))
         (setf (aref Jj i ii) (aref J i (+ 6 ii)))))
     (let ((dthb
            (transform (m* (send self :calc-inverse-jacobian Jb) Jj)
                       (scale (* -1 avoid-nspace-gain)
                              (joint-angle-limit-nspace
                               (send-all (send self :calc-union-link-list ll) :joint))))))
       (setf (elt dthb 0) 0)
       (setf (elt dthb 1) 0)
       dthb
       )))
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
   "Print angle vector with limb alingment and limb indent.
    For example, if robot is rarm, larm, and torso, print result is:
    #f(
       rarm-j0 ... rarm-jN
       larm-j0 ... larm-jN
       torso-j0 ... torso-jN
       )"
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
  (:calc-zmp-from-forces-moments
   (forces moments ;; [N] [Nm], sensor local force & moment
    &key (wrt :world)
         ;; :wrt is :local => calc local zmp for robot's root-link coords
         ;; :wrt is :world => calc world zmp for robot
         (limbs
          (if (send self :force-sensors)
              (remove nil (mapcar #'(lambda (fs)
                                      (find-if #'(lambda (l) (equal fs (car (send self l :force-sensors)))) '(:rleg :lleg)))
                                  (send self :force-sensors)))
            '(:rleg :lleg)))
         (force-sensors (mapcar #'(lambda (l) (send self :force-sensor l)) limbs))
         (cop-coords (mapcar #'(lambda (l) (send self l :end-coords)) limbs))
         (fz-thre 1e-3) ;; [N]
         (limb-cop-fz-list
          (mapcar #'(lambda (fs f m cc)
                      (let ((fsp (scale 1e-3 (send fs :worldpos))) ;; [mm]->[m]
                            (nf (send fs :rotate-vector f))
                            (nm (send fs :rotate-vector m)))
                        (send self :calc-cop-from-force-moment
                              nf nm fs cc :fz-thre fz-thre :return-all-values t)))
                  force-sensors forces moments cop-coords)))
   "Calculate zmp[mm] from sensor local forces and moments
       If force_z is large, zmp can be defined and returns 3D zmp.
       Otherwise, zmp cannot be defined and returns nil."
   (let* ((limb-cop-fz-list2 (remove nil limb-cop-fz-list))
          (limb-fz (reduce #'+ (mapcar #'(lambda (x) (cadr (memq :fz x))) limb-cop-fz-list2) :initial-value 0)))
     (if (< limb-fz fz-thre)
         nil
       (let ((tmpzmp (scale (/ 1.0 limb-fz) (reduce #'v+ (mapcar #'(lambda (x) (scale (cadr (memq :fz x)) (cadr (memq :cop x)))) limb-cop-fz-list2)))))
         (cond
          ((eq wrt :world) tmpzmp)
          ((eq wrt :local) (send (car (send self :links)) :inverse-transform-vector tmpzmp))
          (t )
          )))
     ))
  )

;;; moved from rbrarin libraries
(defmethod robot-model
  (:foot-midcoords
   (&optional (mid 0.5))
   "Calculate midcoords of :rleg and :lleg end-coords.
    In the following codes, leged robot is assumed."
   (apply #'midcoords mid (send self :legs :end-coords))
   )
  (:fix-leg-to-coords
   (fix-coords &optional (l/r :both) &key (mid 0.5) &allow-other-keys) ;; support-leg
   "Fix robot's legs to a coords
    In the following codes, leged robot is assumed."
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
   "Move robot COG to change centroid-on-foot location,
    leg : legs for target of robot's centroid, which should be :both, :rleg, and :lleg.
    fix-limbs : limb names which are fixed in this IK."
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
         (solve-angle-vector-args) (debug-view nil)
         ((:all-limbs al)
          (if (send self :force-sensors)
              (remove nil (mapcar #'(lambda (fs)
                                      (find-if #'(lambda (l) (equal fs (car (send self l :force-sensors)))) '(:rleg :lleg)))
                                  (send self :force-sensors)))
            '(:rleg :lleg)))
         ((:default-zmp-offsets dzo)
          (mapcan #'(lambda (x) (list x (float-vector 0 0 0))) al)) ;; [mm]
         (init-pose-function #'(lambda () (send self :move-centroid-on-foot :both '(:rleg :lleg))))
         (start-with-double-support t)
         (end-with-double-support t)
         (ik-thre 1) (ik-rthre (deg2rad 1))
         (calc-zmp t))
   "Calculate walking pattern from foot step list and return pattern list as a list of angle-vector, root-coords, time, and so on."
   (let* ((res) (ret) (tm 0)
          (gg (instance gait-generator :init self dt)))
     (funcall init-pose-function)
     ;; initial move centroid on foot
     (send gg :initialize-gait-parameter
           footstep-list default-step-time (send self :centroid)
           :default-step-height default-step-height :default-double-support-ratio 0.2
           :default-zmp-offsets dzo :all-limbs al
           :thre ik-thre :rthre ik-rthre
           :start-with-double-support start-with-double-support :end-with-double-support end-with-double-support)

     (while (null (setq ret (send gg :proc-one-tick :type :cycloid :debug t :solve-angle-vector-args solve-angle-vector-args))))
     (if calc-zmp (dotimes (i 2) (send self :calc-zmp))) ;; for zmp initialization

     ;; following are in control loop
     (while (setq ret (send gg :proc-one-tick :type :cycloid :debug t :solve-angle-vector-args solve-angle-vector-args))
       ;; only for debug view
       (let ((czmp (if calc-zmp
                       (send self :calc-zmp
                             (send self :angle-vector)
                             (send (car (send self :links)) :copy-worldcoords)
                             :dt dt :pZMPz (elt (elt ret 5) 2))))
             (end-coords-list (mapcar #'(lambda (x) (send self x :end-coords :copy-worldcoords)) (gg . all-limbs)))
             (contact-state (elt ret 8))
             )
         (when debug-view
             (send self :draw-gg-debug-view
                   end-coords-list
                   contact-state
                   (elt ret 5) ;; rz
                   (elt ret 6) ;; cog
                   (elt ret 7) ;; pz
                   czmp dt)
             (x::window-main-one))
         (push
          (list :angle-vector (car ret)
                :root-coords (cadr ret)
                :czmp czmp :zmp (elt ret 5) :cog (elt ret 6)
                :time tm
                :pz (elt ret 7)
                :all-limbs al
                :contact-state contact-state
                :end-coords-list end-coords-list
                )
          res)
         (setq tm (+ tm dt))
         ))
     (reverse res)
     ))
  (:draw-gg-debug-view
   (end-coords-list contact-state rz cog pz czmp dt)
   (send *viewer* :draw-objects :flush nil)
   (labels ((with-modify-color
             (col func)
             (let ((pc (send *viewer* :viewsurface :color)))
               (send *viewer* :viewsurface :color col)
               (funcall func)
               (send *viewer* :viewsurface :color pc))))
     (mapcar #'(lambda (ec cs)
                 (send ec :draw-on :flush nil :size 300 :color
                       (if (eq :swing cs) #f(0 1 0) #f(1 0 0))))
             end-coords-list contact-state)
     (send rz :draw-on :flush nil :size 300 :color #f(0 0 1))
     (if czmp (send czmp :draw-on :flush nil :size 200 :width 5))
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
   )
  ;; generate footstep parameter
  ;; currently only default, forward and outside
  (:gen-footstep-parameter
   (&key (ratio 1.0))
   "Generate footstep parameter"
   (warn ";; generating footstep-parameter...~%")
   (let ((pav (send self :angle-vector))
         (pc (send self :copy-worldcoords)))
     (send self :reset-pose)
     (send self :fix-leg-to-coords (make-coords) '(:rleg :lleg))
     (let ((dol (abs (elt (apply #'v- (send self :legs :end-coords :worldpos)) 1))))
       (labels ((ik-test
                 (target-coords-func diff-func &optional (limit-func))
                 (send self :reset-pose)
                 (send self :fix-leg-to-coords (make-coords) '(:rleg :lleg))
                 (let* ((tc (send self :rleg :end-coords :copy-worldcoords))
                        (init-tc (send tc :copy-worldcoords)))
                   (while (and
                           (send self :inverse-kinematics
                                 (funcall target-coords-func tc)
                                 :link-list (send self :link-list (send self :rleg :end-coords :parent))
                                 :move-target (send self :rleg :end-coords) :warnp nil)
                           (if limit-func (funcall limit-func tc init-tc) t)
                           ))
                   (funcall diff-func tc init-tc))))
         (let ((fol (ik-test #'(lambda (tc) (send tc :translate #f(10 0 0)))
                             #'(lambda (tc init-tc) (abs (elt (v- (send tc :worldpos) (send init-tc :worldpos)) 0)))))
               (ool (ik-test #'(lambda (tc) (send tc :translate #f(0 -10 0)))
                             #'(lambda (tc init-tc) (abs (elt (v- (send tc :worldpos) (send init-tc :worldpos)) 1)))))
               (frr (ik-test #'(lambda (tc) (send tc :rotate (deg2rad -2.5) :z))
                             #'(lambda (tc init-tc) (abs (rad2deg (- (caar (send tc :rpy-angle)) (caar (send init-tc :rpy-angle))))))
                             #'(lambda (tc init-tc) (> 90 (abs (rad2deg (- (caar (send tc :rpy-angle)) (caar (send init-tc :rpy-angle)))))))
                             )))
           (send self :newcoords pc)
           (send self :angle-vector pav)
           (warn ";; generating footstep-parameter... done.~%")
           (send self :put :footstep-parameter
                 (list :default-half-offset (float-vector 0 (* 0.5 dol) 0)
                       :forward-offset-length (* fol 0.5 ratio)
                       :outside-offset-length (* ool 0.5 ratio)
                       :rotate-rad (deg2rad (* frr 0.5 ratio))))
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
                (send cc :name leg)
                cc))))
   "Calculate foot step list from goal x position [mm], goal y position [mm], and goal yaw orientation [deg]."
   (let* ((x-sign (if (> xx 0.0) 1.0 -1.0))
          (y-sign (if (> yy 0.0) 1.0 -1.0))
          (th-sign (if (> th 0.0) 1.0 -1.0))
          (dx 0.0) (dy 0.0) (dth 0.0) (cnt 0)
          (leg (if (eps= (float yy) 0.0)
                   (if (> th 0.0) :lleg :rleg)
                 (if (> yy 0.0) :lleg :rleg)))
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
                           (- (* (calc-max-count yy yy-max) 2) 1)
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
  (:go-pos-quadruped-params->footstep-list
   (xx yy th ;; [mm] [mm] [deg]
       &key (type :crawl))
   "Calculate foot step list for quadruped walking from goal x position [mm], goal y position [mm], and goal yaw orientation [deg]."
   (let ((biped-fsl (send self :go-pos-params->footstep-list xx yy th :forward-offset-length 100 :rotate-rad 3))
         quadruped-fsl)
     ;; calculate foot step list for quad walk
     (dolist (fs biped-fsl)
       (let* ((fs-leg-name (send fs :name))
              (offset (send (send self fs-leg-name :end-coords :copy-worldcoords) :transformation fs :world))
              (target-arm-name))
         (case type
               (:crawl
                (dolist (l (list fs-leg-name (case fs-leg-name (:lleg :larm) (:rleg :rarm))))
                  (push (list (make-coords :coords (send (send self l :end-coords :copy-worldcoords) :transform offset :world) :name l))
                        quadruped-fsl)))
               (:trot
                (push (mapcar #'(lambda (l)
                                  (make-coords :coords (send (send self l :end-coords :copy-worldcoords) :transform offset :world) :name l))
                              (list fs-leg-name (setq target-arm-name (case fs-leg-name (:lleg :rarm) (:rleg :larm)))))
                      quadruped-fsl))
               (:pace
                (push (mapcar #'(lambda (l)
                                  (make-coords :coords (send (send self l :end-coords :copy-worldcoords) :transform offset :world) :name l))
                              (list fs-leg-name (case fs-leg-name (:lleg :larm) (:rleg :rarm))))
                      quadruped-fsl))
               (:gallop
                (push (mapcar #'(lambda (l)
                                  (make-coords :coords (send (send self l :end-coords :copy-worldcoords) :transform offset :world) :name l))
                              (list (case fs-leg-name (:rleg :rleg) (:lleg :rarm))
                                    (case fs-leg-name (:rleg :lleg) (:lleg :larm))))
                      quadruped-fsl))
               )))
     (reverse quadruped-fsl)))

  ;; Support polygon methods
  (:support-polygons
   ()
   "Return support polygons."
   (send-all (send-all support-polygons :body) :worldcoords)
   support-polygons)
  (:support-polygon
   (name)
   "Return support polygon.
    If name is list, return convex hull of all polygons.
    Otherwise, return polygon with given name"
   (if (consp name)
       (instance polygon :init :vertices (quickhull
                                          (apply
                                           #'append
                                           (mapcar #'(lambda (nm)
                                                       (send (send self :support-polygon nm) :vertices))
                                                   name))))
     (find name (send self :support-polygons) :test #'equal :key #'(lambda (x) (send x :name)))))
  (:make-support-polygons
   ()
   (setq support-polygons
         (remove nil (list
          (send self :make-sole-polygon :rleg)
          (send self :make-sole-polygon :lleg))))
   )
  ;; TODO
  ;;  1. Sole polygon change according to toe joint usage is not supported.
  ;;  2. Sole body is bounding box. This should be convex hull (https://github.com/euslisp/EusLisp/issues/17)
  (:make-sole-polygon
   (name)
   (labels ((all-descendant-links
             (l)
             (if (and l (send l :descendants))
                 (append (list l) (flatten (mapcar #'all-descendant-links (remove-if-not #'(lambda (x) (derivedp x bodyset-link)) (send l :descendants)))))
               )))
     (unless (send self name :links) (return-from :make-sole-polygon nil))
     (send-all (all-descendant-links (car (send self name :links))) :worldcoords)
     (let* ((target-nm (read-from-string (format nil "~A-sole-body" name)))
            (vs (remove-duplicates
                 (flatten (send-all (flatten (send-all (all-descendant-links (car (send self name :links))) :bodies)) :vertices))
                 :test #'(lambda (x y) (eps-v= x y *epsilon*))))
            (min-vs (find-extream vs #'(lambda (x) (elt x 2)) #'<))
            (b (send
                (make-bounding-box
                 (remove-if
                  #'(lambda (p)
                      (< 5.0 (- (elt p 2) (elt min-vs 2)))) vs)) ;; 5.0 is margin
                :body)))
       (send (send self name :end-coords :parent) :assoc b)
       (send self :put target-nm b)
       (send b :worldcoords)
       (let ((f (find-if #'(lambda (x) (memq :bottom (send x :id))) (send b :faces))))
         (send f :name name)
         f))))
  (:make-default-linear-link-joint-between-attach-coords
   (attach-coords-0 attach-coords-1 end-coords-name linear-joint-name)
   "Make default linear arctuator module such as muscle and cylinder and append lins and joint-list.
    Module includes parent-link => (j0) => l0 => (j1) => l1 (linear actuator) => (j2) => l2 => end-coords.
    attach-coords-0 is root side coords which linear actulator is attached to.
    attach-coords-1 is end side coords which linear actulator is attached to.
    end-coords-name is the name of end-coords.
    linear-joint-name is the name of linear actuator."
   (let ((l0) (l1) (l2) (j0) (j1) (j2)
         (linear-link-len (distance (send attach-coords-1 :worldpos) (send attach-coords-0 :worldpos))))
     ;; Make links
     (setq l0 (instance bodyset-link :init (make-cascoords)
                        :bodies (list (let ((b0 (make-cylinder 5 (* 0.5 linear-link-len))))
                                        (send b0 :set-color :red)
                                        b0))))
     (send l0 :newcoords (send attach-coords-0 :copy-worldcoords))
     (orient-coords-to-axis l0 (normalize-vector (v- (send attach-coords-1 :worldpos) (send attach-coords-0 :worldpos))))

     (setq l1 (instance bodyset-link :init (make-cascoords)
                        :bodies (list (let ((b0 (make-cylinder 5 (* 0.5 linear-link-len))))
                                        (send b0 :set-color :red)
                                        (send b0 :translate (float-vector 0 0 (* -0.5 linear-link-len)))
                                        b0))))
     (send l1 :newcoords (send attach-coords-1 :copy-worldcoords))
     (orient-coords-to-axis l1 (normalize-vector (v- (send attach-coords-1 :worldpos) (send attach-coords-0 :worldpos))))

     (setq l2 (instance bodyset-link :init (make-cascoords)
                        :bodies (list (let ((b0 (make-cube 5 5 5))) (send b0 :set-color :red) b0))))
     (send l2 :newcoords (send attach-coords-1 :copy-worldcoords))

     ;; Make end-coords
     (push (make-cascoords :coords (send attach-coords-1 :copy-worldcoords) :name end-coords-name)
           end-coords-list)

     ;; Assoc links
     (send (send attach-coords-0 :parent) :assoc l0)
     (send l0 :assoc l1)
     (send l1 :assoc l2)
     (send l2 :assoc (car end-coords-list))
     (send l1 :translate (float-vector 0 0 (* -1 linear-link-len))) ;; move to length = 0 position

     ;; Make joint
     (setq j0 (instance sphere-joint :init
                        :parent-link (send l0 :parent) :child-link l0
                        :min (float-vector -360 -360 -360)
                        :max (float-vector 360 360 360)))
     (setq j1 (instance linear-joint :init
                        :parent-link (send l1 :parent) :child-link l1
                        :name linear-joint-name
                        :axis :z :min 0 :max 1000))
     (setq j2 (instance sphere-joint :init
                        :parent-link (send l2 :parent) :child-link l2
                        :min (float-vector -360 -360 -360)
                        :max (float-vector 360 360 360)))
     (send j1 :joint-angle linear-link-len) ;; set linear joint angle to default length

     ;; Append joint-list and links
     (setq links (append links (list l0 l1 l2)))
     (setq joint-list (append joint-list (list j0 j1 j2)))
     ))
  (:calc-static-balance-point
   (&key (target-points
          (mapcar #'(lambda (tmp-arm)
                      (send (send self tmp-arm :end-coords) :worldpos))
                  '(:rarm :larm)))
         (force-list (make-list (length target-points) :initial-element (float-vector 0 0 0)))
         (moment-list (make-list (length target-points) :initial-element (float-vector 0 0 0)))
         (static-balance-point-height (elt (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)) 2))
         (update-mass-properties t))
   "Calculate static balance point which is equivalent to static extended ZMP.
    The output is expressed by the world coordinates.
    target-points are end-effector points on which force-list and moment-list apply.
    force-list [N] and moment-list [Nm] are list of force and moment at target-points.
    static-balance-point-height is height of static balance point [mm]."
   (let* ((sbp (float-vector 0 0 static-balance-point-height))
          (mg (* 1e-6 (send self :weight update-mass-properties) (elt *g-vec* 2))))
     (dotimes (idx 2)
       (let ((denom mg)
             (nume (* mg (elt (send self :centroid nil) idx))))
         (mapcar #'(lambda (f m tp)
                     (setq nume
                           (+ nume
                              (- (* (- (elt tp 2) static-balance-point-height) (elt f idx))
                                 (* (elt tp idx) (elt f 2)))
                              (case idx (0 (- (elt m 1))) (1 (elt m 0))))
                           denom (- denom (elt f 2))))
                 force-list moment-list target-points)
         (setf (elt sbp idx) (/ nume denom))
         ))
     sbp))
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
