;; test code for irteus/irtdyna.l, irtmodel.l, and irtrobot.l using robot model

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(load "irteus/demo/sample-arm-model.l")
(load "irteus/demo/sample-multidof-arm-model.l")
(unless (boundp '*sample-robot*)
  (setq *sample-robot* (instance sample-robot :init))
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list *sample-robot*))))
(unless (boundp '*sarm-robot*)
  (setq *sarm-robot* (instance sample-multidof-arm-robot :init)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; common test codes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; test zmp
(defun test-zmp-comomn
  (robot &key ((:viewer vw) (if (boundp '*irtviewer*) *irtviewer*)))
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list robot)))
  (every
   #'identity
   (mapcar #'(lambda (leg)
               (let ((zmp-cog-p))
                 (send robot :newcoords (make-coords))
                 (send robot :reset-pose)
                 (send robot :fix-leg-to-coords (make-coords) '(:rleg :lleg))
                 (send robot :calc-zmp)
                 (unless (or (null x::*display*) (= x::*display* 0))
                   (send vw :look-all))
                 (send robot :move-centroid-on-foot leg '(:rleg :lleg))
                 (unless (or (null x::*display*) (= x::*display* 0))
                   (send vw :look-all))
                 ;; if angle-vector and root-coords are updated, zmp does not equal to cog because inertia term exists
                 (push (not (eps= (norm (subseq (v- (send robot :centroid) (send robot :calc-zmp)) 0 2)) 0.0)) zmp-cog-p)
                 (send robot :calc-zmp)
                 ;; if angle-vector and root-coords are updated twice, zmp should equal to cog
                 (push (eps= (norm (subseq (v- (send robot :centroid) (send robot :calc-zmp)) 0 2)) 0.0) zmp-cog-p)
                 (every #'identity zmp-cog-p)))
           '(:rleg :lleg :both))))

(defun init-pose-torque-tests
  (robot)
  (mapcar #'(lambda (jnt)
              (send jnt :joint-angle
                    (min (max 0.0 (send jnt :min-angle)) (send jnt :max-angle))))
          (send robot :joint-list)))

(defun max-angle-with-inf-check-torque-tests
  (jnt)
  (if (and (eq *inf* (send jnt :max-angle)) (derivedp jnt rotational-joint)) 90 (send jnt :max-angle)))

(defun min-angle-with-inf-check-torque-tests
  (jnt)
  (if (and (eq *-inf* (send jnt :min-angle)) (derivedp jnt rotational-joint)) -90 (send jnt :min-angle)))

;; check torque comparing toroques from links weights with torques from :torque-vector
;; however, currently robots start torque check from initial-pose (all joints are 0) so that yaw joints are not checked.
(defun test-torque-from-its-own-weight-common
  (robot &key ((:viewer vw) (if (boundp '*irtviewer*) *irtviewer*)) (debug-view))
  (init-pose-torque-tests robot)
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list robot)))
  (labels ((check-torque-for-one-joint
            (jnt ja)
            (init-pose-torque-tests robot)
            (send jnt :joint-angle ja)
            (unless (or (null x::*display*) (= x::*display* 0))
              (send vw :draw-objects))
            (send robot :weight) ;; for calculating c-til and m-til
            (let* ((weight-force (scale (* 0.001 (send (send jnt :child-link) :get :m-til))
                                        (scale -0.001 *g-vec*)))
                   (torque-from-link-weight
                    (v.
                     ;; axis to convert link-weight-moment -> joint torque
                     (normalize-vector (send (send (send jnt :parent-link :copy-worldcoords)
                                                   :transform (jnt . default-coords))
                                             :rotate-vector
                                             (case (jnt . axis)
                                               (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
                                               (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
                                               (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
                                               (t (jnt . axis)))))
                     ;; moment caused by link weight
                     (if (derivedp jnt rotational-joint)
                         (v*
                          (scale 0.001 (v- (send (send jnt :child-link) :get :c-til) (send jnt :child-link :worldpos)))
                          weight-force)
                       weight-force)
                     ))
                   (torque-from-method
                    (elt (send robot :torque-vector
                               :force-list (list (float-vector 0 0 0))
                               :moment-list (list (float-vector 0 0 0))
                               :target-coords (list (send robot :head :end-coords)))
                         (if (find-method robot :actuators) (send jnt :servo :no) (position jnt (send robot :joint-list)))))
                   (torque-diff (+ torque-from-link-weight torque-from-method)))
              (unless (eps= torque-diff 0.0)
                (if debug-view
                    (format t ";; diff ~7,3f[Nm] is too large!! <- torque(weight) ~7,3f [Nm] - torque(method) ~7,3f[Nm] (~A)~%"
                            torque-from-link-weight torque-from-method torque-diff (send jnt :name))))
              (eps= torque-diff 0.0))))
    (every #'identity
           (mapcar #'(lambda (jnt)
                       (every #'identity
                              (mapcar #'(lambda (ang) (check-torque-for-one-joint jnt ang))
                                      (if (> (max-angle-with-inf-check-torque-tests jnt) 0 (min-angle-with-inf-check-torque-tests jnt))
                                          (list (max-angle-with-inf-check-torque-tests jnt) 0 (min-angle-with-inf-check-torque-tests jnt))
                                        (list (max-angle-with-inf-check-torque-tests jnt) (min-angle-with-inf-check-torque-tests jnt)))
                                      )))
                   (send robot :joint-list)))
    ))

;; check torque comparing toroques from ext-force with torques from :torque-vector
;; however, currently robots start torque check from initial-pose (all joints are 0) so that yaw joints are not checked.
(defun test-torque-from-ext-force-common
  (robot &key ((:viewer vw) (if (boundp '*irtviewer*) *irtviewer*)) (debug-view))
  (init-pose-torque-tests robot)
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list robot)))
  (labels ((check-torque-for-one-joint
            (jnt ja)
            (init-pose-torque-tests robot)
            (send jnt :joint-angle ja)
            (unless (or (null x::*display*) (= x::*display* 0))
              (send vw :draw-objects))
            (send robot :weight) ;; for calculating c-til and m-til
            (let* ((ax ;; axis to convert link-weight-moment -> joint torque
                    (normalize-vector (send (send (send jnt :parent-link :copy-worldcoords)
                                                  :transform (jnt . default-coords))
                                            :rotate-vector
                                            (case (jnt . axis)
                                              (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
                                              (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
                                              (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
                                              (t (jnt . axis))))))
                   (mt (make-cascoords
                        :coords
                        (send (send jnt :child-link :copy-worldcoords)
                              :translate (scale 100 (v* #f(0 0 1) ax)) :world)))
                   (ff #f(0 0 -50))
                   (diff-torque-from-force
                    (v. ax
                        ;; moment caused by link weight
                        (if (derivedp jnt rotational-joint)
                            (scale -1 (v* (scale 1e-3 (v- (send mt :worldpos) (send jnt :child-link :worldpos))) ff))
                          (scale -1 ff)))))
              (send (send jnt :child-link) :assoc mt)
              (let* ((idx (if (find-method robot :actuators) (send jnt :servo :no) (position jnt (send robot :joint-list))))
                     (diff-torque-from-method
                      (- (elt (send robot :torque-vector :force-list (list ff) :moment-list (list (float-vector 0 0 0)) :target-coords (list mt)) idx)
                         (elt (send robot :torque-vector :force-list (list (float-vector 0 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send robot :head :end-coords))) idx)))
                     (torque-diff (- diff-torque-from-force diff-torque-from-method)))
                (unless (eps= torque-diff 0.0)
                  (if debug-view
                      (format t ";; diff ~7,3f[Nm] is too large!! <- torque(weight) ~7,3f [Nm] - torque(method) ~7,3f[Nm] (~A, ~A)~%"
                              torque-diff diff-torque-from-force diff-torque-from-method (send jnt :name) (send jnt :joint-angle))))
                (send (send jnt :child-link) :dissoc mt)
                (eps= torque-diff 0.0)))))
    (every #'identity
           (mapcar #'(lambda (jnt)
                       (every #'identity
                              (mapcar #'(lambda (ang) (check-torque-for-one-joint jnt ang))
                                      (if (> (max-angle-with-inf-check-torque-tests jnt) 0 (min-angle-with-inf-check-torque-tests jnt))
                                          (list (max-angle-with-inf-check-torque-tests jnt) 0 (min-angle-with-inf-check-torque-tests jnt))
                                        (list (max-angle-with-inf-check-torque-tests jnt) (min-angle-with-inf-check-torque-tests jnt)))
                                      )))
                   (send robot :joint-list)))
    ))

;; sample mobile robot
(defclass sample-mobile-robot
  :super robot-model
  :slots (wheel-l wheel-r end-coords))

(defmethod sample-mobile-robot
  (:init 
   (&rest args)
   (let (c body-base body-wheel-l body-wheel-r)
     (send-super* :init args)
     (setq body-base (make-cylinder 50 150))
     (send body-base :locate #f(150 0 200))
     (setq body-base (body+ (make-cylinder 200 200) body-base))
     (send body-base :locate #f(0 0 100))
     (send body-base :set-color :seagreen)
     (setq body-base (instance bodyset-link :init (make-cascoords)
			       :bodies (list body-base) :name :body))

     (setq body-wheel-l (instance bodyset-link :init (make-cascoords)
				  :bodies (list (make-cylinder 140 30))
				  :name :wheel-l))
     (send body-wheel-l :rotate -pi/2 :x)
     (send body-wheel-l :locate #f(0 -140 -200))
     (send-all (send body-wheel-l :bodies) :set-color :red)

     (setq body-wheel-r (instance bodyset-link :init (make-cascoords)
				  :bodies (list (make-cylinder 140 30))
				  :name :wheel-r))
     (send body-wheel-r :rotate -pi/2 :x)
     (send body-wheel-r :locate #f(0 -140 200))
     (send-all (send body-wheel-r :bodies) :set-color :red)

     (send self :assoc body-base)

     (send body-base :assoc (setq end-coords (make-cascoords :pos #f(10 0 0))))
     (send body-base :assoc body-wheel-l)
     (send body-base :assoc body-wheel-r)
     
     (setq wheel-l (instance rotational-joint :init :name :wheel-l
			     :child-link body-wheel-l :parent-link body-base
			     :axis :z
			     :min *-inf* :max *inf*))
     (setq wheel-r (instance rotational-joint :init :name :wheel-r
			     :child-link body-wheel-r :parent-link body-base
			     :axis :z
			     :min *-inf* :max *inf*))
     
     (setq links (list body-base body-wheel-l body-wheel-r))
     (setq joint-list (list wheel-l wheel-r))
     (send self :init-ending)
     self))
  (:wheel-l (&rest args) (forward-message-to wheel-l args))
  (:wheel-r (&rest args) (forward-message-to wheel-r args))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  (:reset-pose ())
  )

;; check for wheel-joint and omniwheel-joint
(defun test-joint-for-mobile-robot-common
  (&rest args
	 &key (robot-class) (wheel-class) (use-wholebody) (debug-view :no-message)
	 &allow-other-keys)
  (setq *ground* (make-cube 3000 3000 10))
  (send *ground* :set-color :skyblue)
  (setq *robot* (instance robot-class :init))
  (send *robot* :reset-pose)
  (if (eq robot-class sarmclass)
      (send *robot* :angle-vector #f(0.0 0.0 90.0 0.0 90.0 90.0 0.0 0.0)))
  (send *robot* :newcoords (make-coords))
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list *robot* *ground*)))
  (let ((robot-link-list
         (if use-wholebody
             (send *robot* :link-list (send *robot* :end-coords :parent)))))
    (with-append-root-joint
     (link-list-with-virtual-link *robot* robot-link-list :joint-class wheel-class)
     (send* *robot* :inverse-kinematics
            (make-cascoords :pos #f(1500 -1500 0) :rpy (float-vector pi 0 0))
            :link-list (car link-list-with-virtual-link)
            :move-target (send *robot* :end-coords)
            :debug-view debug-view
            args))
    ))

;; sample robot for sphere-joint and 6dof-joint check
(defclass sample-multidof-joint-robot
  :super cascaded-link
  :slots (end-coords))

(defmethod sample-multidof-joint-robot
  (:init
   (&rest args &key (joint-class sphere-joint) (child-init-coords (make-coords)) &allow-other-keys)
   (send-super* :init args)
   (let* ((b1 (make-gdome (make-icosahedron 30)))
	  (b2 (body+ (make-cylinder 10 100)
		     (send (make-cube 20 5 10) :locate #f(15 0 95))))
	  (l1 (instance bodyset-link :init (make-cascoords) :bodies (list b1) :name :root))
	  (l2 (instance bodyset-link :init (make-cascoords) :bodies (list b2) :name :bar)))
     (send l2 :newcoords child-init-coords)
     (let ((j (instance joint-class :init
			:parent-link l1 :child-link l2
                        :max-joint-velocity (float-vector *inf* *inf* *inf* *inf* *inf* *inf*))))
       (setq links (list l1 l2))
       (setq joint-list (list j))
       (setq end-coords (make-cascoords :coords
					(send (send l2 :copy-worldcoords) :translate #f(0 0 100))))
       (send self :assoc l1)
       (send l1 :assoc l2)
       (send l2 :assoc end-coords)
       (send self :init-ending)
       self)))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  (:joint (&rest args) (forward-message-to (car joint-list) args))
  )

;; check computation of orientation of 6dof-joint and sphere-joint
;; check diff between end-coords after FK and IK
(defun test-orientation-of-joint-class-common
  (joint-class)
  (setq *robot* (instance sample-multidof-joint-robot :init :joint-class joint-class
		      :child-init-coords (make-coords :pos #f(100 200 300) :rpy (list (deg2rad 10) (deg2rad 20) (deg2rad 30)))))
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list *robot* (send *robot* :end-coords))))
  (let ((div 100.0) (max-dif-pos 0.0) (max-dif-rot 0.0) (range 90)
	(ja (send *robot* :joint :joint-angle))
	(ec (send *robot* :end-coords :copy-worldcoords))
	(cc (send *robot* :joint :child-link :copy-worldcoords)))
    (dotimes (i 600)
      (let ((prev-ec ec) (prev-ja ja) (prev-cc cc)
	    (dd (* range (sin (/ i div)))))
	;; calc next joint-angle
	(setq ja (let ((tmp-ja (instantiate float-vector (send *robot* :joint :joint-dof))))
		   (dotimes (i (send *robot* :joint :joint-dof))
		     (setf (elt tmp-ja i) dd))
		   tmp-ja))

       (send *robot* :joint :joint-angle ja)
       (setq ec (send *robot* :end-coords :copy-worldcoords))
       (setq cc (send *robot* :joint :child-link :copy-worldcoords))
       (send *robot* :joint :joint-angle prev-ja)
       (with-difference-position-and-rotation
	(dp dr (send *robot* :end-coords) ec)
	(send *robot* :inverse-kinematics-loop dp dr
	      :link-list (send *robot* :link-list (send *robot* :end-coords :parent))
	      :union-link-list (send *robot* :link-list (send *robot* :end-coords :parent))
	      :move-target (send *robot* :end-coords)
              :avoid-nspace-gain 0.0 :avoid-weight-gain 0.0
	      ;; ease limitation of inverse-kinematics-loop
	      :thre 0 :rthre 0 :p-limit 1000 :r-limit 1000
              ))
       (setq calc-ec (send *robot* :end-coords :copy-worldcoords))
       (if (> (norm (send ec :difference-position calc-ec)) max-dif-pos)
	   (setq max-dif-pos (norm (send ec :difference-position calc-ec))))
       (if (> (norm (send ec :difference-rotation calc-ec)) max-dif-rot)
	   (setq max-dif-rot (norm (send ec :difference-rotation calc-ec))))

       (unless (or (null x::*display*) (= x::*display* 0))
         (send *irtviewer* :draw-objects :flush nil)
         (send ec :draw-on :flush nil :size 400)
         (send calc-ec :draw-on :flush nil :color #f(1 0 0))
         (send *irtviewer* :viewer :flush))
       ))
    (list max-dif-pos max-dif-rot)))

;; test :cog-convergence-check
(defun test-cog-convergence-check-common
  (robot)
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords))
  (and (send robot :move-centroid-on-foot :both '(:rleg :lleg) :centroid-thre 10)
       (send robot :move-centroid-on-foot :both '(:rleg :lleg) :centroid-thre #'(lambda (diff) (> 10 (norm diff))))
       (send robot :move-centroid-on-foot :both '(:rleg :lleg) :centroid-thre #f(10 10)))
  )

;; test ik fail log
(defun test-load-ik-fail-log-common
  (robot ik-function)
  (labels ((find-log-files
            (log-directory)
            (remove-if-not #'(lambda (x) (substringp (format nil "~A-" (send (class robot) :name)) x)) (directory log-directory))))
    (let* ((log-directory (format nil "/tmp/irtmodel-ik-~A" (unix::getpid)))
           (prev-log-files (find-log-files log-directory)))
      (funcall ik-function robot)
      (let ((new-log-file ;; find latest log
             (car (remove-if #'(lambda (x) (member x prev-log-files :test #'string=)) (find-log-files log-directory)))))
        ;; ik log execution
        (progn (load (format nil "~A/~A" log-directory new-log-file))(ik-setup)(null-output (ik-check)))
        t))))

(defun test-load-ik-fail-log-rarm-ik-common
  (robot)
  (test-load-ik-fail-log-common
   robot
   #'(lambda (robot)
       (send robot :newcoords (make-coords))
       (send robot :rarm :move-end-pos #f(10000 0 0) :world :debug-view :no-message)))
  )

(defun test-load-ik-fail-log-assoc-ik-common
  (robot)
  (test-load-ik-fail-log-common
   robot
   #'(lambda (robot)
       (send robot :newcoords (make-coords))
       (send robot :reset-pose)
       (let ((b (make-cylinder 10 250)))
         (send b :move-coords (send robot :larm :end-coords) (send b :worldcoords))
         (unless (or (null x::*display*) (= x::*display* 0))
           (objects (list robot b)))
         (let ((c (make-cascoords :pos (send b :transform-vector (float-vector 0 0 250)))))
           (send b :assoc c)
           (send (send robot :larm :end-coords :parent) :assoc b)
           (send robot :larm :move-end-pos #f(10000 0 0) :world :rotation-axis nil :move-target c :debug-view :no-message)
           (send (send robot :larm :end-coords :parent) :dissoc b)
           (send b :dissoc c)
           )))
   ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; unit tests
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftest test-irtdyna-samplerobot
  (assert (test-zmp-comomn *sample-robot*))
  (assert (test-torque-from-its-own-weight-common *sample-robot*))
  (assert (test-torque-from-ext-force-common *sample-robot*)))

(deftest test-cog-convergence-check
  (assert (test-cog-convergence-check-common *sample-robot*)))

(deftest test-joint-for-mobile-robot
  (assert
   (test-joint-for-mobile-robot-common
    :robot-class sample-mobile-robot :wheel-class omniwheel-joint))
  (assert
   (test-joint-for-mobile-robot-common
    :robot-class sarmclass :wheel-class omniwheel-joint
    :translation-axis :z))
  (assert
   (test-joint-for-mobile-robot-common
    :robot-class sarmclass :wheel-class omniwheel-joint
    :use-wholebody t)))

(deftest test-orientation-of-joint-class
  (assert
   (let ((ret (test-orientation-of-joint-class-common sphere-joint)))
     (and
      (> 0.01 (elt ret 0))
      (> (deg2rad 1e-2) (elt ret 1)))))
  (assert
   (let ((ret (test-orientation-of-joint-class-common 6dof-joint)))
     (and
      (> 0.05 (elt ret 0))
      (> (deg2rad 1e-2) (elt ret 1))))))

(deftest test-load-ik-fail-log-rarm-ik-for-sample-robot
  (assert (test-load-ik-fail-log-rarm-ik-common *sample-robot*)))

(deftest test-load-ik-fail-log-rarm-ik-for-sample-multidof-joint-robot
  (assert (test-load-ik-fail-log-rarm-ik-common *sarm-robot*)))

(deftest test-load-ik-fail-log-assoc-ik-for-sample-robot
  (assert (test-load-ik-fail-log-assoc-ik-common *sample-robot*)))

(run-all-tests)
(exit 0)
