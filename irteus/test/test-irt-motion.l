;; test code for irteus/irtdyna.l, irtmodel.l, and irtrobot.l using robot model

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(unless (boundp '*robot*)
  (setq *robot* (instance sample-robot :init))
  (unless (or (null x::*display*) (= x::*display* 0))
    (objects (list *robot*))))

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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; unit tests
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftest test-irtdyna-samplerobot
  (assert (test-zmp-comomn *robot*))
  (assert (test-torque-from-its-own-weight-common *robot*))
  (assert (test-torque-from-ext-force-common *robot*)))

(run-all-tests)
(exit 0)
