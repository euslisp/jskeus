(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(load "irteus/demo/demo.l")
(defmacro do-until-key (&rest forms) (with-gensyms (do-until-key-counter tm) `(progn (setq tm (instance mtimer :init)) (dotimes (do-until-key-counter 10) (warning-message 2 ";; do-until-key ~7,3f ~A/~A~%" (send tm :stop) do-until-key-counter 10) ,@forms))))

(deftest test-full-body-ik
  (full-body-ik))

(deftest test-full-body-ik-use-torso
  (full-body-ik :use-torso nil))

(deftest test-full-body-ik-use-leg
  (full-body-ik :use-leg t))

(deftest test-dual-arm-ik
  (dual-arm-ik))

(deftest test-dual-manip-ik
  (dual-manip-ik))

(deftest test-crank-motion
  (crank-motion))

(deftest test-hand-grasp
  (hand-grasp))

(deftest test-hanoi-arm
  (hanoi-arm))

(deftest test-particle
  (particle))

;; check whether all ik are solved for walking examples
(deftest test-walk-motion-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (walk-motion-for-sample-robot))))))

(deftest test-footstep-generator
  (unless (boundp '*robot*) (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let ((start-mc (send *robot* :foot-midcoords))
        goal-mc trans
        (x 0) (y 200) (th 180))
    (send *robot* :calc-walk-pattern-from-footstep-list
          (send *robot* :go-pos-params->footstep-list x y th))
    (setq goal-mc (send *robot* :foot-midcoords))
    (setq trans (send start-mc :transformation goal-mc :local))
    (assert
     (and (eps= 0.0 (norm (v- (float-vector x y 0.0) (send trans :pos))))
          (eps= 0.0 (- th (rad2deg (elt (car (send trans :rpy-angle)) 0))))))))

(deftest test-trot-walk-motion-for-sample-robot-go-backward-over
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (trot-walk-motion-for-sample-robot))))))

(deftest test-trot-walk-motion-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (trot-walk-motion-for-sample-robot :go-backward-over nil))))))

(deftest test-crawl-walk-motion-for-sample-robot-go-backward-over
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (crawl-walk-motion-for-sample-robot))))))

(deftest test-crawl-walk-motion-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (crawl-walk-motion-for-sample-robot :go-backward-over nil))))))

(deftest test-walk-motion-for-robots
  (assert
   (every #'identity
          (mapcar #'(lambda (rs-list) (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x))) rs-list))))
                  (walk-motion-for-robots)))))

(deftest test-test-preview-control-0
  (assert
   (every #'identity (test-preview-control-0))))

(deftest test-test-extended-preview-control-0
  (assert
   (every #'identity (test-preview-control-0 :preview-controller-class extended-preview-controller))))

(deftest test-test-preview-control-1
  (assert
   (every #'identity (test-preview-control-1))))

(deftest test-test-extended-preview-control-1
  (assert
   (every #'identity (test-preview-control-1 :preview-controller-class extended-preview-controller))))

(deftest test-test-preview-control-2
  (assert
   (every #'identity (test-preview-control-2))))

(deftest test-test-extended-preview-control-2
  (assert
   (every #'identity (test-preview-control-2 :preview-controller-class extended-preview-controller))))

(deftest test-test-preview-control-3
  (assert
   (every #'identity (test-preview-control-3))))

(deftest test-test-extended-preview-control-3
  (assert
   (every #'identity (test-preview-control-3 :preview-controller-class extended-preview-controller))))

(deftest test-test-preview-control-dynamics-filter-for-sample-robot
  (assert
   (every #'identity (test-preview-control-dynamics-filter-for-sample-robot))))

(deftest test-test-extended-preview-control-dynamics-filter-for-sample-robot
  (assert
   (every #'identity (test-preview-control-dynamics-filter-for-sample-robot :preview-controller-class extended-preview-controller))))

(deftest test-test-sample-1dof-closed-link-robot
  (assert
   (every #'identity (test-sample-1dof-closed-link-robot))))

(deftest test-test-sample-2dof-closed-link-robot
  (assert
   (every #'identity (test-sample-2dof-closed-link-robot))))

(deftest test-test-interlocking-joint-arm
  (assert
   (test-interlocking-joint-arm)))

(deftest test-test-sample-legged-robot-with-interlocking-joints
  (assert
   (test-sample-legged-robot-with-interlocking-joints)))

(run-all-tests)
(exit)
