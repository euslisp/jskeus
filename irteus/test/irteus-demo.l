(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(load "irteus/demo/demo.l")
(if (or (null x::*display*) (= x::*display* 0)) ;; aboid to show  (window-main-one nil) was called, but no display is found. message to reduce outputs in travis
    (defun x::window-main-one (&rest args) ))
(defmacro do-until-key (&rest forms) (with-gensyms (do-until-key-counter tm) `(progn (setq tm (instance mtimer :init)) (dotimes (do-until-key-counter 10) ,@forms))))

(deftest test-full-body-ik
  (full-body-ik :debug-view nil))

(deftest test-full-body-ik-use-torso
  (full-body-ik :use-torso nil :debug-view nil))

(deftest test-full-body-ik-use-leg
  (full-body-ik :use-leg t :debug-view nil))

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

(deftest test-walk-motion-single-support-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (walk-motion-single-support-for-sample-robot))))))

(deftest test-walk-motion-stair-climb-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (walk-motion-stair-climb-for-sample-robot))))))

(deftest test-trot-walk-motion-for-sample-robot-go-backward-over
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (trot-walk-motion-for-sample-robot))))))

(deftest test-trot-walk-motion-for-sample-robot
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (trot-walk-motion-for-sample-robot :go-backward-over nil))))))

(deftest test-trot-walk-motion-for-sample-robot-with-sphere-hand
  (assert
   (not (some #'null (mapcar #'(lambda (x) (cadr (memq :angle-vector x)))
                             (trot-walk-motion-for-sample-robot :go-backward-over nil :rotation-axis (list t t nil nil)))))))

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

(deftest test-test-virtual-joint-for-rotation-axis-all
  (makunbound '*robot*)
  (assert
   (test-virtual-joint-for-rotation-axis-all)))

(deftest test-look-at-ik
  (assert
   (look-at-ik-all)))

(eval-when (load eval)
  (run-all-tests)
  (exit))
