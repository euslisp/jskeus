;; test code for robot usage

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "irteus/demo/sample-robot-model.l")
(unless (boundp '*robot*)
  (setq *robot* (instance sample-robot :init)))

(deftest test-properties
  ;; accessor of links
  (assert (send *robot* :links) (format nil "links accessor"))
  ;; accessor of joint-list
  (assert (send *robot* :joint-list) (format nil "joint-list accessor"))
  ;; accessor of link
  (dolist (l (send *robot* :links))
    (assert (send *robot* :link (send l :name)) (format nil "link accessor (~18A)" (send l :name))))
  ;; accessor of joint
  (dolist (j (send *robot* :joint-list))
    (assert (send *robot* :joint (send j :name)) (format nil "joint accessor (~18A)" (send j :name))))
  ;; accessor of end-coords
  (dolist (l '(:rarm :larm :rleg :lleg :head))
    (assert (send *robot* l :end-coords) (format nil "end-coords accessor (~8A)" l)))
  ;; accessor of mass properties
  (dolist (l (send *robot* :links))
    (assert (and (numberp (send l :weight)) (vectorp (l . acentroid)) (arrayp (send l :inertia-tensor)))
            (format nil "link mass properties accessor (~18A)" (send l :name))))
  ;; accessor of joint
  (dolist (j (send *robot* :joint-list))
    (assert (and (send j :min-angle) (send j :max-angle) (send j :max-joint-velocity) (send j :max-joint-torque))
            (format nil "joint properties accessor (~18A)" (send j :name))))
  )

(run-all-tests)
(exit)
