(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest rotation-matrix
  (let (rot axis angle rot-angle)
    (setq rot (make-coords :rpy #f(-1.220e-08 -5.195e-09 1.333e-09)))
    (setq axis (cadr (rotation-angle (send rot :rot))))
    (print axis *error-output*)
    (assert (null axis) "small rotatoin-angle")

    (setq rot (make-coords :rpy #f(-1.220e-08 -5.195e-09 -1.333e-09)))
    (setq axis (cadr (rotation-angle (send rot :rot))))
    (print axis *error-output*)
    (assert (null axis) "small rotatoin-angle")

    (setq angle 0.00141 axis #f(-0.684209 0.536652 0.493825))
    (setq rot (make-coords :axis axis :angle angle))
    (print (send rot :rot) *error-output*)
    (setq rot-angle (rotation-angle (send rot :rot)))
    (print (list rot-angle (list angle axis) (norm axis) (v. (cadr rot-angle) axis)) *error-output*)
    (assert (eps= (car rot-angle) angle) "check angle of rotation-angle")
    (assert (eps-v= (cadr rot-angle) axis) "check axis rotatoin-angle"))

    (dotimes (i 1000)
      (setq axis (normalize-vector (random-vector 1.0)))
      (setq angle (- (random 2pi) pi))
      (setq rot (make-coords :axis axis :angle angle))
      (setq rot-angle (rotation-angle (send rot :rot)))
      (print (list rot-angle (list angle axis)) *error-output*)
      (cond ((and (< angle 1e-4) (null rot-angle))
             (assert (null rot-angle) "check angle of rotation-angle"))
            ((> (v. (cadr rot-angle) axis) 0)
             (assert (eps= (car rot-angle) angle) "check angle of rotation-angle")
             (assert (eps-v= (cadr rot-angle) axis) "check axis rotatoin-angle"))
            (t
             (assert (eps= (car rot-angle) (- angle)) "check angle of rotation-angle")
             (assert (eps-v= (cadr rot-angle) (v- axis)) "check axis rotatoin-angle"))
        ))
    ))

(run-all-tests)
(exit)
