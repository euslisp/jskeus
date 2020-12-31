(require :unittest "lib/llib/unittest.l")

;;(setq sys::*gc-hook* #'(lambda (a b) (format t "GC! free:~A  total:~A~%" a b)))

(init-unit-test)

(setq i-max 100000)

(deftest test-makecoords-quaternion
  (setq vmrss-orig (elt (unix::getrusage 0) 2))
  (dotimes (j 10)
    (dotimes (i i-max)
      (setq ret #f(0 0 0 1 0 0 0))
      (make-cascoords :pos (float-vector 0 0 0)
                      :rot
                      (user::quaternion2matrix (float-vector 1 0 0 0))))
    ;;
    (setq vmrss (elt (unix::getrusage 0) 2))
    (format *error-output* "~A gc:~A, vmrss:~A -> ~A~%" j (sys::gc) vmrss-orig vmrss)
    (assert (< vmrss (* 2 vmrss-orig)) "make-coords quaternion2matrix")))


(eval-when (load eval)
  (run-all-tests)
  (exit))
