(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest test-priority-queue
  (let (q e)
    (setq q (instance priority-queue :init))
    ;; push
    (dolist (i (list 1 5 3 2 0 4 2)) (send q :push i))
    (format t "priority-queue ~A, length ~A~%"
            (send q :get-val 'lisp::heap)
            (send q :length))
    (assert (= (send q :length) 7) "length of ~A is 7" (send q :get-val 'lisp::heap))
    ;; pop
    (dolist (i (list 0 1 2 2 3 4 5))
      (assert (not (send q :empty)) "queue is not empty")
      (setq e (send q :pop))
      (format t "queue ~A, popped item ~A~%"
              (send q :get-val 'lisp::heap) e)
      (assert (= e i) "dequeued item is ~A" e))
    (assert (send q :empty) "queue is empty")))

(run-all-tests)
(exit)
