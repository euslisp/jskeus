;; test code for unittest

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest test1
  )

(deftest test2
  (dotimes (i 10)
    (assert t "true"))
  )

(deftest test3
  (dotimes (i 10)
    (assert (null nil) "false"))
  )

(deftest test4
  (dotimes (i 10)
    (assert t "ok"))
  (warning-message 1 "*** This test  fails intentinally ****~%")
  (dotimes (i 10)
    (assert nil "false")
    ;; force remove trace from expected errored test
    (setq ((car (*unit-test* . result)) . failures) nil)
    )
  (dotimes (i 10)
    (assert t "ok"))
  )

(deftest test5
  (assert-eq (+ 1 1) 2)
  (assert-neq (+ 1 3) 2)
  (assert-equal '(:a :b :c) '(:a :b :c))
  (assert-equal "hoge" (copy-seq "hoge"))
  (assert-nequal '(:a :b :c) '(:a :b :c :d))
)

(deftest test6 ;; this test should fail at each evaluation
  (assert-eq (+ 1 3) 2) ;; should fail
  (assert-neq (+ 1 1) 2) ;; should fail
  (assert-eq '(:a :b :c) '(:a :b :c)) ;; should fail
  (assert-eq "hoge" (copy-seq "hoge")) ;; should fail
  (assert-equal '(:a :b :c) '(:a :b :c :d)) ;; should fail
  (assert-nequal '(:a :b :c) '(:a :b :c)) ;; should fail
  (setq ((car (*unit-test* . result)) . failures) nil)
)

(run-all-tests)

(setq ret -1)
(format t "functions = ~A~%" (length (send *unit-test* :functions)))
(send *unit-test* :result)
(if (and (= (length (send *unit-test* :functions)) 6)
         (= (apply #'+ (send-all (send *unit-test* :result) :num-tests)) 6)
         (= (apply #'+ (send-all (send *unit-test* :result) :num-failures)) 2)
         (= (apply #'+ (send-all (send *unit-test* :result) :num-successes)) 4))
    (setq ret 0))
(exit ret)

