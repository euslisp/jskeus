(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(defmacro do-until-key (&rest forms) (with-gensyms (do-until-key-counter tm) `(progn (setq tm (instance mtimer :init)) (dotimes (do-until-key-counter 10) (warning-message 2 ";; do-until-key ~7,3f ~A/~A~%" (send tm :stop) do-until-key-counter 10) ,@forms))))
(deftest demo
  (funcall demo-func)
  )

(run-all-tests)
(exit)
