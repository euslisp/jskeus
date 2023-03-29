;; test code for probe-file

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; irteusx errors when a file named 'NIL' exists https://github.com/euslisp/jskeus/pull/618
(deftest nil-file-with-irtx
  (let ()
    (unix::chdir "/tmp")
    (with-open-file (f "NIL" :direction :output)(format f "NIL"))
    (assert (load "irteus/irtx.l"))
    ))


(run-all-tests)
(exit)
