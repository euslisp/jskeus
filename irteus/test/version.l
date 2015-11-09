;; version.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest test-version-string-p ()
  (assert (version-string-p (car lisp-implementation-version))
          "(version-string-p (car lisp-implementation-version))")
  (assert (version-string-p "0") "(version-string-p \"0\")")
  (assert (version-string-p "1") "(version-string-p \"1\")")
  (assert (version-string-p "10000") "(version-string-p \"10000\")")
  (assert (version-string-p "1.0") "(version-string-p \"1.0\")")
  (assert (version-string-p "1.0.1") "(version-string-p \"1.0.1\")")
  (assert (version-string-p "0.0.0") "(version-string-p \"0.0.0\")")
  (assert (version-string-p "0.0.1") "(version-string-p \"0.0.1\")")
  (assert (version-string-p "0.1.1") "(version-string-p \"0.1.1\")")
  (assert (not (version-string-p ".")) "(not (version-string-p \".\")")
  (assert (not (version-string-p "-")) "(not (version-string-p \"-\")")
  (assert (not (version-string-p "-1")) "(not (version-string-p \"-1\")")
  (assert (not (version-string-p "1.-1")) "(not (version-string-p \"1.-1\")")
  (assert (not (version-string-p "1-1")) "(not (version-string-p \"1-1\")")
  (assert (not (version-string-p "foo")) "(not (version-string-p \"foo\")")
  (assert (not (version-string-p "")) "(not (version-string-p \"\")")
  (assert (not (version-string-p "Alpha")) "(not (version-string-p \"Alpha\")")
  (assert (not (version-string-p " ")) "(not (version-string-p \" \")"))

(deftest test-version-string-to-list ()
  (assert (equal (list 9 12) (version-string-to-list "9.12"))
          "(equal (list 9 12) (version-string-to-list \"9.12\"))"))

(deftest test-version-compare ()
  (assert (version< "9.12" "9.13") "(version< \"9.12\" \"9.13\")")
  (assert (not (version< "9.12" "9.12")) "(not (version< \"9.12\" \"9.12\"))")
  (assert (version<= "9.12" "9.13") "(version<= \"9.12\" \"9.13\")")
  (assert (version= "9.12" "9.12") "(version= \"9.12\" \"9.12\")")
  (assert (not (version= "9" "10")) "(not (version= \"9\" \"10\"))")
  (assert (version= "9.12" "9.12.0") "(version= \"9.12\" \"9.12.0\")")
  (assert (version= "9.12" "9.12.0.0.0") "(version= \"9.12\" \"9.12.0.0.0\")")
  (assert (not (version= "9.12" "9.12.0.0.1")) "(not (version= \"9.12\" \"9.12.0.0.0\"))")
  (assert (version> "9.12" "9.11.0.0.0") "(version> \"9.12\" \"9.11.0.0.0\")")
  (assert (not (version> "9.12" "9.12")) "(not (version> \"9.12\" \"9.12\"))"))

(run-all-tests)

(provide :version) ;; end of version.l
