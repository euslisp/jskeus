(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest test-compile
  (let* ((test-dir "./irteus/test/")
         (source-file (format nil "~a/test-to-be-compiled.l" test-dir))
         (binary-file (format nil "~a/test-to-be-compiled.so" test-dir))
         )
    (comp:compile-file-if-src-newer source-file test-dir)
    (load binary-file)
    (test-to-be-compiled)
    ))

(run-all-tests)
(exit)
