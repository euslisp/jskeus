(defun test-to-be-compiled
  ()
  (assert (eps= (1+ 5) 6))
  (assert (eps= (1+ 5.5) 6.5))
  )
