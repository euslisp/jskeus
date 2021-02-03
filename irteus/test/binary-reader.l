(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest binary-reader
  (let (sv iv lv fv f32v dv)
    ;; integer
    (setq sv #g((4) :short   "CgBkAOgDECc="))
    (assert (v= sv #i(10 100 1000 10000)))
    (setq iv #g((4) :integer "FAAAAMgAAADQBwAAIE4AAA==" ))
    (assert (v= iv #i(20 200 2000 20000)))
    (setq lv #g((4) :long    "HgAAAAAAAAAsAQAAAAAAALgLAAAAAAAAMHUAAAAAAAA="))
    (assert (v= lv #i(30 300 3000 30000)))
    ;; float
    (setq fv #g((4) :float "bxKDOgrXIzzNzMw9AACAPw=="))
    (assert (eps-v= fv #f(0.001 0.01 0.1 1.0)))
    (setq f32v #g((4) :float "bxKDOgrXIzzNzMw9AACAPw=="))
    (assert (eps-v= f32v #f(0.001 0.01 0.1 1.0)))
    (setq dv #g((4) :double "/Knx0k1iYD97FK5H4XqUP5qZmZmZmck/MzMzMzMz8z8="))
    (assert (eps-v= dv #f(0.002 0.02 0.2 1.2)))
    )
  (let (im fm fmm)
    (setq im #g((3 3) :integer "CgAAABQAAAAeAAAAZAAAAMgAAAAsAQAA6AMAANAHAAC4CwAA"))
    (assert (equal (array-dimensions im) (list 3 3)))
    (assert
     (v= (array-entity im) #i(10 20 30 100 200 300 1000 2000 3000)))
    (setq fm  #g((3 3) :float "bxKDOm8SAzumm0Q7CtcjPArXozyPwvU8zczMPc3MTD6amZk+"))
    (assert (equal (array-dimensions fm) (list 3 3)))
    (assert
     (eps-v= (array-entity fm) #f(0.001 0.002 0.003 0.01 0.02 0.03 0.1 0.2 0.3)))
    (setq fmm #g((2 2 2) :float "bxKDOm8SAzsK1yM8CtejPM3MzD3NzEw+zcyMP5qZmT8="))
    (assert (equal (array-dimensions fmm) (list 2 2 2)))
    (assert
     (eps-v= (array-entity fmm) #f(0.001 0.002 0.01 0.02 0.1 0.2 1.1 1.2)))
    )
  ;;(setq note #g((4) :notexistingtype "")) ;; error
  )

(run-all-tests)
(exit)
