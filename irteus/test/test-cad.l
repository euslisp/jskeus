;; test code for euscad

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; stl
(deftest read-write-stl
  (let* ((b0 (make-cube 100 100 100))
	 (tmpdir (format nil "/tmp/tmp~A" (unix::getpid)))
	 (tmpstl (format nil "~A/cube.stl" tmpdir)))
    (unix::mkdir tmpdir)
    (warning-message 2 "writing to ~A~%" tmpstl)
    (eus2stl tmpstl (body-to-faces b0))
    ;;
    (setq b1 (stl2eus tmpstl))

    (assert (eps-v= (v- (send (send b1 :box) :maxpoint) (send (send b1 :box) :minpoint)) #f(100 100 100) 3) "check cube size")
    ))

;; wrl
(deftest read-write-wrl
  (let* ((b0 (make-cube 100 100 100))
	 (tmpdir (format nil "/tmp/tmp~A" (unix::getpid)))
	 (tmpwrl (format nil "~A/cube.wrl" tmpdir)))
    (unix::mkdir tmpdir)
    (warning-message 2 "writing to ~A~%" tmpwrl)
    (eus2wrl tmpwrl b0)
    ))

;; with robot model
(load "models/h7-robot.l")
(deftest read-write-robot-stl
  (let* ((robot (h7))
	 (tmpdir (format nil "/tmp/tmp~A" (unix::getpid)))
	 (tmpstl (format nil "~A/robot.stl" tmpdir))
         b bb1 bb2)
    (unix::mkdir tmpdir)
    (eus2stl tmpstl robot :scale 0.001)
    ;;
    (warning-message 2 "writing to ~A~%" tmpstl)
    (setq b (stl2eus tmpstl :scale 1000.0))

    (setq bb1 (make-bounding-box (flatten (send-all (send robot :bodies) :vertices))))
    (setq bb2 (send b :box))

    (warning-message 2 "original bounding box ~A~%" (v- (send (send bb1 :box) :maxpoint) (send (send bb1 :box) :minpoint)))
    (warning-message 2 "     stl bounding box ~A~%" (v- (send (send bb2 :box) :maxpoint) (send (send bb2 :box) :minpoint)))
    (assert (eps-v= (v- (send (send bb1 :box) :maxpoint) (send (send bb1 :box) :minpoint))
                    (v- (send (send bb2 :box) :maxpoint) (send (send bb2 :box) :minpoint))
                    10))
    )
  )

(deftest read-write-robot-wrl
  (let* ((robot (h7))
	 (tmpdir (format nil "/tmp/tmp~A" (unix::getpid)))
	 (tmpwrl (format nil "~A/robot.wrl" tmpdir))
         b bb1 bb2)
    (unix::mkdir tmpdir)
    (warning-message 2 "writing to ~A~%" tmpwrl)
    (eus2wrl tmpwrl robot)
    )
  )

(run-all-tests)
(exit)
