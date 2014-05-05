(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(setq euslisp-dir (read-from-string (read-line (piped-fork "rospack" "find" "euslisp"))))

(load "models/irt-all-robots.l")
(unless x::*root* (defun make-all-robots ())) ;; do not create viwer

(setq robot-viewers (make-all-robots))
(deftest make-all-robots
  (let (fname (fname-list ""))
    (dolist (viewer robot-viewers)
      (send viewer :draw-objects)
      (setq fname (format nil "~A/build/irt_robots_~A.png" euslisp-dir (send viewer :name)))
      (setq fname-list (format nil "~A ~A" fname-list fname))
      (send viewer :viewer :viewsurface :write-to-image-file fname)
      )
    (unix::system (format nil "montage -geometry 100x100+1+1 ~A ~A/build/irt_all_robots.png"
			  fname-list euslisp-dir))
    ))

(load "models/irt-all-objects.l")
(unless x::*root* (defun make-all-objects ())) ;; do not create viwer

(setq object-viewers (make-all-objects))
(deftest make-all-objects
  (let (fname (fname-list ""))
    (dolist (viewer object-viewers)
      (send viewer :draw-objects)
      (setq fname (format nil "~A/build/irt_objects_~A.png" euslisp-dir (send viewer :name)))
      (setq fname-list (format nil "~A ~A" fname-list fname))
      (send viewer :viewer :viewsurface :write-to-image-file fname)
      )
    (unix::system (format nil "montage -geometry 100x100+1+1 ~A ~A/build/irt_all_objects.png"
			  fname-list euslisp-dir))
    ))

(run-all-tests)
(exit)
