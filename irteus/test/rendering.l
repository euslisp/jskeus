;; rendering.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(unless (boundp '*irtviewer*)
  (setq *irtviewer* (make-irtviewer)))

(deftest transpose-image ()
  (setq *test-img-path* (format nil "~A/irteus/test/pr2.png" *eusdir*))
  (assert (probe-file *test-img-path*) "test image pr2.png is not found")
  (assert (setq *img* (read-image-file *test-img-path*)) "failed to read-image-file pr2.png")
  (assert (> (send (send *img* :monochromize) :brightest-pixel) 0.0)
          "nothing is drawn in image")
  (setq *src-img* (copy-object *img*))
  (gl::transpose-image-rows *img*)
  (assert (not (string= (send *src-img* :entity) (send *img* :entity)))
          "transpose-image-rows must change entity from original image")
  (gl::transpose-image-rows *img*)
  (assert (string= (send *src-img* :entity) (send *img* :entity))
          "image transposed twice must be the same as original image")
)

(deftest offscreen-rendering ()
  (load "irteus/demo/sample-robot-model.l")
  (setq *robot* (instance sample-robot :init))
  (setq *cam* (send *robot* :camera "left-camera"))
  (setq *obj* (make-cube 10 1000 1000 :pos #f(1500 30 350)))
  (send *obj* :set-color #f(0 0 1))
  (send *cam* :draw-objects (list *obj*))
  (setq *img* (send *cam* :image-viewer :viewsurface :getglimage :offscreen t))
  (assert (> (send (send *img* :blue) :average-pixel) 0.0)
          "offscreen rendering"))

(run-all-tests)
(exit)
