(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(defun triangulation-concave ()
  (let* ((aface (instance face :init :vertices
                          (list (float-vector  15    0 0)
                                (float-vector  15  120 0)
                                (float-vector -15  120 0)
                                (float-vector -15    0 0)
                                (float-vector -120   0 0)
                                (float-vector -120 -30 0)
                                (float-vector  120 -30 0)
                                (float-vector  120  0  0))))
         (faces (geo::face-to-triangle aface))
         lines
         )
    ;; check result of triangle generation
    (assert faces)

    ;; make-vertical lines
    (setq lines
          (mapcar #'(lambda (p) (make-line (coerce (append p (list -10)) float-vector)
                                           (coerce (append p (list  10)) float-vector)))
                  (list (list   40  40)
                        (list  -40  40)
                        (list  -40 -40)
                        (list   40 -40)
                        (list    0 -40)
                        )))

    ;; intersection between face and line should return the same result after triangulation
    (dolist (l lines)
      (assert
       (equal (null (send aface :intersect-line (send l :pvertex) (send l :nvertex)))
              (null (some #'(lambda (f) (send f :intersect-line (send l :pvertex) (send l :nvertex))) faces))))
      )
    nil
    ))

(defun triangulation-with-multi-hole (param-id)
  (let ((plate (make-cube 1000 1000 5))
        (params ;; params for holes (list (list segments radius center) .. )
         (case param-id
           (0 (list
               (list  9 87.841726125643674550 (float-vector -250.0 -250.0 0.0))
               (list  6 36.824839027036261996 (float-vector -250.0  250.0 0.0))
               (list  9 52.020745915260789616 (float-vector  250.0 -250.0 0.0))
               (list 10 51.644968476707617810 (float-vector  250.0  250.0 0.0))
               (list  6 63.093857512806295063 (float-vector    0.0    0.0 0.0))
               ))
           (1 (list
               (list  9 87.8 (float-vector -250.0 -250.0 0.0))
               (list  6 36.8 (float-vector -250.0  250.0 0.0))
               (list  9 52.0 (float-vector  250.0 -250.0 0.0))
               (list 10 51.6 (float-vector  250.0  250.0 0.0))
               (list  6 63.0 (float-vector    0.0    0.0 0.0))
               ))))
        aface faces lines)

    ;; make plate with holes
    (dolist (p params)
      (let ((cyl (make-cylinder (cadr p) 20 :segments (car p))))
        (send cyl :translate  (caddr p))
        (send cyl :translate  (float-vector 0 0 -10))
        (setq plate (body- plate cyl))
        ))

    (setq aface (find-if #'(lambda (x) (eq :top (car (send x :id)))) (send plate :faces)))

    (setq faces (geo::face-to-triangle aface))

    ;; make-vertical lines
    (setq lines
          (mapcar #'(lambda (p) (make-line (coerce (append p (list -10)) float-vector)
                                           (coerce (append p (list  10)) float-vector)))
                  (list (list    0  250)
                        (list    0 -250)
                        (list  250    0)
                        (list -250    0)
                        (list    0    0)
                        )))
    ;; intersection between face and line should return the same result after triangulation
    (dolist (l lines)
      (assert
       (equal (null (send aface :intersect-line (send l :pvertex) (send l :nvertex)))
              (null (some #'(lambda (f) (send f :intersect-line (send l :pvertex) (send l :nvertex))) faces))))
      )
    nil
    ))

;; geo::face-to-triangle-fast (irtgeo.l) is a drop-in replacement for
;; geo::face-to-triangle-aux that triangulates via libGLU's sweep-line
;; tessellator when it's linked in (eusgl/irteusgl only -- see
;; *glu-tess-available* in irtgeo.l), falling back to the original
;; ear-clipping path otherwise. It's now used by pqp.l's
;; :make-pqpmodel and irtgl.l's make-glvertices-from-faces, both of
;; which previously called face-to-triangle-aux directly.
;;
;; The tests below check that face-to-triangle-fast produces results
;; consistent with the original path (same triangle count, same total
;; area, same line-intersection behavior) on the same holed faces
;; `triangulation-with-multi-hole` above already exercises, then
;; benchmark both paths against a face with more holes to show the
;; actual real-world benefit of using GLU here, not just of adding the
;; collector primitives in isolation.

;; NB: named tri-bench-now, not now -- lib/llib/unittest.l's own
;; run-test relies on a global `now` returning a timer object with an
;; :elapsed method, which a same-named plain-float helper here would
;; silently break.
(defun tri-bench-now ()
  (let ((tv (unix::gettimeofday))) (+ (car tv) (* 1e-6 (cadr tv)))))

(defun make-holed-plate (nholes &key (plate-size 1000) (hole-radius 30) (ring-radius 400))
  "square plate with NHOLES circular holes evenly spaced around a
ring -- returns the :top face (the one all the holes are cut into)."
  (let ((plate (make-cube plate-size plate-size 5)))
    (dotimes (i nholes)
      (let* ((angle (* 2 pi (/ (float i) nholes)))
             (cyl (make-cylinder hole-radius 20 :segments 12)))
        (send cyl :translate (float-vector (* ring-radius (cos angle))
                                            (* ring-radius (sin angle))
                                            -10))
        (setq plate (body- plate cyl))))
    (find-if #'(lambda (x) (eq :top (car (send x :id)))) (send plate :faces))))

(defun triangulation-fast-consistency (param-id)
  "Same multi-hole face as triangulation-with-multi-hole, but checks
geo::face-to-triangle-fast against geo::face-to-triangle-aux instead
of just checking geo::face-to-triangle against itself."
  (let ((plate (make-cube 1000 1000 5))
        (params (case param-id
                  (0 (list
                      (list  9 87.841726125643674550 (float-vector -250.0 -250.0 0.0))
                      (list  6 36.824839027036261996 (float-vector -250.0  250.0 0.0))
                      (list  9 52.020745915260789616 (float-vector  250.0 -250.0 0.0))
                      (list 10 51.644968476707617810 (float-vector  250.0  250.0 0.0))
                      (list  6 63.093857512806295063 (float-vector    0.0    0.0 0.0))
                      ))
                  (1 (list
                      (list  9 87.8 (float-vector -250.0 -250.0 0.0))
                      (list  6 36.8 (float-vector -250.0  250.0 0.0))
                      (list  9 52.0 (float-vector  250.0 -250.0 0.0))
                      (list 10 51.6 (float-vector  250.0  250.0 0.0))
                      (list  6 63.0 (float-vector    0.0    0.0 0.0))
                      ))))
        aface faces-aux faces-fast lines)

    (dolist (p params)
      (let ((cyl (make-cylinder (cadr p) 20 :segments (car p))))
        (send cyl :translate (caddr p))
        (send cyl :translate (float-vector 0 0 -10))
        (setq plate (body- plate cyl))
        ))

    (setq aface (find-if #'(lambda (x) (eq :top (car (send x :id)))) (send plate :faces)))

    (setq faces-aux (geo::face-to-triangle-aux aface))
    (setq faces-fast (geo::face-to-triangle-fast aface))

    ;; same number of triangles, same total area (a change in exact
    ;; diagonal choice is fine, a change in covered area is not)
    (assert (= (length faces-aux) (length faces-fast)))
    (assert (eps= (apply #'+ (send-all faces-aux :area))
                  (apply #'+ (send-all faces-fast :area))
                  1e-3))

    ;; make-vertical lines
    (setq lines
          (mapcar #'(lambda (p) (make-line (coerce (append p (list -10)) float-vector)
                                           (coerce (append p (list  10)) float-vector)))
                  (list (list    0  250)
                        (list    0 -250)
                        (list  250    0)
                        (list -250    0)
                        (list    0    0)
                        )))
    ;; intersection between face and line should return the same result after triangulation
    (dolist (l lines)
      (assert
       (equal (null (send aface :intersect-line (send l :pvertex) (send l :nvertex)))
              (null (some #'(lambda (f) (send f :intersect-line (send l :pvertex) (send l :nvertex))) faces-fast))))
      )
    nil
    ))

(defun triangulation-benchmark-many-holes (&optional (nholes 6))
  "Not a correctness test by itself (triangulation-fast-consistency
above covers that) -- this exists to make the actual speed benefit of
wiring GLU into face-to-triangle-fast visible in the test log, since
euslisp/EusLisp#542 by itself only adds the low-level eus_tess_*
collector without demonstrating any real caller benefiting from it.
Still asserts the two paths agree, so a real regression still fails
the test."
  (let* ((aface (make-holed-plate nholes))
         t0 t1 faces-aux faces-fast time-aux time-fast)
    (setq t0 (tri-bench-now))
    (setq faces-aux (geo::face-to-triangle-aux aface))
    (setq t1 (tri-bench-now))
    (setq time-aux (- t1 t0))

    (setq t0 (tri-bench-now))
    (setq faces-fast (geo::face-to-triangle-fast aface))
    (setq t1 (tri-bench-now))
    (setq time-fast (- t1 t0))

    (format t "~%;; triangulation-benchmark-many-holes: ~A holes, ~A boundary vertices + ~A hole vertices~%"
            nholes (length (send aface :vertices))
            (apply #'+ (mapcar #'(lambda (h) (length (send h :vertices))) (send aface :holes))))
    (format t ";;   face-to-triangle-aux  (ear clipping): ~A triangles in ~A sec~%"
            (length faces-aux) time-aux)
    (format t ";;   face-to-triangle-fast (GLU when available): ~A triangles in ~A sec~%"
            (length faces-fast) time-fast)
    (if geo::*glu-tess-available*
        (format t ";;   -> GLU is available here, speedup ~Ax~%" (/ time-aux (max time-fast 1e-6)))
      (format t ";;   -> GLU is NOT available in this build (non-GL eus/irteus); face-to-triangle-fast fell back to the same ear-clipping path, as expected~%"))

    (assert (= (length faces-aux) (length faces-fast)))
    (assert (eps= (apply #'+ (send-all faces-aux :area))
                  (apply #'+ (send-all faces-fast :area))
                  1e-3))
    nil))

(defun tri-area-stats (faces)
  "(:n :min :max :mean :std :ratio) over FACES' :area -- summarizes how
evenly-sized (vs. sliver-prone) a triangulation's output is."
  (let* ((areas (send-all faces :area))
         (n (length areas))
         (mn (reduce #'min areas))
         (mx (reduce #'max areas))
         (mean (/ (apply #'+ areas) n))
         (var (/ (apply #'+ (mapcar #'(lambda (a) (* (- a mean) (- a mean))) areas)) n)))
    (list :n n :min mn :max mx :mean mean :std (sqrt var) :max/min (/ mx mn))))

(defun triangulation-quality-comparison (&optional (nholes 6))
  "Compares face-to-triangle-aux (ear clipping), face-to-triangle-fast
(GLU sweep-line) and face-to-triangle-delaunay (Bowyer-Watson) on the
same many-holed face: all three must cover the identical area (same
region triangulated -- see triangulation-fast-consistency /
triangulation-benchmark-many-holes above for that already being
covered for aux vs fast), but HOW they cut it into triangles differs.
Delaunay maximizes the minimum angle over all triangulations of a
point set (the empty-circumcircle property), so it is expected to
avoid the long thin slivers that ear-clipping and GLU can both
produce -- this prints each method's area-distribution stats so that
difference is visible, rather than just asserting triangle count and
total area match (which triangulation-fast-consistency /
triangulation-benchmark-many-holes already do for aux vs fast)."
  (let* ((aface (make-holed-plate nholes))
         (faces-aux (geo::face-to-triangle-aux aface))
         (faces-fast (geo::face-to-triangle-fast aface))
         (faces-del (geo::face-to-triangle-delaunay aface))
         (area-aux (apply #'+ (send-all faces-aux :area)))
         (area-fast (apply #'+ (send-all faces-fast :area)))
         (area-del (apply #'+ (send-all faces-del :area))))
    (format t "~%;; triangulation-quality-comparison: ~A holes~%" nholes)
    (format t ";;   aux      (ear clipping): ~A~%" (tri-area-stats faces-aux))
    (format t ";;   fast     (GLU)         : ~A~%" (tri-area-stats faces-fast))
    (format t ";;   delaunay (Bowyer-Watson): ~A~%" (tri-area-stats faces-del))
    ;; same total area covered, regardless of how each method cut it up
    (assert (eps= area-aux area-fast (* 1e-6 area-aux)))
    (assert (eps= area-aux area-del (* 1e-2 area-aux)))
    nil))

(deftest test-triangulation-concave
  (triangulation-concave)
  )

(deftest test-triangulation-with-hole-0
  (triangulation-with-multi-hole 0)
  )

(deftest test-triangulation-with-hole-1
  (triangulation-with-multi-hole 1)
  )

(deftest test-triangulation-fast-consistency-0
  (triangulation-fast-consistency 0)
  )

(deftest test-triangulation-fast-consistency-1
  (triangulation-fast-consistency 1)
  )

(deftest test-triangulation-benchmark-many-holes
  (triangulation-benchmark-many-holes)
  )

(deftest test-triangulation-quality-comparison
  (triangulation-quality-comparison)
  )

(eval-when (load eval)
  (run-all-tests)
  (exit 0))
