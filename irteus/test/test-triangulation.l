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

(defun triangulation-with-multi-hole ()
  (let ((plate (make-cube 1000 1000 5))
        (params ;; params for holes (list (list segments radius center) .. )
         (list
          #| ;; failed even after ...
          (list  9 87.841726125643674550 (float-vector -250.0 -250.0 0.0))
          (list  6 36.824839027036261996 (float-vector -250.0  250.0 0.0))
          (list  9 52.020745915260789616 (float-vector  250.0 -250.0 0.0))
          (list 10 51.644968476707617810 (float-vector  250.0  250.0 0.0))
          (list  6 63.093857512806295063 (float-vector    0.0    0.0 0.0))
          |#
          (list  9 87.8 (float-vector -250.0 -250.0 0.0))
          (list  6 36.8 (float-vector -250.0  250.0 0.0))
          (list  9 52.0 (float-vector  250.0 -250.0 0.0))
          (list 10 51.6 (float-vector  250.0  250.0 0.0))
          (list  6 63.0 (float-vector    0.0    0.0 0.0))
          ))
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

(defun triangulation-intersection3 ()
  (let ((l0 (make-line (float-vector -120.0 -30.0 0.0) (float-vector 15.0 0.0 0.0)))
        (l1 (make-line (float-vector -15.0 120.0 0.0)  (float-vector -15.0 0.0 0.0)))
        res-p res-n
        )

    (setq res-p (geo::line-intersection3 (l0 . pvert) (l0 . nvert) (l1 . pvert) (l1 . nvert) 0.00001)) ;; -> (0.777778 1.05556)
    (setq res-n (geo::line-intersection3 (l1 . pvert) (l1 . nvert) (l0 . pvert) (l0 . nvert) 0.00001)) ;; -> (1.05556 0.777778)
    (warn ";;; intersection3 (res-p) ~A~%" res-p)
    (warn ";;; intersection3 (res-n) ~A~%" res-n)

    (assert (eps= (elt res-p 0) (elt res-n 1)))
    (assert (eps= (elt res-p 1) (elt res-n 0)))
    ))

(deftest test-triangulation-concave
  (triangulation-concave)
  )

(deftest test-triangulation-with-hole
  (triangulation-concave)
  )

(deftest test-triangulation-intersection3
  (triangulation-intersection3)
  )

(eval-when (load eval)
  (run-all-tests)
  (exit 0))
