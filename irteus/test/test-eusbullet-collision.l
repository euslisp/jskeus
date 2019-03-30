(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest test-eusbullet-collision-sphere-analytical
  (let* ((radius1 100)
         (radius2 200)
         (obj1 (make-sphere radius1))
         (obj2 (make-sphere radius2))
         (cnt 0)
         (bt-dist)
         (analy-dist)
         (ret)
         )
    (send obj1 :set-color (float-vector 1 0 0) 0.5)
    (send obj2 :set-color (float-vector 0 1 0) 0.4)
    (objects (list obj1 obj2))

    (do-until-key
     ;; move object
     (incf cnt)
     (send obj1 :newcoords (make-coords :pos (float-vector (* 500.0 (sin (/ cnt 20.0))) 50 0)))
     (send *irtviewer* :draw-objects)
     ;; get bullet distance
     (setq ret (bt-collision-distance obj1 obj2))
     (setq bt-dist (elt ret 0))
     ;; get analytical distance and compare
     (setq analy-dist (- (norm (send obj1 :worldpos)) (+ radius1 radius2)))
     (assert (eps= bt-dist analy-dist 1e-3))
     ;; draw
     (send (elt ret 1) :draw-on :flush nil :width 16 :size 50 :color (float-vector 1 0.4 0.4))
     (send (elt ret 2) :draw-on :flush nil :width 16 :size 50 :color (float-vector 0.4 1 0.4))
     (send (make-line (elt ret 1) (elt ret 2)) :draw-on :flush nil
           :width 8 :color (if (> (elt ret 0) 0) (float-vector 0 1 1) (float-vector 1 1 0)))
     (send *irtviewer* :viewer :flush)
     (unix::usleep (* 20 1000))
     (when (> cnt 100)
       (return-from nil nil))
     )
    ))

(eval-when (load eval)
  (run-all-tests)
  (exit 0))
