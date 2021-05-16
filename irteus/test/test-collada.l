(require :unittest "lib/llib/unittest.l")

(defclass testrobot
  :super cascaded-link
  :slots (j0 j1))
(defmethod testrobot
  (:init
   (&key (name "testrobot") (move-root))
   (send-super :init)
   (let ((b (instance bodyset-link :init (make-cascoords) :name 'body
                      :bodies (list (make-cube 100 100 100))))
         (l0 (instance bodyset-link :init (make-cascoords) :name 'l0
                       :bodies (list (make-cube 100 100 100))))
         (l1 (instance bodyset-link :init (make-cascoords) :name 'l1
                       :bodies (list (make-cube 100 100 100))))
         )
     (send self :assoc b)

     (send l0 :locate (float-vector 0   0 100))
     (send l1 :locate (float-vector 0 100 100))

     (send b  :assoc l0)
     (send l0 :assoc l1)

     (if move-root (send b :translate (float-vector 0 0 500)))

     (setq j0 (instance rotational-joint :init :name 'j0
                        :parent-link b  :child-link l0 :axis :x))
     (setq j1 (instance rotational-joint :init :name 'j1
                        :parent-link l0 :child-link l1 :axis :x))

     (setq links (list b l0 l1))
     (setq joint-list (list j0 j1))

     (send self :init-ending)
     )
   self)
  )

(init-unit-test)

(deftest test-link-offset ()
  (setq *normal-model* (instance testrobot :init :move-root nil))
  (setq *moved-model* (instance testrobot :init :move-root t))
  ;;
  (let* ((desc (collada::eusmodel-description *normal-model*))
         (c0 (send (cdr (assoc :origin (elt (car desc) 0))) :copy-worldcoords))
         (c1 (send (cdr (assoc :origin (elt (car desc) 1))) :copy-worldcoords))
         (c2 (send (cdr (assoc :origin (elt (car desc) 2))) :copy-worldcoords)))
    (send c0 :transform c1)
    (send c0 :transform c2)
    (assert (eps-coords= c0
                         (make-coords :pos (float-vector 0 100 100)))
            (format nil "normal-model / coords ~A is not correct" c0))

    )
  ;;
  (let* ((desc (collada::eusmodel-description *moved-model*))
         (c0 (send (cdr (assoc :origin (elt (car desc) 0))) :copy-worldcoords))
         (c1 (send (cdr (assoc :origin (elt (car desc) 1))) :copy-worldcoords))
         (c2 (send (cdr (assoc :origin (elt (car desc) 2))) :copy-worldcoords)))
    (send c0 :transform c1)
    (send c0 :transform c2)
    (assert (eps-coords= c0
                         (make-coords :pos (float-vector 0 100 600)))
            (format nil "moved-model / coords ~A is not correct" c0))

    )
  )

(deftest test-mesh-offset ()
  (setq *normal-model* (instance testrobot :init :move-root nil))
  (setq *moved-model* (instance testrobot :init :move-root t))
  ;;
  (let* ((desc (collada::eusmodel-description *normal-model*))
         (mesh0 (make-bounding-box (send (cadr (assoc :mesh (elt (car desc) 0))) :vertices) 0.0))
         (mesh2 (make-bounding-box (send (cadr (assoc :mesh (elt (car desc) 2))) :vertices) 0.0))
         )
    (assert (eps-v= (send mesh0 :center) (float-vector 0 0 0))
            (format nil "normal / mesh0 (~A) is not equal #f(0 0 0)"
                    (send mesh0 :center)))
    (assert (eps-v= (send mesh2 :center) (float-vector 0 100 100))
            (format nil "normal / mesh2 (~A) is not equal #f(0 100 100)"
                    (send mesh2 :center)))
    )
  ;;
  (let* ((desc (collada::eusmodel-description *moved-model*))
         (mesh0 (make-bounding-box (send (cadr (assoc :mesh (elt (car desc) 0))) :vertices) 0.0))
         (mesh2 (make-bounding-box (send (cadr (assoc :mesh (elt (car desc) 2))) :vertices) 0.0))
         )
    (assert (eps-v= (send mesh0 :center) (float-vector 0 0 0))
            (format nil "moved / mesh0 (~A) is not equal #f(0 0 0)"
                    (send mesh0 :center)))
    (assert (eps-v= (send mesh2 :center) (float-vector 0 100 100))
            (format nil "moved / mesh2 (~A) is not equal #f(0 100 100)"
                    (send mesh2 :center)))
    )
  )

(eval-when (load eval)
  (run-all-tests)
  (exit))
