;; test code for joint
;; setup sample robot

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(defclass 2dof-robot
  :super cascaded-link
  :slots (end-coords l1 l2 l3 j1 j2))
(defmethod 2dof-robot
  (:init ()
         (send-super :init)
         (setq l3 (send self :make-link (make-cube 40 40 80) #f(0 0 40) :read :l3))
         (setq end-coords (make-cascoords :pos #f(0 0 80)))
         (send l3 :assoc end-coords)
         (send l3 :locate #f(0 0 10))
         ;;
         (setq l2 (send self :make-link (make-cube 60 60 10) #f(0 0 5) :blue 'l2))
         (send l2 :assoc l3)
         (send l2 :locate #f(0 0 80))
         ;;
         (setq l1 (send self :make-link (body+ (make-cube 40 40 80 :pos #f(0 0 40))
                                               (make-cube 300 300 2)) #f(0 0 0) :white 'l1))
         (send l1 :assoc l2)
         (setq j1 (instance rotational-joint :init :parent-link l1 :child-link l2 :axis :z
                            :min-angle -90 :max-angle 90)
               j2 (instance rotational-joint :init :parent-link l2 :child-link l3 :axis :y
                            :min-angle -90 :max-angle 90))
         ;;
         (setq links (list l1 l2 l3) joint-list (list j1 j2))
     (send self :init-ending)
     self)
  (:make-link (b off color name)
     (send b :locate off) (send b :set-color color)
     (instance bodyset-link :init (make-cascoords) :bodies (list b) :name name))
  ;;
  (:j1 (&rest args) (forward-message-to j1 args))
  (:j2 (&rest args) (forward-message-to j2 args))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  )

(setq *robot* (instance 2dof-robot :init))
(objects (list *robot*))

;; min-max
(deftest test-min-max
  (let ((j1 (send *robot* :j1))
        (j2 (send *robot* :j2)))
    (do ((x (send j1 :min-angle) (+ x 10)))
        ((> x (send j1 :max-angle)))
      (do ((y (send j2 :min-angle) (+ y 10)))
          ((> y (send j2 :max-angle)))
        (send *robot* :angle-vector (float-vector x y))
        (assert (= (send j1 :joint-angle) x) (format nil "check joint value for ~A match (= ~A ~A)" (send j1 :name) (send j1 :joint-angle) x))
        (assert (= (send j2 :joint-angle) y) (format nil "check joint value for ~A match (= ~A ~A)" (send j2 :name) (send j2 :joint-angle) y))
        (send *irtviewer* :draw-objects)
        ))
    ))

(deftest test-min-max-exceed
  (let ((j1 (send *robot* :j1))
        (j2 (send *robot* :j2))
        x y)
    ;;
    (dotimes (i 100)
      (setq x (+ (random (- (send j1 :max-angle) (send j1 :min-angle))) (send j1 :min-angle))
            y (+ (random (- (send j2 :max-angle) (send j2 :min-angle))) (send j2 :min-angle)))

      (if (= (random 2) 1)
          (incf x (+ (random x)))
        (incf y (+ (random y))))
      ;;
      (send *robot* :angle-vector (float-vector x y))
      (setq x (send j1 :joint-angle)
            y (send j2 :joint-angle))
      ;;
      (cond ((< x (send j1 :min-angle)) ;; x is too small
             (assert (= (send j1 :min-angle) x) (format nil "check joint value for ~A match (= ~A ~A) (joint-angle:~A min-angle:~A)" (send j1 :name) (send j1 :min-angle) x (send j1 :joint-angle) (send j1 :min-angle))))
            ((> x (send j1 :max-angle)) ;; x is too big
             (assert (= (send j1 :max-angle) x) (format nil "check joint value for ~A match (= ~A ~A) (joint-angle:~A max-angle:~A)" (send j1 :name) (send j1 :max-angle) x (send j1 :joint-angle) (send j1 :max-angle))))
            (t
             (assert (= (send j1 :joint-angle) x) (format nil "check joint value for ~A match (= ~A ~A)" (send j1 :name) (send j1 :joint-angle) x))))

      (cond ((< y (send j2 :min-angle)) ;; y is too small
             (assert (= (send j2 :min-angle) y) (format nil "check joint value for ~A match (= ~A ~A) (joint-angle:~A min-angle:~A)" (send j2 :name) (send j2 :min-angle) y (send j2 :joint-angle) (send j2 :min-angle))))
            ((> y (send j2 :max-angle)) ;; y is too big
             (assert (= (send j2 :max-angle) y) (format nil "check joint value for ~A match (= ~A ~A) (joint-angle:~A max-angle:~A)" (send j2 :name) (send j2 :max-angle) y (send j2 :joint-angle) (send j2 :max-angle))))
            (t
             (assert (= (send j2 :joint-angle) y) (format nil "check joint value for ~A match (= ~A ~A)" (send j2 :name) (send j2 :joint-angle) y))))
      ;;
      (send *irtviewer* :draw-objects)
      )
    ))

;; min-max-table
(deftest test-min-max-table
  (let* ((j1 (send *robot* :j1))
         (j2 (send *robot* :j2))
         (j1-min-max-table (make-hash-table))
         (j2-min-max-table (make-hash-table))
         (j1-org-min-angle (send j1 :min-angle)) (j2-org-min-angle (send j2 :min-angle))
         (j1-org-max-angle (send j1 :max-angle)) (j2-org-max-angle (send j2 :max-angle))
         min-max-table-view
         x y)

    ;; set min-max-table
    ;; j1-hash
    (mapcar #'(lambda (self-joint target-joint j-min-max-table)
                (do ((i (round (send target-joint :min-angle)) (+ i 1))) ((> i (round (send target-joint :max-angle))))
                  (setf (gethash (round i) j-min-max-table) (cons (- (abs i) (abs (send self-joint :min-angle))) (- (send self-joint :max-angle) (abs i)))))
                (send self-joint :joint-min-max-table j-min-max-table)
                (send self-joint :joint-min-max-target target-joint))
            (list j1 j2)
            (list j2 j1)
            (list j1-min-max-table j2-min-max-table))

    ;; display min-max table if needed
    (unless (or (null x::*display*) (= x::*display* 0))
      (setq min-max-table-view
            (instance x::panel :create
                      :width  (round (- j1-org-max-angle j1-org-min-angle))
                      :height (round (- j2-org-max-angle j2-org-min-angle))
                      :atitle "min-max-table-view"))
      ;;
      (do ((x j1-org-min-angle (+ x 1)))
          ((> x j1-org-max-angle))
        (do ((y j2-org-min-angle (+ y 1)))
            ((> y j2-org-max-angle))
          (let* ((j1-min-max (gethash (round y) j1-min-max-table)) ;; j1-min-max-table is functoin of j2
                 (j2-min-max (gethash (round x) j2-min-max-table))
                 (j1-min (car j1-min-max))
                 (j1-max (cdr j1-min-max))
                 (j2-min (car j2-min-max))
                 (j2-max (cdr j2-min-max)))
            (cond ((and (<= j1-min x j1-max)
                        (<= j2-min y j2-max)) ;; ok
                   (send min-max-table-view :color #xffffff)
                   (send min-max-table-view :draw-line (float-vector (+ x j1-org-max-angle) (+ y j2-org-max-angle)) (float-vector (+ x j1-org-max-angle) (+ y j2-org-max-angle)))
                   ;;
                   )
                  ((and (or (< x j1-min) (< j1-max x))
                        (or (< y j2-min) (< j2-max y))) ;; vaiolated ok
                   (send min-max-table-view :color #x000000)
                   (send min-max-table-view :draw-line (float-vector (+ x j1-org-max-angle) (+ y j2-org-max-angle)) (float-vector (+ x j1-org-max-angle) (+ y j2-org-max-angle)))
                   )
                  (t
                   ;; ??
                   ))
            )))
      (send min-max-table-view :flush)
      );; display
    ;; do test
    (let ((ret t))
      (dotimes (i (round (* (- j1-org-max-angle j1-org-min-angle) (- j2-org-max-angle j2-org-min-angle))))
        (setq x (round (+ (random (- j1-org-max-angle j1-org-min-angle)) j1-org-min-angle))
              y (round (+ (random (- j2-org-max-angle j2-org-min-angle)) j2-org-min-angle)))
        ;;
        (send *robot* :angle-vector (float-vector x y))
        ;;
        ;; check if x-y with in the limit
        (unless (eq (and (= x (send j1 :joint-angle)) (= y (send j2 :joint-angle))) ;; check if :anlge-vector did not modify value
                    (<= (abs y) (- 90 (abs x)))) ;; ground truth
          (if min-max-table-view
              (format t ";; Is (~A ~A) safe posture? ~A ~A~%" x y (and (= x (send j1 :joint-angle)) (= y (send j2 :joint-angle))) (<= (abs y) (- 90 (abs x))))))
        ;; check in view
        (cond ((and (= x (send j1 :joint-angle)) (= y (send j2 :joint-angle)) min-max-table-view) ;; with i nlimit
               (send min-max-table-view :color #x0000ff))
              (t ;; out of limit
               (if min-max-table-view (format t ";; out of limit ~A -> ~A~%" (float-vector x y) (send *robot* :angle-vector)))
               (setq ret
                     (and ret
                          (< (- (- (abs (send j2 :joint-angle)) (abs j1-org-max-angle)) *epsilon*) (send j1 :joint-angle) (+ (- (abs j1-org-max-angle) (abs (send j2 :joint-angle))) *epsilon*))
                            (< (- (- (abs (send j1 :joint-angle)) (abs j2-org-max-angle)) *epsilon*) (send j2 :joint-angle) (+ (- (abs j2-org-max-angle) (abs (send j1 :joint-angle))) *epsilon*))))
               (when min-max-table-view
                 (send min-max-table-view :color #xff0000)
                 (send min-max-table-view :draw-line
                       (float-vector (- x j1-org-min-angle) (- y j2-org-min-angle))
                       (v- (send *robot* :angle-vector) (float-vector j1-org-min-angle j2-org-min-angle))))))
        (when min-max-table-view
          (send min-max-table-view :draw-line (float-vector (- x j1-org-min-angle) (- y j2-org-min-angle)) (float-vector (- x j1-org-min-angle) (- y j2-org-min-angle)))
          (send min-max-table-view :flush)
          ))
      (assert ret ";; check min-max violation")
      )
    ;;
    ;; restore
    (setq (j1 . joint-min-max-table) nil)
    (setq (j1 . joint-min-max-target) nil)
    (setq (j2 . joint-min-max-table) nil)
    (setq (j2 . joint-min-max-target) nil)
    ))

(run-all-tests)
(exit)
