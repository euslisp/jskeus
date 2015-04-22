;; test code for angle-vector with min-max

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; setup sample robot
(defclass 2dof-robot
  :super robot-model
  :slots (end-coords l1 l2 l3 j1 j2))
(defmethod 2dof-robot
  (:init ()
         (send-super :init)
         (setq l3 (send self :make-link (make-cube 4 3 10) #f(0 0 5) :white 'l3))
         (setq end-coords (make-cascoords :pos #f(0 0 80)))
         (send l3 :assoc end-coords)
         ;;
         (setq l2 (send self :make-link (make-cube 0 0 0) #f(0 0 0) :blue 'l2))
         (send l2 :assoc l3)
         (send l2 :locate #f(0 0 1.1))
         ;;
         ;; (setq l1 (send self :make-link (body+ (make-cube 6 11 2)
         ;;                                       (make-cube 6 3 4 :pos #f(0 4 0))
         ;;                                       (make-cube 6 3 4 :pos #f(0 -4 0))
	 ;; 				       ) #f(0 0 -1) :blue 'l1))
	 ;;
         (setq l1 (send self :make-link (make-cube 6 11 2) #f(0 0 -1) :blue 'l1))
	 ;;
         (send l1 :assoc l2)
         (setq j1 (instance rotational-joint :init :parent-link l1 :child-link l2 :axis :z
                            :min-angle -90 :max-angle 90)
               j2 (instance rotational-joint :init :parent-link l2 :child-link l3 :axis :y
                            :min-angle -45 :max-angle 45))
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


;; test angle-vector without min-max
(defun test-joint-common
  (&key (margin 0))
  (let* ((j1-min-orig (send *robot* :j1 :min-angle)) (j2-min-orig (send *robot* :j2 :min-angle))
	 (j1-max-orig (send *robot* :j1 :max-angle)) (j2-max-orig (send *robot* :j2 :max-angle))
	 tmpj2-j1
	 tmpj1-j2)
    (do ((x j1-min-orig (+ x 5)))
	((> x j1-max-orig))
      (do ((y j2-min-orig (+ y 5)))
	  ((> y j2-max-orig))
	(send *robot* :j2 :joint-angle y)
	(send *robot* :j1 :joint-angle x)
	(setq tmpj2-j1 (send *robot* :angle-vector))
	(send *robot* :angle-vector (float-vector x y))
	(assert (eps-v= (send *robot* :angle-vector) tmpj2-j1 0.01) (format nil "check angle-vector match (v= ~A ~A)" (send *robot* :angle-vector) tmpj2-j1))
	(send *robot* :j1 :joint-angle x)
	(send *robot* :j2 :joint-angle y)
	(setq tmpj1-j2 (send *robot* :angle-vector))
	(send *robot* :angle-vector (float-vector x y))
	(assert (eps-v= (send *robot* :angle-vector) tmpj1-j2 0.01) (format nil "check angle-vector match (v= ~A ~A)" (send *robot* :angle-vector) tmpj1-j2))
	(send *irtviewer* :draw-objects)
	))))  

;; test angle-vector with min-max
(defun test-make-joint-min-max-table-common
  (&key (margin 0)) ;; [deg]
  (let* ((j1-min-orig (send *robot* :j1 :min-angle)) (j2-min-orig (send *robot* :j2 :min-angle))
	 (j1-max-orig (send *robot* :j1 :max-angle)) (j2-max-orig (send *robot* :j2 :max-angle))
	 tmpj2-j1
	 tmpj1-j2)
    (send *robot* :self-collision-check)
    (send *robot* :make-joint-min-max-table
    	  (send (send *robot* :j1) :parent-link) (send (send *robot* :j2) :child-link) (send *robot* :j1) (send *robot* :j2)
    	  :margin margin)
    (do ((x j1-min-orig (+ x 5)))
	((> x j1-max-orig))
      (do ((y j2-min-orig (+ y 5)))
	  ((> y j2-max-orig))
	(send *robot* :j2 :joint-angle y)
	(send *robot* :j1 :joint-angle x)
	(setq tmpj2-j1 (send *robot* :angle-vector))
	(send *robot* :angle-vector (float-vector x y))
	(assert (eps-v= (send *robot* :angle-vector) tmpj2-j1 0.01) (format nil "check angle-vector match (v= ~A ~A)" (send *robot* :angle-vector) tmpj2-j1))
	(send *robot* :j1 :joint-angle x)
	(send *robot* :j2 :joint-angle y)
	(setq tmpj1-j2 (send *robot* :angle-vector))
	(send *robot* :angle-vector (float-vector x y))
	(assert (eps-v= (send *robot* :angle-vector) tmpj1-j2 0.01) (format nil "check angle-vector match (v= ~A ~A)" (send *robot* :angle-vector) tmpj1-j2))
	(send *irtviewer* :draw-objects)
	))))

(deftest test-joint-common-margin-0deg
  (test-joint-common :margin 0))

(deftest test-make-joint-min-max-table-margin-0deg
  (test-make-joint-min-max-table-common :margin 0))

(run-all-tests)
(exit)
