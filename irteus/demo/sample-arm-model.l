;;;
;;;        Element Parts of Standard Manipulator
;;;        for Intelligent Robotics
;;;                JSK, The University of Tokyo
;;;        
;;;                2004.12.13 created by M. Inaba
;;;
(defun make-sarm-base nil
  (let ((b1 (make-cube 180 136 167))
        (c (make-cylinder 35 65.5))
        (b0 (make-prism
             (list
              (float-vector -140 700 -20)
              (float-vector 700 700 -20)
              (float-vector 700 -700 -20)
              (float-vector -140 -700 -20))
             20))
        )
    (send b1 :locate
          (float-vector -50
                        0
                        (- (/ 165 2.0) 1.0)))
    (send c :locate (float-vector 0 0 164.5))
    (setq b1 (body+ b1 c))
    (send b1 :worldcoords)
    b1))

(defun make-sarm-l1 nil
  (let* ((h 136)
         (h/2 (/ h -2))
         (b (make-prism
             (list
              (float-vector  -40 h/2 -70)
              (float-vector   40 h/2 -70)
              (float-vector   40 h/2   0)
              (float-vector   30 h/2  40)
              (float-vector  -30 h/2  40)
              (float-vector  -40 h/2   0))
             (float-vector 0 h 0)))
         (d 100)
         (d/2 (/ d -2))
         (b1
          (make-prism
           (list
            (float-vector -50 d/2 -60)
            (float-vector  50 d/2 -60)
            (float-vector  50 d/2  80)
            (float-vector -50 d/2  80)
            )
           (float-vector 0 d 0)))
         )
    (setq b (body- b b1))
    (send b :worldcoords)
    b)
  )

(defun make-sarm-l2 nil
  (let*
      ((b (make-prism
           (list
            (float-vector  40    0 -40)
            (float-vector  30  -30 -40)
            (float-vector  15  -40 -40)
            (float-vector -15  -40 -40)
            (float-vector -30  -30 -40)
            (float-vector -40    0 -40)
            (float-vector -40  150 -40)
            (float-vector -30  300 -40)
            (float-vector
             (- (* 30 (cos #d30)))
             (+ 300 (* 30 (sin #d30)))
             -40)
            (float-vector
             (- (* 30 (cos #d60)))
             (+ 300 (* 30 (sin #d60)))
             -40)
            (float-vector
             (- (* 30 (cos #d90)))
             (+ 300 (* 30 (sin #d90)))
             -40)
            (float-vector
             (* 30 (cos #d60))
             (+ 300 (* 30 (sin #d60)))
             -40)
            (float-vector
             (* 30 (cos #d30))
             (+ 300 (* 30 (sin #d30)))
             -40)
            (float-vector 30 300 -40)
            (float-vector 40 150 -40)
            )
           80))
       (b1 (make-prism
            (list
             (float-vector  50  180 -35)
             (float-vector -50  180 -35)
             (float-vector -50  350 -35)
             (float-vector  50  350 -35))
            70))
       )
    (setq b (body- b b1))
    (send b :worldcoords)
    b)
  )

(defun make-sarm-l3 nil
  (let ((b (make-prism
            (list
             (float-vector 142  30 -30)
             (float-vector 142 -30 -30)
             (float-vector   0 -30 -30)
             (float-vector
              (- (* 30 (sin #d30)))
              (- (* 30 (cos #d30)))
              -30)
             (float-vector
              (- (* 30 (sin #d60)))
              (- (* 30 (cos #d60)))
              -30)
             (float-vector
              (- (* 30 (sin #d90)))
              (- (* 30 (cos #d90)))
              -30)
             (float-vector
              (- (* 30 (sin #d60)))
              (* 30 (cos #d60))
              -30)
             (float-vector
              (- (* 30 (sin #d30)))
              (* 30 (cos #d30))
              -30)
             (float-vector 0 30 -30)
             )
            60)))
    (send b :worldcoords)
    b))

(defun make-sarm-l4 nil
  (let
      ((b (make-prism
           (list
            (float-vector
             -30
             (* 30 (cos #d150))
             (+ 400 (* 30 (sin #d150))))
            (float-vector
             -30
             (* 30 (cos #d120))
             (+ 400 (* 30 (sin #d120))))
            (float-vector
             -30
             (* 30 (cos #d90))
             (+ 400 (* 30 (sin #d90))))
            (float-vector
             -30
             (* 30 (cos #d60))
             (+ 400 (* 30 (sin #d60))))
            (float-vector
             -30
             (* 30 (cos #d30))
             (+ 400 (* 30 (sin #d30))))
            (float-vector -30  30  143)
            (float-vector -30 -30  143)
            (float-vector -30 -30  400)
            )
           (float-vector 60 0 0)))
       (b1 (make-prism
	    (list
             (float-vector -26  40 437)
             (float-vector -26  40 367)
             (float-vector -26 -40 367)
             (float-vector -26 -40 437)
             )
            (float-vector 52 0 0)))
       )
    (setq b (body- b b1))
    (send b :worldcoords)
    b))

(defun make-sarm-l5 nil
  (let
      ((b (make-prism
           (list
            (float-vector
             (* 24 (sin #d0))
             (* 24 (cos #d0)) -25)
            (float-vector
             (* 24 (sin #d30))
             (* 24 (cos #d30)) -25)
            (float-vector
             (* 24 (sin #d60))
             (* 24 (cos #d60)) -25)
            (float-vector
             (* 24 (sin #d90))
             (* 24 (cos #d90)) -25)
            (float-vector
             (* 24 (sin #d120))
             (* 24 (cos #d120)) -25)
            (float-vector
             (* 24 (sin #d150))
             (* 24 (cos #d150)) -25)
            (float-vector
             (* 24 (sin #d180))
             (* 24 (cos #d180)) -25)
            (float-vector
             (* 24 (sin #d210))
             (* 24 (cos #d210)) -25)
            (float-vector
             (* 24 (sin #d240))
             (* 24 (cos #d240)) -25)
            (float-vector
             (* 24 (sin #d270))
             (* 24 (cos #d270)) -25)
            (float-vector
             (* 24 (sin #d300))
             (* 24 (cos #d300)) -25)
            (float-vector
             (* 24 (sin #d330))
             (* 24 (cos #d330)) -25)
            )
           50))
       (b1 (make-prism
            (list
             (float-vector 0 19 -17)
             (float-vector 34 19 -17)
             (float-vector 34 -19 -17)
             (float-vector 0 -19 -17)
             )
            34))
       )
    (setq b (body+ b b1))
    (send b :worldcoords)
    b)
  )

(defun make-sarm-l6 nil
  (let ((b (make-prism
            (list
             (float-vector -20 -15 35)
             (float-vector  20 -15 35)
             (float-vector  20 -15 80)
             (float-vector -20 -15 80)
             )
            (float-vector 0 30 0)))
        (b1 (make-prism
             (list
              (float-vector -55 -13  75)
              (float-vector  55 -13  75)
              (float-vector  55 -13 100)
              (float-vector -55 -13 100)
              )
             (float-vector 0 26 0)))
        (b2 (make-prism
             (list
              (float-vector -53 -12  78)
              (float-vector  53 -12  78)
              (float-vector  53 -12 110)
              (float-vector -53 -12 110)
              )
             (float-vector 0 24 0)))
        )
    (setq b (body+ b b1))
    (setq b (body- b b2))
    (send b :worldcoords)
    b))

(defun make-sarm-finger-r nil
  ;; 40mm 10mm 19mm
  (let ((b (make-prism
            (list
             (float-vector -9.5  0 -0.5)
             (float-vector -9.5 80 -0.5)
             (float-vector -9.5 80   -5)
             (float-vector -9.5 40  -10)
             (float-vector -9.5  0  -10)
             )
            (float-vector 19 0 0))))
    (send b :worldcoords)
    b)))

(defun make-sarm-finger-l nil
  ;; 40mm 10mm 19mm
(let ((b (make-prism
          (list
           (float-vector -9.5  0  0.5)
           (float-vector -9.5 80  0.5)
           (float-vector -9.5 80    5)
           (float-vector -9.5 40   10)
           (float-vector -9.5  0   10)
           )
          (float-vector 19 0 0)))
      )
  (send b :worldcoords)
  b))

(defclass sarmclass 
  :super robot-model
  :slots (end-coords
	  ;;jointx jointy jointz
	  joint0 joint1 joint2 joint3 joint4 joint5
	  joint-fr joint-fl))
(defmethod sarmclass
  (:init
   (&rest args)
   (let (;;sarm-x sarm-y sarm-z
		sarm-b0 sarm-b1 sarm-b2 sarm-b3 sarm-b4 sarm-b5 sarm-b6
		 sarm-fr sarm-fl)
     (send-super* :init args)

     ;;(setq sarm-x (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 10 10 10)) :name :sarm-x))
     ;;(setq sarm-y (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 10 10 10)) :name :sarm-y))
     ;;(setq sarm-z (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 10 10 10)) :name :sarm-z))

     (setq sarm-b0 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-base)) :name :sarm-b0))
     (setq sarm-b1 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l1)) :name :sarm-b1))
     (setq sarm-b2 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l2)) :name :sarm-b2))
     (setq sarm-b3 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l3)) :name :sarm-b3))
     (setq sarm-b4 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l4)) :name :sarm-b4))
     (setq sarm-b5 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l5)) :name :sarm-b5))
     (setq sarm-b6 (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-l6)) :name :sarm-b6))
     (setq sarm-fr (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-finger-r)) :name :sarm-fr))
     (setq sarm-fl (instance bodyset-link :init (make-cascoords) :bodies (list (make-sarm-finger-l)) :name :sarm-fl))

     (send sarm-b0 :bodies :set-color :limegreen)
     (send sarm-b1 :bodies :set-color :palegreen)
     (send sarm-b2 :bodies :set-color :royalblue)
     (send sarm-b3 :bodies :set-color :lightskyblue)
     (send sarm-b4 :bodies :set-color :orange)
     (send sarm-b5 :bodies :set-color :orangered)
     (send sarm-b6 :bodies :set-color :pink)

     (send sarm-fl :bodies :set-color :blue)
     (send sarm-fr :bodies :set-color :red)

     (send sarm-fr :transform
           (make-coords :pos #f(0 0 75)
                        :rpy (float-vector pi/2 0 pi/2)))
     (send sarm-fl :transform
           (make-coords :pos #f(0 0 75)
                        :rpy (float-vector pi/2 0 pi/2)))

     (setq end-coords (make-cascoords :pos #f(0 0 150) :rpy (float-vector pi pi 0)))

     (send sarm-b6 :assoc end-coords)
     (send sarm-b6 :assoc sarm-fr)
     (send sarm-b6 :assoc sarm-fl)
     (send sarm-b6 :transform
	   (make-coords :pos #f(0 0 0)
			:rot (rotation-matrix pi/2 :y)))

     (send sarm-b5 :assoc sarm-b6)
     (send sarm-b5 :transform
	   (make-coords :pos #f(0 0 400)
			:rot (rotation-matrix -pi/2 :y)))

     (send sarm-b4 :assoc sarm-b5)
     (send sarm-b4 :transform
	   (make-coords :pos #f(0 0 0)
			:rot (rotation-matrix pi/2 :y)))

     (send sarm-b3 :assoc sarm-b4)
     (send sarm-b3 :transform
	   (make-coords :pos #f(0 310 0)
			:rot (rotation-matrix pi/2 :z)))

     (send sarm-b2 :assoc sarm-b3)
     (send sarm-b2 :transform
	   (make-coords :pos #f(0 0 0)
			:rot (rotation-matrix pi/2 :x)))

     (send sarm-b1 :assoc sarm-b2)
     (send sarm-b1 :transform
	   (make-coords :pos #f(0 0 300)))

     (send sarm-b0 :assoc sarm-b1)
     (send self :assoc sarm-b0)
     ;;(send sarm-z :assoc sarm-b0)
     ;;(send sarm-y :assoc sarm-z)
     ;;(send sarm-x :assoc sarm-y)
     ;;(send self :assoc sarm-x)

     ;;(setq jointx (instance linear-joint :init :parent-link sarm-x :child-link sarm-y :name :jointx :axis :x :min *-inf* :max *inf*))
     ;;(setq jointy (instance linear-joint :init :parent-link sarm-y :child-link sarm-z :name :jointy :axis :y :min *-inf* :max *inf*))
     ;;(setq jointz (instance rotational-joint :init :parent-link sarm-z :child-link sarm-b0 :name :jointz :axis :z :min *-inf* :max *inf*))

     (setq joint0 (instance rotational-joint :init :parent-link sarm-b0 :child-link sarm-b1 :name :joint0 :axis :z :min -85 :max 85))
     (setq joint1 (instance rotational-joint :init :parent-link sarm-b1 :child-link sarm-b2 :name :joint1 :axis :-z :min -30 :max 120))
     (setq joint2 (instance rotational-joint :init :parent-link sarm-b2 :child-link sarm-b3 :name :joint2 :axis :-z :min -30 :max 150))
     (setq joint3 (instance rotational-joint :init :parent-link sarm-b3 :child-link sarm-b4 :name :joint3 :axis :z :min -165 :max 165))
     (setq joint4 (instance rotational-joint :init :parent-link sarm-b4 :child-link sarm-b5 :name :joint4 :axis :-z :min -90 :max 90))
     (setq joint5 (instance rotational-joint :init :parent-link sarm-b5 :child-link sarm-b6 :name :joint5 :axis :z :min -180 :max 180))
     (setq joint-fl (instance linear-joint :init :parent-link sarm-b6 :child-link sarm-fr :name :joint-fl :axis :-z :min 0 :max 42))
     (setq joint-fr (instance linear-joint :init :parent-link sarm-b6 :child-link sarm-fl :name :joint-fr :axis :z :min 0 :max 42))

     (setq links (list ;;sarm-x sarm-y sarm-z
                  sarm-b0 sarm-b1 sarm-b2 sarm-b3 sarm-b4 sarm-b5 sarm-b6
                  sarm-fr sarm-fl))
     (setq joint-list (list ;;jointx jointy jointz
                       joint0 joint1 joint2 joint3 joint4 joint5
                       joint-fl joint-fr))
     (setq collision-avoidance-links (list sarm-b2 sarm-b3 sarm-b4 sarm-b5 sarm-b6))

     (send self :init-ending)
     self))
  (:joint0 (&rest args) (forward-message-to joint0 args))
  (:joint1 (&rest args) (forward-message-to joint1 args))
  (:joint2 (&rest args) (forward-message-to joint2 args))
  (:joint3 (&rest args) (forward-message-to joint3 args))
  (:joint4 (&rest args) (forward-message-to joint4 args))
  (:joint5 (&rest args) (forward-message-to joint5 args))
  (:joint-fl (&rest args) (forward-message-to joint-fl args))
  (:joint-fr (&rest args) (forward-message-to joint-fr args))
  (:end-coords (&rest args) (forward-message-to end-coords args))
  (:reset-pose
   ()
   (send self :angle-vector #f(0.0 0.0 90.0 0.0 90.0 0.0 0.0 0.0))
   )
  (:move-fingers
   (l)
   (send joint-fl :joint-angle l)
   (send joint-fr :joint-angle l))
  (:init-ending
   ()
   (setq bodies (flatten (send-all links :bodies)))
   (dolist (j joint-list)
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (if (send j :parent-link)
	 (send (send j :parent-link) :add-child-links (send j :child-link)))
     )
   (send self :update-descendants))
  (:base-link (&rest args) (forward-message-to (send jointz :child-link) args))
  )

(defmethod sarmclass
  (:open-hand
   ()
   (let ((obj (get self :grasping-object)))
     (send (send self :end-coords :parent) :dissoc obj)
     (setf (get self :grasping-object) nil)
     (send self :move-fingers 50)))
  (:close-hand
   (&optional obj)
   (if obj
       (progn
         (let (a)
           (setq a (send self :open-hand))
           (while (> a 0)
             (if (pqp-collision-check-objects
                  (list (send self :joint-fr :child-link)
                        (send self :joint-fl :child-link))
                  (list obj))
                 (return))
             (send self :move-fingers a)
             (decf a 1))
           (setf (get self :grasping-object) obj)
           (send (send self :end-coords :parent) :assoc obj)))
     (send self :move-fingers 0)))

  )

;;;
;;;
;;;

