;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sample programs for robot models with closed loop kinematics
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; How to make robot model with closed actuators
;;  e.g., musculoskeltal robots and robots with electric or hydrouric cylinder actuators.
;; 1. Make simple link model without closed loop
;; 2. Make closed loop links and joint-list as open loop robot models and append links and joint-list to simple link model
;; 3. Call :init-ending


;; Sample robot with 1dof underactuated rotational joint and 1dof linear actuator such as muscle and cylinder.
(defclass sample-1dof-closed-link-robot
  :super robot-model
  :slots ()
  )

(defmethod sample-1dof-closed-link-robot
  (:init
   ()
   (prog1
       (send-super :init :name "sample-1dof-closed-link-robot")
     ;; 1. Make simple link robot model
     ;;    make link
     (let ((l0 (send self :make-link :root-link :green "attach-coords1"))
           (l1 (send self :make-link :link1 :yellow "attach-coords0"))
           (j0))
       (send l0 :rotate pi/2 :y)
       (send l0 :translate (float-vector -200 0 0) :world)
       (send l1 :translate (float-vector -200 0 0) :world)
       (send self :assoc l0)
       (send l0 :assoc l1)
       ;;    make joint
       (setq j0 (instance rotational-joint :init
                          :parent-link l0 :child-link l1
                          :axis :y))
       (setq links (list l0 l1))
       (setq joint-list (list j0))
       ;; 2. Make closed loop links and joint-list and append them
       (send self :make-cylinder)
       ;; 3. Call init-ending
       (send self :init-ending)
       ))
   )
  (:make-link
   (name color attach-coords-name)
   (let ((b0 (make-cylinder 15 200)))
     (send b0 :set-color color)
     (let ((ln (instance bodyset-link :init (make-cascoords)
                         :bodies (list b0) :name name)))
       (push (make-cascoords :pos (send ln :transform-vector (float-vector 0 0 180))
                             :name attach-coords-name)
             end-coords-list)
       (send ln :assoc (car end-coords-list))
       ln)))
  (:make-cylinder
   ()
   (send self :make-default-linear-link-joint-between-attach-coords
         (send self :end-coords "attach-coords0")
         (send self :end-coords "attach-coords1")
         "end-coords"
         "cylinder-linear-joint")
   )
  ;; Utilities
  (:get-cylinder-length
   ()
   "Get cylinder length [mm]."
   (distance (send (send self :end-coords "attach-coords0") :worldpos)
             (send (send self :end-coords "attach-coords1") :worldpos))
   )
  )

;; Sample robot with 2dof underactuated rotational joint and 2dof linear actuator such as muscle and cylinder.
(defclass sample-2dof-closed-link-robot
  :super robot-model
  :slots ()
  )

(defmethod sample-2dof-closed-link-robot
  (:init
   ()
   (prog1
       (send-super :init :name "sample-2dof-closed-link-robot")
     ;; 1. Make simple link robot model
     ;;    make link
     (let ((l0 (send self :make-root-link))
           (l1 (send self :make-link1))
           (l2 (send self :make-link2))
           (j0) (j1))
       (send l1 :translate (float-vector 0 0 50))
       (send l2 :translate (float-vector 0 0 80))
       (send self :assoc l0)
       (send l0 :assoc l1)
       (send l1 :assoc l2)
       ;;    make joint
       (setq j0 (instance rotational-joint :init
                          :parent-link l0 :child-link l1
                          :axis :x :name :joint1))
       (setq j1 (instance rotational-joint :init
                          :parent-link l1 :child-link l2
                          :axis :y :name :joint2))
       (setq links (list l0 l1 l2))
       (setq joint-list (list j0 j1))
       ;; 2. Make closed loop links and joint-list and append them
       (send self :make-cylinder :right)
       (send self :make-cylinder :left)
       ;; 3. Call init-ending
       (send self :init-ending)
       ))
   )
  (:make-root-link
   ()
   (let ((b0 (make-cylinder 12 50))
         (b1 (make-cylinder 12 100)))
     (send-all (list b0 b1) :set-color :green)
     (send b1 :rotate pi/2 :y)
     (send b1 :translate (float-vector -50 20 20) :world)
     (send b0 :assoc b1)
     (let ((ln (instance bodyset-link :init (make-cascoords)
                         :bodies (list b0 b1) :name :root-link)))
       (dolist (l/r (list :right :left))
         (push (make-cascoords :pos (send ln :transform-vector (float-vector (case l/r (:right -50) (:left 50)) 20 20))
                               :name (format nil "~c-attach-coords1" (elt (string-downcase l/r) 0)))
               end-coords-list)
         (send ln :assoc (car end-coords-list)))
       ln)))
  (:make-link1
   ()
   (let ((b0 (make-cylinder 10 30)))
     (send b0 :set-color :yellow)
     (instance bodyset-link :init (make-cascoords)
               :bodies (list b0) :name :link1)))
  (:make-link2
   ()
   (let ((b0 (make-cylinder 10 50))
         (b1 (make-cylinder 10 70)))
     (send-all (list b0 b1) :set-color :green)
     (send b1 :rotate pi/2 :y)
     (send b1 :translate (float-vector -35 0 20) :world)
     (send b0 :assoc b1)
     (let ((ln (instance bodyset-link :init (make-cascoords)
                         :bodies (list b0 b1) :name :link2)))
       (dolist (l/r (list :right :left))
         (push (make-cascoords :pos (send ln :transform-vector (float-vector (case l/r (:right -35) (:left 35)) 0 20))
                               :name (format nil "~c-attach-coords0" (elt (string-downcase l/r) 0)))
               end-coords-list)
         (send ln :assoc (car end-coords-list)))
       ln)))
  (:make-cylinder
   (l/r)
   (send self :make-default-linear-link-joint-between-attach-coords
         (send self :end-coords (format nil "~c-attach-coords0" (elt (string-downcase l/r) 0)))
         (send self :end-coords (format nil "~c-attach-coords1" (elt (string-downcase l/r) 0)))
         (format nil "~c-end-coords" (elt (string-downcase l/r) 0))
         (format nil "~c-cylinder-linear-joint" (elt (string-downcase l/r) 0))
         ))
  ;; Utilities
  (:get-cylinder-length
   (l/r)
   "Get cylinder length[mm] from :right or :left."
   (distance (send (send self :end-coords (format nil "~c-attach-coords0" (elt (string-downcase l/r) 0))) :worldpos)
             (send (send self :end-coords (format nil "~c-attach-coords1" (elt (string-downcase l/r) 0))) :worldpos))
   )
  )

(defun test-sample-1dof-closed-link-robot
  ()
  "Sample program for sample-1dof-closed-link-robot.
   Inverse calculation and forward calculation are tested."
  (setq *robot* (instance sample-1dof-closed-link-robot :init))
  (objects (list *robot*))
  (let ((cylinder-length-list) (link-angle-list))
    (format t ";; Test inverse calculation (link joint angle => cylinder length).~%")
    (do ((ja -80.0 (+ ja 10.0))) ((> ja 50.0))
      (send (car (send *robot* :joint-list)) :joint-angle ja)
      (send *irtviewer* :draw-objects :flush nil)
      (send *irtviewer* :viewer :draw-line
            (send (send *robot* :end-coords "attach-coords0") :worldpos)
            (send (send *robot* :end-coords "attach-coords1") :worldpos))
      (format t ";;  ja = ~A[deg] => cylinder-len = ~A [mm]~%" ja (send *robot* :get-cylinder-length))
      (send *irtviewer* :viewer :viewsurface :flush)
      (push (send *robot* :get-cylinder-length) cylinder-length-list)
      (push ja link-angle-list)
      (unix:usleep 100000)
      (x::window-main-one)
      )

    (format t ";; Test forward calculation (cylinder length => link joint angle).~%")
    (mapcar #'(lambda (len ja)
                (send *robot* :inverse-kinematics-for-closed-loop-forward-kinematics
                      (list (send *robot* :end-coords "attach-coords1"))
                      :move-target (list (send *robot* :end-coords "end-coords"))
                      :link-list (mapcar #'(lambda (ec)
                                             (send *robot* :link-list (send ec :parent)))
                                         (list (send *robot* :end-coords "end-coords")))
                      :constrained-joint-angle-list (list len)
                      :constrained-joint-list (list (send *robot* :joint "cylinder-linear-joint"))
                      :debug-view :no-message
                      :thre (list 0.05) :rthre (list (deg2rad 0.05)))
                (format t ";;  cylinder-len = ~A[mm] => ref-ja ~A, act-ja ~A[deg]~%" len ja (send (car (send *robot* :joint-list)) :joint-angle))
                (send *irtviewer* :draw-objects)
                (eps= ja (send (car (send *robot* :joint-list)) :joint-angle) 0.1)
                )
            (reverse cylinder-length-list) (reverse link-angle-list))
    ))
(warn "(test-sample-1dof-closed-link-robot) ;; Example for 1dof closed-link robot.~%")

(defun test-sample-2dof-closed-link-robot
  ()
  "Sample program for sample-2dof-closed-link-robot.
   Inverse calculation and forward calculation are tested."
  (setq *robot* (instance sample-2dof-closed-link-robot :init))
  (objects (list *robot*))
  (let ((cylinder-length-list) (link-angle-list))
    (format t ";; Test inverse calculation (link joint angle => cylinder length).~%")
    (do ((j0 -50.0 (+ j0 25.0))) ((> j0 25.0))
      (do ((j1 -50.0 (+ j1 25.0))) ((> j1 50.0))
      (send (car (send *robot* :joint-list)) :joint-angle j0)
      (send (cadr (send *robot* :joint-list)) :joint-angle j1)
      (send *irtviewer* :draw-objects :flush nil)
      (send *irtviewer* :viewer :draw-line
            (send (send *robot* :end-coords (format nil "r-attach-coords0")) :worldpos)
            (send (send *robot* :end-coords (format nil "r-attach-coords1")) :worldpos))
      (send *irtviewer* :viewer :draw-line
            (send (send *robot* :end-coords (format nil "l-attach-coords0")) :worldpos)
            (send (send *robot* :end-coords (format nil "l-attach-coords1")) :worldpos))
      (format t ";;  ja = ~A[deg] => cylinder-len = ~A [mm]~%" (list j0 j1) (list (send *robot* :get-cylinder-length :right) (send *robot* :get-cylinder-length :left)))
      (send *irtviewer* :viewer :viewsurface :flush)
      (push (list (send *robot* :get-cylinder-length :right) (send *robot* :get-cylinder-length :left)) cylinder-length-list)
      (push (list j0 j1) link-angle-list)
      (unix:usleep 100000)
      (x::window-main-one)
      ))

    (format t ";; Test forward calculation (cylinder length => link joint angle).~%")
    (mapcar #'(lambda (len ja)
                (send *robot* :init-pose)
                (send *robot* :inverse-kinematics-for-closed-loop-forward-kinematics
                      (list (send *robot* :end-coords "r-attach-coords1") (send *robot* :end-coords "l-attach-coords1"))
                      :move-target (list (send *robot* :end-coords "r-end-coords") (send *robot* :end-coords "l-end-coords"))
                      :link-list (mapcar #'(lambda (ec)
                                             (send *robot* :link-list (send ec :parent)))
                                         (list (send *robot* :end-coords "r-end-coords") (send *robot* :end-coords "l-end-coords")))
                      :constrained-joint-angle-list len
                      :constrained-joint-list (list (send *robot* :joint "r-cylinder-linear-joint") (send *robot* :joint "l-cylinder-linear-joint"))
                      :debug-view :no-message
                      :thre (list 0.01 0.01) :rthre (list (deg2rad 0.05) (deg2rad 0.05))
                      :manipulability-limit 0.01 :manipulability-gain 1e-10
                      )
                (format t ";;  cylinder-len = ~A[mm] => ref-ja ~A, act-ja ~A[deg]~%" len ja (send-all (subseq (send *robot* :joint-list) 0 2) :joint-angle))
                (send *irtviewer* :draw-objects)
                (every #'identity (mapcar #'(lambda (x y) (eps= x y 0.1)) ja (send-all (subseq (send *robot* :joint-list) 0 2) :joint-angle)))
                )
            (reverse cylinder-length-list) (reverse link-angle-list))
    ))
(warn "(test-sample-2dof-closed-link-robot) ;; Example for 2dof closed-link robot.~%")

