
(defclass sample-robot
  :super robot-model
  :slots (jc0 jc1 jh0 jh1
              jal0 jal1 jal2 jal3 jal4 jal5 jal6
              jar0 jar1 jar2 jar3 jar4 jar5 jar6
              jll0 jll1 jll2 jll3 jll4 jll5
              jlr0 jlr1 jlr2 jlr3 jlr4 jlr5))
(defmethod sample-robot
  (:init 
   (&rest args
    &key (name "sample-robot")
         ;; key word arguments for configure model size
         (leg-radius 50) (upper-leg-length 250) (lower-leg-length 250) (ankle-length 50)
         (crotch-width 75) (foot-depth 200) (foot-width 100) (foot-thickness 25) (foot-offset 50)
	 (arm-radius 50) (upper-arm-length 275) (lower-arm-length 195) (shoulder-width 150) (hand-length 50)
         (waist-length 100) (torso-length 200)
         ;; mass distribution
         ;;   refering to "WEIGHT, VOLUME, AND CENTER OF MASS OF SEGMENTS OF THE HUMAN BODY", C.E.Clauser et al., AMRL technical report TR-69-70(1969), http://www.dtic.mil/dtic/tr/fulltext/u2/710622.pdf
         ;;   ratio is '((:head 7.3) (:trunk 50.7) (:upper-arm 2.6) (:forearm 1.6) (:hand 0.7) (:tight 10.3) (:calf 4.3) (:foot 1.5))
         ;;   default total mass is 20.9[kg], which is calculated from total-height and avarage 6-years-old-Japanese Male weight http://www.mhlw.go.jp/toukei/youran/data25k/2-06.xls
         (head-weight 1525.7)
         (torso-weight (/ (* torso-length 10596.3) (+ torso-length waist-length)))
         (waist-weight (/ (* waist-length 10596.3) (+ torso-length waist-length)))
         (upper-arm-weight 543.4) (lower-arm-weight 334.4) (hand-weight 146.3)
         (upper-leg-weight 2152.7) (lower-leg-weight 898.7) (foot-weight 313.5)
         )
   (send-super* :init :name name args)
   ;; 1. make links links and assoc all links
   (let ((aroot-link (send self :make-root-link waist-length waist-weight)))
     (setq torso (send self :make-torso-links torso-length torso-weight)
           head (send self :make-head-links head-weight)
           rarm (send self :make-arm-links :rarm
                      arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
                      upper-arm-weight lower-arm-weight hand-weight)
           larm (send self :make-arm-links :larm
                      arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
                      upper-arm-weight lower-arm-weight hand-weight)
           rleg (send self :make-leg-links :rleg
                      leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                      upper-leg-weight lower-leg-weight foot-weight)
           lleg (send self :make-leg-links :lleg
                      leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                      upper-leg-weight lower-leg-weight foot-weight))
     ;; arrange limbs
     (send (car rarm) :translate (float-vector 0 (- shoulder-width) (- torso-length 25)) :world)
     (send (car larm) :translate (float-vector 0 shoulder-width (- torso-length 25)) :world)
     (send (car rleg) :translate (float-vector 0 (- crotch-width) (+ (- waist-length) -50)) :world)
     (send (car lleg) :translate (float-vector 0 crotch-width (+ (- waist-length) -50)) :world)
     (send (car head) :locate (float-vector 0 0 (+ 25 torso-length)))

     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc aroot-link)
     (send aroot-link :assoc (car torso))
     (send (cadr torso) :assoc (car head))
     (send (cadr torso) :assoc (car rarm))
     (send (cadr torso) :assoc (car larm))
     (send aroot-link :assoc (car rleg))
     (send aroot-link :assoc (car lleg))

     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (setq jc0 (instance rotational-joint :init :parent-link aroot-link :child-link (car torso) :name :torso-waist-y :axis :z :min -45 :max 45))
     (setq jc1 (instance rotational-joint :init :parent-link (car torso) :child-link (cadr torso) :name :torso-waist-p :axis :y))

     (setq jh0 (instance rotational-joint :init :parent-link (cadr torso) :child-link (car head) :name :head-neck-y :axis :z :min -150 :max 150))
     (setq jh1 (instance rotational-joint :init :parent-link (car head) :child-link (cadr head) :name :head-neck-p :axis :y))
     (setq jal0 (instance rotational-joint :init :parent-link (cadr torso) :child-link (elt larm 0) :name :larm-shoulder-p :axis :y :min -150 :max 150))
     (setq jal1 (instance rotational-joint :init :parent-link (elt larm 0) :child-link (elt larm 1) :name :larm-shoulder-r :axis :x :min -30 :max 180))
     (setq jal2 (instance rotational-joint :init :parent-link (elt larm 1) :child-link (elt larm 2) :name :larm-shoulder-y :axis :z))
     (setq jal3 (instance rotational-joint :init :parent-link (elt larm 2) :child-link (elt larm 3) :name :larm-elbow-p :axis :y :min -180 :max 0))
     (setq jal4 (instance rotational-joint :init :parent-link (elt larm 3) :child-link (elt larm 4) :name :larm-wrist-y :axis :z))
     (setq jal5 (instance rotational-joint :init :parent-link (elt larm 4) :child-link (elt larm 5) :name :larm-wrist-r :axis :x))
     (setq jal6 (instance rotational-joint :init :parent-link (elt larm 5) :child-link (elt larm 6) :name :larm-wrist-p :axis :y))

     (setq jar0 (instance rotational-joint :init :parent-link (cadr torso)  :child-link (elt rarm 0) :name :rarm-shoulder-p :axis :y :min -150 :max 150))
     (setq jar1 (instance rotational-joint :init :parent-link (elt rarm 0) :child-link (elt rarm 1) :name :rarm-shoulder-r :axis :-x :min -30 :max 180))
     (setq jar2 (instance rotational-joint :init :parent-link (elt rarm 1) :child-link (elt rarm 2) :name :rarm-shoulder-y :axis :-z))
     (setq jar3 (instance rotational-joint :init :parent-link (elt rarm 2) :child-link (elt rarm 3) :name :rarm-elbow-p :axis :y :min -180 :max 0))
     (setq jar4 (instance rotational-joint :init :parent-link (elt rarm 3) :child-link (elt rarm 4) :name :rarm-wrist-y :axis :-z))
     (setq jar5 (instance rotational-joint :init :parent-link (elt rarm 4) :child-link (elt rarm 5) :name :rarm-wrist-r :axis :-x))
     (setq jar6 (instance rotational-joint :init :parent-link (elt rarm 5) :child-link (elt rarm 6) :name :rarm-wrist-p :axis :y))

     (setq jll0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt lleg 0) :name :lleg-crotch-y :axis :z))
     (setq jll1 (instance rotational-joint :init :parent-link (elt lleg 0) :child-link (elt lleg 1) :name :lleg-crotch-r :axis :x))
     (setq jll2 (instance rotational-joint :init :parent-link (elt lleg 1) :child-link (elt lleg 2) :name :lleg-crotch-p :axis :y :min -120 :max 120))
     (setq jll3 (instance rotational-joint :init :parent-link (elt lleg 2) :child-link (elt lleg 3) :name :lleg-knee-p :axis :y :min 0))
     (setq jll4 (instance rotational-joint :init :parent-link (elt lleg 3) :child-link (elt lleg 4) :name :lleg-ankle-p :axis :y))
     (setq jll5 (instance rotational-joint :init :parent-link (elt lleg 4) :child-link (elt lleg 5) :name :lleg-ankle-r :axis :x))

     (setq jlr0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt rleg 0) :name :rleg-crotch-y :axis :-z))
     (setq jlr1 (instance rotational-joint :init :parent-link (elt rleg 0) :child-link (elt rleg 1) :name :rleg-crotch-r :axis :-x))
     (setq jlr2 (instance rotational-joint :init :parent-link (elt rleg 1) :child-link (elt rleg 2) :name :rleg-crotch-p :axis :y :min -120 :max 120))
     (setq jlr3 (instance rotational-joint :init :parent-link (elt rleg 2) :child-link (elt rleg 3) :name :rleg-knee-p :axis :y :min 0))
     (setq jlr4 (instance rotational-joint :init :parent-link (elt rleg 3) :child-link (elt rleg 4) :name :rleg-ankle-p :axis :y))
     (setq jlr5 (instance rotational-joint :init :parent-link (elt rleg 4) :child-link (elt rleg 5) :name :rleg-ankle-r :axis :-x))

     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (append (list aroot-link) torso head larm rarm lleg rleg))
     (setq joint-list (list jc0 jc1 jh0 jh1
                            jal0 jal1 jal2 jal3 jal4 jal5 jal6
                            jar0 jar1 jar2 jar3 jar4 jar5 jar6
                            jll0 jll1 jll2 jll3 jll4 jll5
                            jlr0 jlr1 jlr2 jlr3 jlr4 jlr5
                            ))
     ;;    These are for robot-model.
     (setq larm-root-link (car larm) rarm-root-link (car rarm)
           lleg-root-link (car lleg) rleg-root-link (car rleg)
           torso-root-link (car torso) head-root-link (car head))
     (setq collision-avoidance-links (list aroot-link (elt torso 1) (elt larm 3) (elt rarm 3)))

     ;; set max torques
     (dolist (l (list aroot-link (elt torso 1) (elt head 1)))
       (let* ((valid-bodies (remove-if #'(lambda (x)
                                           (and (> (send x :volume) 0) (< (send x :volume) 0))) ;; nan check
                                       (send l :bodies))))
         (send l :centroid
               (if (= (length valid-bodies) 1)
                   (send (car valid-bodies) :centroid)
                 (scale (/ 1.0 (reduce #'+ (mapcar #'(lambda (x) (send x :volume)) valid-bodies)))
                        (reduce #'v+ (mapcar #'(lambda (x) (scale (send x :volume) (send x :centroid))) valid-bodies)))))
         ))
     (send-all (send self :joint-list) :max-joint-torque 7.5) ;; [Nm]
     ;; set sensors
     (setq force-sensors
           (mapcar #'(lambda (l)
                       (send (send self l :end-coords :parent) :assoc
                             (make-cascoords :coords (send self l :end-coords :copy-worldcoords)
                                             :name (format nil "~A-fsensor" (string-downcase l)))))
                   (list :rarm :larm :rleg :lleg)))
     (setq imu-sensors
           (list (let ((sen (make-cascoords
                             :worldpos (send (car (send self :torso :waist-p :child-link :bodies)) :centroid)
                             :name "torso-imusensor")))
                   (send (send self :torso :waist-p :child-link) :assoc sen)
                   sen)))
     (setq cameras
           (mapcar #'(lambda (pos name)
                       (let ((sen (instance camera-model :init (make-cylinder 10 20) :name name)))
                         (send sen :rotate pi/2 :y)
                         (send sen :rotate -pi/2 :z)
                         (send sen :locate (v+ pos (send (car (send (send self :head :neck-p :child-link) :bodies)) :centroid)) :world)
                         (send (send self :head :neck-p :child-link) :assoc sen)
                         sen))
                   (list (float-vector 0 30 0) (float-vector 0 -30 0))
                   (list "left-camera" "right-camera")))
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self))
  ;; Methods to define robot links
  (:make-root-link
   (waist-length waist-weight)
   (let ((bc0 (make-cube 100 200 waist-length)))
     (send bc0 :locate (float-vector 0 0 (+ (* -0.5 waist-length) -25)))
     (send bc0 :set-color :green)
     (instance bodyset-link :init (make-cascoords) :bodies (list bc0) :name :waist :weight waist-weight)))
  (:make-torso-links
   (torso-length torso-weight)
   (let ((bc1 (make-default-robot-link 0 50 :y :torso-link0))
	 (bc2 (make-cube 100 200 torso-length)))
     (send bc1 :locate #f(0 0 -12.5))
     (send bc2 :locate (float-vector 0 0 (* 0.5 torso-length)))
     (send bc2 :set-color :green)
     (setq bc2 (instance bodyset-link :init (make-cascoords :pos #f(0 0 -12.5)) :bodies (list bc2) :name :torso-link1 :weight torso-weight))
     (send bc1 :assoc bc2)
     (list bc1 bc2)))
  (:make-head-links
   (head-weight)
   (let ((bh0 (make-default-robot-link 0 50 :y :head-link0))
	 (bh2 (make-cube 120 100 150))
	 (bh2e (make-cylinder 10 30))
	 (bh1))
     (send bh2 :locate #f(0 0 80))
     (send bh2 :set-color :green)
     (send bh2e :rotate pi/2 :y)
     (send bh2e :locate #f(60 0 70) :world)
     (send bh2e :set-color :green)
     (send bh2 :assoc bh2e)
     (setq bh1 (instance bodyset-link :init (make-cascoords) :bodies (list bh2 bh2e) :name :head-link1 :weight head-weight))
     (setq head-end-coords (make-cascoords :pos #f(60 0 100) :rpy (float-vector 0 pi/2 0)))
     (send bh1 :assoc head-end-coords)
     (send bh0 :assoc bh1)
     (list bh0 bh1)))
  (:make-arm-links
   (l/r arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
        upper-arm-weight lower-arm-weight hand-weight)
   (let ((ba1 (make-default-robot-link 0 arm-radius :y (read-from-string (format nil "~A-link0" l/r))))
	 (ba2 (make-default-robot-link 0 arm-radius :x (read-from-string (format nil "~A-link1" l/r))))
	 (ba3 (make-default-robot-link upper-arm-length arm-radius :z (read-from-string (format nil "~A-link2" l/r))))
	 (ba4 (make-default-robot-link lower-arm-length arm-radius :y (read-from-string (format nil "~A-link3" l/r))))
	 (ba5 (make-default-robot-link 0 arm-radius :z (read-from-string (format nil "~A-link4" l/r))))
	 (ba6 (make-default-robot-link 0 arm-radius :x (read-from-string (format nil "~A-link5" l/r))))
	 (ba7 (make-default-robot-link hand-length arm-radius :y (read-from-string (format nil "~A-link6" l/r)))))
     (send ba3 :weight upper-arm-weight)
     (send ba4 :weight lower-arm-weight)
     (send ba7 :weight hand-weight)
     (case l/r
       (:rarm
	(setq rarm-end-coords (make-cascoords))
	(send rarm-end-coords :locate (float-vector 0 0 (- hand-length)))
	(send rarm-end-coords :rotate pi/2 :y)
	(send ba7 :assoc rarm-end-coords))
       (:larm
	(setq larm-end-coords (make-cascoords))
	(send larm-end-coords :locate (float-vector 0 0 (- hand-length)))
	(send larm-end-coords :rotate pi/2 :y)
	(send ba7 :assoc larm-end-coords)))
     (send ba6 :assoc ba7)
     (send ba5 :assoc ba6)
     (send ba5 :translate (float-vector 0 0 (- lower-arm-length)) :world)
     (send ba4 :assoc ba5)
     (send ba4 :translate (float-vector 0 0 (- upper-arm-length)) :world)
     (send ba3 :assoc ba4)
     (send ba2 :assoc ba3)
     (send ba1 :assoc ba2)
     (list ba1 ba2 ba3 ba4 ba5 ba6 ba7)))
  (:make-leg-links
   (l/r leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
        upper-leg-weight lower-leg-weight foot-weight)
   (let* ((bl1 (make-default-robot-link 0 leg-radius :y (read-from-string (format nil "~A-link0" l/r))))
	  (bl2 (make-default-robot-link 0 leg-radius :x (read-from-string (format nil "~A-link1" l/r))))
	  (bl3 (make-default-robot-link (- upper-leg-length (/ leg-radius 2.0)) leg-radius :z (read-from-string (format nil "~A-link2" l/r))))
	  (bl4 (make-default-robot-link (- lower-leg-length (/ leg-radius 2.0)) leg-radius :y (read-from-string (format nil "~A-link3" l/r))))
	  (bl5 (make-default-robot-link 0 leg-radius :x (read-from-string (format nil "~A-link4" l/r))))
	  (bl6b (make-cube foot-depth foot-width foot-thickness))
	  (bl6))
     (send bl3 :weight upper-leg-weight)
     (send bl4 :weight lower-leg-weight)
     (send bl6b :locate (float-vector foot-offset 0 (- ankle-length)))
     (send bl6b :set-color :green)
     (setq bl6 (make-default-robot-link ankle-length leg-radius :y (read-from-string (format nil "~A-link5" l/r)) (list bl6b)))
     (send bl6 :weight foot-weight)
     (case l/r
       (:rleg
	(setq rleg-end-coords (make-cascoords))
	(send rleg-end-coords :locate (float-vector 0 0 (- (+ ankle-length (/ foot-thickness 2.0)))))
	(send bl6 :assoc rleg-end-coords))
       (:lleg
	(setq lleg-end-coords (make-cascoords))
	(send lleg-end-coords :locate (float-vector 0 0 (- (+ ankle-length (/ foot-thickness 2.0)))))
	(send bl6 :assoc lleg-end-coords)))
     (send bl5 :assoc bl6)
     (send bl5 :translate (float-vector 0 0 (- lower-leg-length)) :world)
     (send bl4 :assoc bl5)
     (send bl4 :translate (float-vector 0 0 (- upper-leg-length)) :world)
     (send bl3 :assoc bl4)
     (send bl2 :assoc bl3)
     (send bl1 :assoc bl2)
     (list bl1 bl2 bl3 bl4 bl5 bl6)))
  (:reset-pose ()
    (send self :angle-vector #f(0.0 0.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0)))
  )

(defmethod sample-robot
  ;; user-defined joint
  (:torso-waist-y (&rest args) (forward-message-to jc0 args))
  (:torso-waist-p (&rest args) (forward-message-to jc1 args))
  (:head-neck-y (&rest args) (forward-message-to jh0 args))
  (:head-neck-p (&rest args) (forward-message-to jh1 args))
  (:larm-shoulder-p (&rest args) (forward-message-to jal0 args))
  (:larm-shoulder-r (&rest args) (forward-message-to jal1 args))
  (:larm-shoulder-y (&rest args) (forward-message-to jal2 args))
  (:larm-elbow-p (&rest args) (forward-message-to jal3 args))
  (:larm-wrist-y (&rest args) (forward-message-to jal4 args))
  (:larm-wrist-r (&rest args) (forward-message-to jal5 args))
  (:larm-wrist-p (&rest args) (forward-message-to jal6 args))
  (:rarm-shoulder-p (&rest args) (forward-message-to jar0 args))
  (:rarm-shoulder-r (&rest args) (forward-message-to jar1 args))
  (:rarm-shoulder-y (&rest args) (forward-message-to jar2 args))
  (:rarm-elbow-p (&rest args) (forward-message-to jar3 args))
  (:rarm-wrist-y (&rest args) (forward-message-to jar4 args))
  (:rarm-wrist-r (&rest args) (forward-message-to jar5 args))
  (:rarm-wrist-p (&rest args) (forward-message-to jar6 args))
  (:lleg-crotch-y (&rest args) (forward-message-to jll0 args))
  (:lleg-crotch-r (&rest args) (forward-message-to jll1 args))
  (:lleg-crotch-p (&rest args) (forward-message-to jll2 args))
  (:lleg-knee-p (&rest args) (forward-message-to jll3 args))
  (:lleg-ankle-p (&rest args) (forward-message-to jll4 args))
  (:lleg-ankle-r (&rest args) (forward-message-to jll5 args))
  (:rleg-crotch-y (&rest args) (forward-message-to jlr0 args))
  (:rleg-crotch-r (&rest args) (forward-message-to jlr1 args))
  (:rleg-crotch-p (&rest args) (forward-message-to jlr2 args))
  (:rleg-knee-p (&rest args) (forward-message-to jlr3 args))
  (:rleg-ankle-p (&rest args) (forward-message-to jlr4 args))
  (:rleg-ankle-r (&rest args) (forward-message-to jlr5 args))
  )
