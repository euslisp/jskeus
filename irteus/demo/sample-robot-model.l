
(defclass sample-robot
  :super robot-model
  :slots ())
(defmethod sample-robot
  (:init 
   (&rest args)
   (let (c bal1 bal2 bal3 bal4 bal5 bal6 bal7
	   bar1 bar2 bar3 bar4 bar5 bar6 bar7
	   bll1 bll2 bll3 bll4 bll5 bll6 bll6b
	   blr1 blr2 blr3 blr4 blr5 blr6 blr6b
	   jal0 jal1 jal2 jal3 jal4 jal5 jal6
	   jar0 jar1 jar2 jar3 jar4 jar5 jar6
	   jll0 jll1 jll2 jll3 jll4 jll5
	   jlr0 jlr1 jlr2 jlr3 jlr4 jlr5
	   bc0 bc1 bc2 jc0 jc1
	   bh0 bh1 bh2 jh0 jh1
	   bh2e
	   )
     (send-super* :init args)
     (setq c (make-coords :pos #f(0 100 0) :rpy (float-vector pi/2 pi/2 0)))

     (setq bc0 (make-cube 100 200 100))
     (send bc0 :locate #f(0 0 -75))
     (send bc0 :set-color :green)
     (setq bc0 (instance bodyset-link :init (make-cascoords) :bodies (list bc0) :name :waist))
     (setq bc1 (make-default-robot-link 0 50 :y :torso-link0))
     (send bc1 :locate #f(0 0 -12.5))

     (setq bc2 (make-cube 100 200 200))
     (send bc2 :locate #f(0 0 100))
     (send bc2 :set-color :green)
     (setq bc2 (instance bodyset-link :init (make-cascoords :pos #f(0 0 -12.5)) :bodies (list bc2) :name :torso-link1))

     (send self :assoc bc0)
     (send bc0 :assoc bc1)
     (send bc1 :assoc bc2)
     
     (setq jc0 (instance rotational-joint :init :parent-link bc0 :child-link bc1 :name :torso-waist-y :axis :z :min -45 :max 45))
     (setq jc1 (instance rotational-joint :init :parent-link bc1 :child-link bc2 :name :torso-waist-p :axis :y))
     
     ;;
     ;;

     (setq bh0 (make-default-robot-link 0 50 :y :head-link0))

     (setq bh2 (make-cube 120 100 150))
     (send bh2 :locate #f(0 0 80))
     (send bh2 :set-color :green)
     (setq bh2e (make-cylinder 10 30))
     (send bh2e :rotate pi/2 :y)
     (send bh2e :locate #f(60 0 70) :world)
     (send bh2e :set-color :green)
     (send bh2 :assoc bh2e)
     (setq bh1 (instance bodyset-link :init (make-cascoords) :bodies (list bh2 bh2e) :name :head-link1))

     (setq head-end-coords (make-cascoords :pos #f(60 0 100) :rpy (float-vector 0 pi/2 0)))
     (send bh1 :assoc head-end-coords)
     (send bh0 :assoc bh1)
     (send bh0 :locate #f(0 0 225))
     (send bc2 :assoc bh0)
     
     (setq jh0 (instance rotational-joint :init :parent-link bc2 :child-link bh0 :name :head-neck-y :axis :z))
     (setq jh1 (instance rotational-joint :init :parent-link bh0 :child-link bh1 :name :head-neck-p :axis :y))

     ;;
     ;;

     (setq bal1 (make-default-robot-link 0 50 :y :larm-link0))
     (setq bal2 (make-default-robot-link 0 50 :x :larm-link1))
     (setq bal3 (make-default-robot-link 275 50 :z :larm-link2))
     (setq bal4 (make-default-robot-link 195 50 :y :larm-link3))
     (setq bal5 (make-default-robot-link 0 50 :z :larm-link4))
     (setq bal6 (make-default-robot-link 0 50 :x :larm-link5))
     (setq bal7 (make-default-robot-link 50 50 :y :larm-link6))
     (setq larm-end-coords (make-cascoords))
     (send larm-end-coords :locate #f(0 0 -50))
     (send larm-end-coords :rotate pi/2 :y)
     (send bal7 :assoc larm-end-coords)
     (send bal6 :assoc bal7)
     (send bal5 :assoc bal6)
     (send bal5 :translate #f(0 0 -195) :world)
     (send bal4 :assoc bal5)
     (send bal4 :translate #f(0 0 -275) :world)
     (send bal3 :assoc bal4)
     (send bal2 :assoc bal3)
     (send bal1 :assoc bal2)
     (send bal1 :translate #f(0 150 175) :world)
     (send bc2  :assoc bal1)

     (setq bar1 (make-default-robot-link 0 50 :y :rarm-link0))
     (setq bar2 (make-default-robot-link 0 50 :x :rarm-link1))
     (setq bar3 (make-default-robot-link 275 50 :z :rarm-link2))
     (setq bar4 (make-default-robot-link 195 50 :y :rarm-link3))
     (setq bar5 (make-default-robot-link 0 50 :z :rarm-link4))
     (setq bar6 (make-default-robot-link 0 50 :x :rarm-link5))
     (setq bar7 (make-default-robot-link 50 50 :y :rarm-link6))
     (setq rarm-end-coords (make-cascoords))
     (send rarm-end-coords :locate #f(0 0 -50))
     (send rarm-end-coords :rotate pi/2 :y)
     (send bar7 :assoc rarm-end-coords)
     (send bar6 :assoc bar7)
     (send bar5 :assoc bar6)
     (send bar5 :translate #f(0 0 -195) :world)
     (send bar4 :assoc bar5)
     (send bar4 :translate #f(0 0 -275) :world)
     (send bar3 :assoc bar4)
     (send bar2 :assoc bar3)
     (send bar1 :assoc bar2)
     (send bar1 :translate #f(0 -150 175) :world)
     (send bc2  :assoc bar1)

     (setq jal0 (instance rotational-joint :init :parent-link bc2  :child-link bal1 :name :larm-shoulder-p :axis :y))
     (setq jal1 (instance rotational-joint :init :parent-link bal1 :child-link bal2 :name :larm-shoulder-r :axis :x :min -30 :max 180))
     (setq jal2 (instance rotational-joint :init :parent-link bal2 :child-link bal3 :name :larm-shoulder-y :axis :z))
     (setq jal3 (instance rotational-joint :init :parent-link bal3 :child-link bal4 :name :larm-elbow-p :axis :y :min -180 :max 0))
     (setq jal4 (instance rotational-joint :init :parent-link bal4 :child-link bal5 :name :larm-wrist-y :axis :z))
     (setq jal5 (instance rotational-joint :init :parent-link bal5 :child-link bal6 :name :larm-wrist-r :axis :x))
     (setq jal6 (instance rotational-joint :init :parent-link bal6 :child-link bal7 :name :larm-wrist-p :axis :y))

     (setq jar0 (instance rotational-joint :init :parent-link bc2  :child-link bar1 :name :rarm-shoulder-p :axis :y))
     (setq jar1 (instance rotational-joint :init :parent-link bar1 :child-link bar2 :name :rarm-shoulder-r :axis :-x :min -30 :max 180))
     (setq jar2 (instance rotational-joint :init :parent-link bar2 :child-link bar3 :name :rarm-shoulder-y :axis :-z))
     (setq jar3 (instance rotational-joint :init :parent-link bar3 :child-link bar4 :name :rarm-elbow-p :axis :y :min -180 :max 0))
     (setq jar4 (instance rotational-joint :init :parent-link bar4 :child-link bar5 :name :rarm-wrist-y :axis :z))
     (setq jar5 (instance rotational-joint :init :parent-link bar5 :child-link bar6 :name :rarm-wrist-r :axis :-x))
     (setq jar6 (instance rotational-joint :init :parent-link bar6 :child-link bar7 :name :rarm-wrist-p :axis :y))
     ;;;
     ;;;
     (setq bll1 (make-default-robot-link 0 50 :y :lleg-link0))
     (setq bll2 (make-default-robot-link 0 50 :x :lleg-link1))
     (setq bll3 (make-default-robot-link 225 50 :z :lleg-link2))
     (setq bll4 (make-default-robot-link 225 50 :y :lleg-link3))
     (setq bll5 (make-default-robot-link 0 50 :x :lleg-link4))
     (setq bll6b (make-cube 200 100 25))
     (send bll6b :locate #f(50 0 -50))
     (send bll6b :set-color :green)
     (setq bll6 (make-default-robot-link 50 50 :y :lleg-link5 (list bll6b)))
     (setq lleg-end-coords (make-cascoords))
     (send lleg-end-coords :locate #f(0 0 -62.5))
     (send bll6 :assoc lleg-end-coords)
     (send bll5 :assoc bll6)
     (send bll5 :translate #f(0 0 -250) :world)
     (send bll4 :assoc bll5)
     (send bll4 :translate #f(0 0 -250) :world)
     (send bll3 :assoc bll4)
     (send bll2 :assoc bll3)
     (send bll1 :assoc bll2)
     (send bll1 :translate #f(0 75 -150) :world)
     (send bc0 :assoc bll1)

     (setq blr1 (make-default-robot-link 0 50 :y :rarm-link0))
     (setq blr2 (make-default-robot-link 0 50 :-x :rarm-link1))
     (setq blr3 (make-default-robot-link 225 50 :-z :rarm-link2))
     (setq blr4 (make-default-robot-link 225 50 :y :rarm-link3))
     (setq blr5 (make-default-robot-link 0 50 :-x :rarm-link4))
     (setq blr6b (make-cube 200 100 25))
     (send blr6b :locate #f(50 0 -50))
     (send blr6b :set-color :green)
     (setq blr6 (make-default-robot-link 50 50 :y :rarm-link5 (list blr6b)))
     (setq rleg-end-coords (make-cascoords))
     (send rleg-end-coords :locate #f(0 0 -62.5))
     (send blr6 :assoc rleg-end-coords)
     (send blr5 :assoc blr6)
     (send blr5 :translate #f(0 0 -250) :world)
     (send blr4 :assoc blr5)
     (send blr4 :translate #f(0 0 -250) :world)
     (send blr3 :assoc blr4)
     (send blr2 :assoc blr3)
     (send blr1 :assoc blr2)
     (send blr1 :translate #f(0 -75 -150) :world)
     (send bc0 :assoc blr1)

     (setq jll0 (instance rotational-joint :init :parent-link bc0 :child-link bll1 :name :lleg-crotch-y :axis :z))
     (setq jll1 (instance rotational-joint :init :parent-link bll1 :child-link bll2 :name :lleg-crotch-r :axis :x))
     (setq jll2 (instance rotational-joint :init :parent-link bll2 :child-link bll3 :name :lleg-crotch-p :axis :y))
     (setq jll3 (instance rotational-joint :init :parent-link bll3 :child-link bll4 :name :lleg-knee-p :axis :y :min 0))
     (setq jll4 (instance rotational-joint :init :parent-link bll4 :child-link bll5 :name :lleg-ankle-p :axis :x))
     (setq jll5 (instance rotational-joint :init :parent-link bll5 :child-link bll6 :name :lleg-ankle-r :axis :y))

     (setq jlr0 (instance rotational-joint :init :parent-link bc0 :child-link blr1 :name :rleg-crotch-y :axis :-z))
     (setq jlr1 (instance rotational-joint :init :parent-link blr1 :child-link blr2 :name :rleg-crotch-r :axis :-x))
     (setq jlr2 (instance rotational-joint :init :parent-link blr2 :child-link blr3 :name :rleg-crotch-p :axis :y))
     (setq jlr3 (instance rotational-joint :init :parent-link blr3 :child-link blr4 :name :rleg-kneep-p :axis :y :min 0))
     (setq jlr4 (instance rotational-joint :init :parent-link blr4 :child-link blr5 :name :rleg-ankle-p :axis :-x))
     (setq jlr5 (instance rotational-joint :init :parent-link blr5 :child-link blr6 :name :rleg-ankle-r :axis :y))

     ;;;
     ;;;
     (setq larm-root-link bal1 rarm-root-link bar1
	   lleg-root-link bll1 rleg-root-link blr1
	   torso-root-link bc1 head-root-link bh0)
     (setq larm (list bal1 bal2 bal3 bal4 bal5 bal6 bal7)
	   rarm (list bar1 bar2 bar3 bar4 bar5 bar6 bar7)
	   lleg (list bll1 bll2 bll3 bll4 bll5 bll6)
	   rleg (list blr1 blr2 blr3 blr4 blr5 blr6)
	   head (list bh0 bh1)
	   torso (list bc1 bc2))
     (dolist (l (append (list bc0) torso))
       (send l :weight 100.0))
     (dolist (l (append larm rarm lleg rleg head))
       (send l :weight 7.0))

     (setq links (list bc0 bc1 bc2 bh0 bh1
		       bal1 bal2 bal3 bal4 bal5 bal6 bal7
		       bar1 bar2 bar3 bar4 bar5 bar6 bar7
		       bll1 bll2 bll3 bll4 bll5 bll6
		       blr1 blr2 blr3 blr4 blr5 blr6
		       ))
     (setq joint-list (list jc0 jc1 jh0 jh1
			    jal0 jal1 jal2 jal3 jal4 jal5 jal6
			    jar0 jar1 jar2 jar3 jar4 jar5 jar6
			    jll0 jll1 jll2 jll3 jll4 jll5
			    jlr0 jlr1 jlr2 jlr3 jlr4 jlr5
			    ))
     (setq collision-avoidance-links (list bc0 bc2 bal4 bar4))

     (send self :init-ending)
     self))
  (:reset-pose ()
    (send self :angle-vector #f(0.0 0.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 0.0 0.0 -15.0 30.0 0.0 -15.0 0.0 0.0 -15.0 30.0 0.0 -15.0)))
  )

