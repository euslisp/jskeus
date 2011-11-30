(load "sample-robot-model.l")

(defun crank-motion
  ()
  "crank motion using full body ik"
  (send *irtviewer* :title "crank-motion")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (if (= (length (car (send *robot* :arms))) 6)
      (send *robot* :arms :angle-vector #f(-20 20 0 -50 10 0 0)))
  (unless (some #'null (send *robot* :legs))
    (mapcar #'(lambda (l)
                (send *robot* l :inverse-kinematics
                      (send (send *robot* l :end-coords :copy-worldcoords) :translate #f(0 0 30))))
            '(:rleg :lleg))
    (send *robot* :transform (send (apply #'midcoords 0.5 (send *robot* :legs :end-coords)) :transformation (make-coords))))
  (send *robot* :update-descendants)

  (labels ((make-crank ()
             (let* ((b0 (make-cylinder 10 50))
		    (b1 (make-cylinder 10 70))
		    (b2 (make-cube 30 120 10)))
	       (send b2 :translate (float-vector 0 -50 55))
	       (send b1 :translate (float-vector 0 -100 60))
	       (send b0 :assoc b1) (send b0 :assoc b2)
	       (let ((br (instance bodyset :init (make-cascoords)
				   :bodies (list b0 b1 b2))))
		 (send br :put :handle
		       (make-cascoords :coords
		            (send (send b1 :copy-worldcoords) :translate (float-vector 0 0 50))))
		 (send br :assoc (send br :get :handle))
		 br))))
     (let* ((cog-target-pos
	     (if (some #'null (send *robot* :legs))
		 (send (car (send *robot* :links)) :worldpos)
	       (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))))
	    (fix-leg-coords
	     (unless (some #'null (send *robot* :legs))
		 (send *robot* :legs :end-coords :copy-worldcoords)))
	    (crank (make-crank))
	    ;; append legs' parameters for move-target, link-list, thre, rotation-axis, and target-coords
	    ;;  all parameter list -> (list larm rleg lleg)
	    (move-target (append (list (send *robot* :larm :end-coords))
				 (send *robot* :legs :end-coords)))
	    (link-list (mapcar #'(lambda (l) (send *robot* :link-list (send l :parent)))
			       move-target))
	    (thre (list 15 1 1))
	    (rotation-axis (list nil t t)))
       (send crank :locate #f(350 100 500) :world)
       (objects (list crank *robot*))
       ;;
       (with-append-root-joint
	(ll *robot* link-list
	    :joint-class 6dof-joint
	    :joint-args
	    (list :min (float-vector -300 -300 -25 -10 -15 -15)
		  :max (float-vector  300  300  25  10  15  15)))
        (let ((weight (fill (instantiate float-vector (send *robot* :calc-target-joint-dimension ll)) 1.0))
	      (6dof-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5))
              (fp (apply #'midpoint 0.5 (send-all fix-leg-coords :worldpos))))
          (dotimes (i 6) (setf (elt weight i) (elt 6dof-weight i)))
          (do-until-key
           (send crank :rotate (deg2rad 15) :z)
           (let* ((target-coords (append (list (send crank :get :handle)) fix-leg-coords)))
             (send *robot* :inverse-kinematics target-coords
                   :link-list ll :move-target move-target
                   :look-at-target t :thre thre :rotation-axis rotation-axis
                   :weight weight
                   :null-space ;; for balancing
                   #'(lambda ()
                       (send *robot* :cog-jacobian-balance-nspace (send *robot* :calc-union-link-list ll)
                             :target-centroid-pos cog-target-pos :cog-gain 1.5))
                   :additional-check
                   #'(lambda ()
                       (> 10.0
                          (norm
                           (subseq (v- fp (send (car (send *robot* :links)) :get :c-til)) 0 2))))
                   :debug-view :no-flush)
             ;; draw
             (send *irtviewer* :draw-objects :flush nil)
             (mapcar #'(lambda (act ref)
                         (send act :draw-on :flush nil :size 100)
                         (send ref :draw-on :flush nil :color #f(1 0 0)))
                     (append (list (let ((ac (send (car (send *robot* :links)) :get :c-til)))
				     (setf (elt ac 2) 0) ac))
			     (send-all move-target :worldpos))
                     (append (list cog-target-pos) target-coords))
             (send *irtviewer* :flush)
             )))
	))
     ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(crank-motion) for fullbody motion~%")
