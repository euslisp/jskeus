(load "sample-robot-model.l")

(defun crank-motion
  ()
  "crank motion using full body ik"
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
     (let* ((move-target (send *robot* :larm :end-coords))
	    (cog-target-pos
	     (if (some #'null (send *robot* :legs))
		 (send (car (send *robot* :links)) :worldpos)
	       (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))))
	    (fix-leg-coords
	     (unless (some #'null (send *robot* :legs))
		 (send *robot* :legs :end-coords :copy-worldcoords)))
	    (link-list (send *robot* :link-list (send move-target :parent)))
	    (crank (make-crank)))
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
              (fp (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))))
          (dotimes (i 3) (setf (elt weight i) i) 0.01)
          (do-until-key
           (send crank :rotate (deg2rad 15) :z)
           (let* ((target (send crank :get :handle)))
             (send *robot* :inverse-kinematics target
                   :link-list (car ll) :move-target move-target
                   :look-at-target t :thre 15
                   :rotation-axis nil :translation-axis t
                   :weight weight
                   :null-space ;; for balancing
                   #'(lambda ()
                       (send *robot* :cog-jacobian-balance-nspace (car ll)
                             :target-centroid-pos cog-target-pos :cog-gain 1.5))
                   :additional-check
                   #'(lambda ()
                       (> 10.0
                          (norm
                           (subseq (v- fp (send (car (send *robot* :links)) :get :c-til)) 0 2))))
                   :debug-view :no-flush)
             ;; leg inverse kinematics
             (unless (some #'null (send *robot* :legs))
               (mapcar #'(lambda (l tc)
                           (send *robot* :inverse-kinematics tc
                                 :move-target (send *robot* l :end-coords)
                                 :link-list (send *robot* l)))
                       '(:lleg :rleg) fix-leg-coords))
             ;; draw
             (send *irtviewer* :draw-objects :flush nil)
             (mapcar #'(lambda (act ref)
                         (send act :draw-on :flush nil :size 100)
                         (send ref :draw-on :flush nil :color #f(1 0 0)))
                     (list (let ((ac (send (car (send *robot* :links)) :get :c-til)))
                             (setf (elt ac 2) 0) ac)
                           (send move-target :worldpos))
                     (list cog-target-pos target))
             (send *irtviewer* :flush)
             )))
	))
     ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(crank-motion) for fullbody motion~%")
