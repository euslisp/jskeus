(load "sample-robot-model.l")

(defun dual-arm-ik nil
  "dual arm ik"
  (send *irtviewer* :title "dual-arm-ik")
  (let ((i 0) link-list move-target target-coords b0)
    (labels
     ((make-sweep
       nil
       (let ((sh 250) (sw 125) (length 700) sweep bar br)
         (setq bar (make-cylinder 10 length))
         (setq sweep (make-prism (list (float-vector sw (- sh) 0)
                                       (float-vector (- sw) (- sh) 0)
                                       (float-vector -20 0 0)
                                       (float-vector  20 0 0)) 20))
         (send bar :locate (float-vector 0 0 sh) :world)
         (send sweep :rotate pi/2 :x)
         (send sweep :locate (float-vector 0 10 sh) :world)
         (send bar :set-color :brown)
         (send sweep :set-color :red)
         (setq br (instance bodyset :init (make-cascoords)
                            :bodies (list bar sweep)))
         (dolist (rate (list 0.8 0.6))
           (push (make-cascoords :pos (float-vector 0 0 (+ (* rate length) sh))) target-coords))
         (dolist (tc (append target-coords (list bar sweep)))
           (send br :assoc tc))
         br)))
     ;;
     (unless (boundp '*robot*)
       (setq *robot* (instance sample-robot :init)))
     (send *robot* :reset-pose)
     (if (= (length (car (send *robot* :arms))) 7)
	 (send *robot* :arms :angle-vector #f(10 20 0 -20 10 0 0)))
     (if (some #'null (send *robot* :legs))
	 (send *robot* :newcoords (make-coords))
       (send *robot* :transform (send (apply #'midcoords 0.5 (send *robot* :legs :end-coords)) :transformation (make-coords))))
     (send *robot* :update-descendants)
     ;;
     ;; make broom model
     (setq b0 (make-sweep))
     (send b0 :locate #f(250 0 0))
     ;;
     ;; setup move-target and link-list
     (setq move-target (send *robot* :arms :end-coords)
           link-list (mapcar #'(lambda (mt) (send *robot* :link-list mt)) (send-all move-target :parent)))
     ;;
     ;; look-at
     (send *robot* :head :look-at (apply #'midpoint 0.5 (send-all target-coords :worldpos)))
     (objects (list *robot* b0))
     ;;
     ;; do sweep
     (do-until-key
      (send *robot* :inverse-kinematics target-coords
            :link-list link-list :move-target move-target
            :stop 500 :thre '(10 10)
            :rotation-axis '(nil nil) :debug-view nil :dump-command nil)
      (send *robot* :head :look-at
            (apply #'midpoint 0.5 (send-all target-coords :worldpos)))
      (send b0 :orient (* 0.2 (sin (/ i 10.0))) :x :world)
      (send b0 :locate (float-vector 250 (* 250 (sin (/ (incf i) 10.0))) 0) :world)
      (send *irtviewer* :draw-objects)
      (incf i)
      ))
    ))
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(dual-arm-ik) for tool usage~%")
