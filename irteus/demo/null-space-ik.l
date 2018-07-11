(load "sample-robot-model.l")

(defun null-space-ik
  ()
  "null space ik"
  (let ((left-target-coords (make-coords :pos #f(300 200 800)))
	(right-target-coords (make-coords :pos #f(400 -200 800)))
	(sign 1)
	(vel 0.05)
	(tm 0))
    (send *irtviewer* :title "null-space-ik")
    (unless (boundp '*robot*)
      (setq *robot* (instance sample-robot :init)))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :update-descendants)
    (setq *box* (make-cube 100 100 100 :pos #f(200 -200 500)))
    (send *box* :set-color :red)
    (objects (list *robot* *box*))
    ;;
    (do-until-key
#|
     you can use :null-space, or :additional-nspace-list
       :weight or :null-space argument (recommended for developper)

         If users want to specify weight or null-space as float-vector, use :weight or :null-space.

       :additional-weight-list or :additional-nspace-list (recommended for users)

         If users want to specify the joint to set weight or nspace, use :additional-weight-list or :additional-nspace-list.
|#

     (send *robot* :larm :inverse-kinematics left-target-coords
	   ;; :null-space (float-vector 0 (* sign vel) 0 0 0 0 0) ;; joint angle speed, in null space
	   :additional-nspace-list
	   (list (list (send *robot* :larm :shoulder-r :child-link) (* sign vel)))
	   :rotation-axis :z;; t
	   ;;:debug-view t
	   :look-at-target t)
     (format t "shoulder-r ~A~%" (send *robot* :larm :shoulder-r :joint-angle))
     (cond ((> (elt (send *robot* :larm :angle-vector) 1) 50)
	    (setq sign -1))
	   ((< (elt (send *robot* :larm :angle-vector) 1) 10)
	    (setq sign  1)))

     (send *robot* :rarm :inverse-kinematics right-target-coords
	   :collision-avoidance-link-pair (list (list (send *robot* :rarm :elbow-p :child-link) *box*))
	   :avoid-collision-joint-gain 1.0 ;; 1.0
	   :avoid-collision-null-gain  200 ;; 1.0
	   :rotation-axis :z;; t
	   :debug-view :no-message
           ;; you can set optional ik convergence check by additional-check function
           :additional-check #'(lambda ()
                                 ;; calculate :inverse-kinematics-loop until collision distance becomes large enough. (> 80mm, in this case)
                                 (let ((dist (car (pqp-collision-distance (send *robot* :rarm :elbow-p :child-link) *box*))))
                                   (> dist 80)))
	   )
     (send *box* :translate (float-vector 0 0 (* 7 (sin tm))))
     (incf tm 0.1)
     
     (x::window-main-one) ;; to process x event
     )
    ))

;;
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(null-space-ik) ;; for humanoid~%")

