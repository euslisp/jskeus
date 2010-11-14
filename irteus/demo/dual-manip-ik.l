(load "sample-robot-model.l")

(defun dual-manip-ik
  (&rest args)
  "dual-armed object manipulation"
  ;; initialize robot
  (send *irtviewer* :title "dual-manip-ik")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :newcoords (make-coords))
  (send *robot* :reset-pose)
  ;; fix leg
  (if (some #'null (send *robot* :legs))
      (send *robot* :newcoords (make-coords))
    (send *robot* :transform (send (apply #'midcoords 0.5 (send *robot* :legs :end-coords)) :transformation (make-coords))))
  (send *robot* :update-descendants)
  ;; generate object model
  (setq *obj* (make-cube 30 160 100))
  (dolist (l '(:rarm :larm))
    (send *obj* :put l ;; generate target coords
          (make-cascoords :coords
                          (send (send (send *obj* :copy-worldcoords) :rotate (case l (:rarm pi/2) (:larm -pi/2)) :z)
                                :translate (float-vector -75 0 0))))
    (send *obj* :assoc (send *obj* :get l)))
  (send *obj* :newcoords (make-coords :pos #f(300 0 700)))
  (send *obj* :put :face-color :blue)
  (send *robot* :head :look-at (send *obj* :worldpos))
  (objects (list *obj* *robot*))
  ;; ik codes
  (let ((i 0))
    (do-until-key
     ;; rotate target coords
     (when (/= i 0)
       (let ((l (cond
                 ((or (= (mod i 4) 1) (= (mod i 4) 2)) :rarm)
                 ((or (= (mod i 4) 3) (= (mod i 4) 0)) :larm)))
             (rot (if (oddp (mod i 4)) (* (case (mod i 4) (1 1) (3 -1)) (deg2rad 90))))
             (trs (float-vector (* (expt -1 (mod i 2)) 50) 0 0)))
         (send *obj* :dissoc (send *obj* :get l))
         (send *obj* :assoc (send *obj* :get l))
         (if (oddp (mod i 4))
             (send (send *obj* :get l) :rotate rot :x))
         (send (send *obj* :get l) :translate trs)))
     ;; solve inverse kinematics
     (with-move-target-link-list
      (mt robot-ll *robot* '(:rarm :larm))
      (let* ((ret
              (append-obj-virtual-joint
               robot-ll (mapcar #'(lambda (l) (send *obj* :get l)) '(:rarm :larm))
               :joint-class 6dof-joint))
             (ll (car ret)) (ot (cadr ret))
             (ul (apply #'append
                        (mapcar
                         #'(lambda (f)
                             (send *robot* :calc-union-link-list
                                   (mapcar #'(lambda (l)
                                               (funcall f #'(lambda (x) (member x (send *robot* :links))) l))
                                           ll)))
                         (list #'remove-if-not #'remove-if))))
             (rotation-axis (cond
                             ((= (mod i 4) 1) '(:x t))
                             ((= (mod i 4) 3) '(t :x))
                             (t '(t t))))
             (weight (fill (instantiate float-vector (send *robot* :calc-target-joint-dimension ll)) 1.0)))
        (dotimes (i 3)
          (setf (elt weight (+ (- (send *robot* :calc-target-joint-dimension ll) 6) i)) 0.001))
        (send (car (last ul)) :assoc *obj*)
        (send* *robot* :inverse-kinematics ot
               :union-link-list ul :link-list ll :move-target mt
               :weight weight
               :rotation-axis rotation-axis
               :jacobi #'(lambda ()
                           (calc-jacobian-from-link-list-including-robot-and-obj-virtual-joint
                            ll mt ot *robot* :rotation-axis rotation-axis))
               ;;:debug-view t
               :debug-view :no-message
               (append (if (= i 0) '(:stop 100)) args))
        (send (car (last ul)) :dissoc *obj*)
        ))
     (incf i)
     (send *robot* :head :look-at (send *obj* :worldpos))
     )
    ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(dual-manip-ik) for dual-armed object manipulation~%")


