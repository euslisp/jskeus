(load "sample-robot-model.l")

(defun walk-motion
  (robot)
  "walking motion using preview-control, gait-generator, and fullbody ik"
  (let* ((is-small-robot
          (< (abs (elt (send (send (car (send robot :links)) :transformation (apply #'midcoords 0.5 (send robot :legs :end-coords))) :worldpos) 2)) 400))
         (default-step-height (if is-small-robot 10 50)))
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (objects (list robot))
  (warn ";; test1 ;; specify footstep-list~%")
  (let* ((test1-scale (if is-small-robot 0.25 1.0))
        (footstep-list
         (list (send robot :rleg :end-coords :copy-worldcoords)
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(100 0 0)))
               (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(200 0 0)))
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(300 0 0)))
               (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(400 0 0)))
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(400 0 0))))))
    (objects (append (list robot) footstep-list))
    ;; footstep-list is coordinates list.
    ;; This should have :l/r method or :l/r property.
    (dolist (x footstep-list)
      (send x :put :l/r (if (evenp (position x footstep-list)) :rleg :lleg)))
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :default-step-height default-step-height)
    )
  ;;
  (warn ";; test2 ;; calc footstep-list from go pos param~%")
  (objects (list robot
                 (apply #'midcoords 0.5 (send robot :legs :end-coords))
                 (send (send (apply #'midcoords 0.5 (send robot :legs :end-coords)) :translate (float-vector 500 150 0)) :rotate (deg2rad 45) :z)))
  (if is-small-robot (send robot :gen-footstep-parameter :ratio 0.3))
  (send robot :calc-walk-pattern-from-footstep-list
        (send robot :go-pos-params->footstep-list
              500 150 45) ;; x[mm] y[mm] th[rad]
        :debug-view :no-message
        :default-step-height default-step-height)
  ))

(defun walk-motion-for-sample-robot ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (walk-motion *robot*))
(warn "(walk-motion-for-sample-robot) for walking motion~%")

(defun quad-walk-motion-for-sample-robot ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  ;; initial quad pose
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords :pos #f(0 0 400) :rpy (list 0 (deg2rad -90) 0)))
  (with-move-target-link-list
   (mt ll *robot* '(:rleg :lleg :rarm :larm))
   (send *robot* :inverse-kinematics
         (list
          (make-coords :pos #f(300 -120 0))
          (make-coords :pos #f(300 120 0))
          (make-coords :pos #f(-200 -120 0) :rpy (list 0 pi/2 0))
          (make-coords :pos #f(-200 120 0) :rpy (list 0 pi/2 0)))
         :stop 200 :move-target mt :link-list ll
         ;;:debug-view :no-message
         ))
  ;; prepare footsteps
  (let ((footstep-list
         (list (list (send *robot* :rleg :end-coords :copy-worldcoords)
                     (send *robot* :larm :end-coords :copy-worldcoords))
               (list (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world)
                     (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world))
               (list (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 100 0 0) :world)
                     (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 100 0 0) :world))
               (list (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 50 0) :world)
                     (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 150 50 0) :world))
               (list (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 200 100 0) :world)
                     (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 200 100 0) :world))
               (list (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 200 100 0) :world)
                     (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 200 100 0) :world))
               )))
    (dolist (x footstep-list)
      (send (car x) :put :l/r (if (evenp (position x footstep-list)) :rleg :lleg))
      (send (cadr x) :put :l/r (if (evenp (position x footstep-list)) :larm :rarm)))
    (objects (list *robot*))
    ;; solve walk motion
    (send *robot* :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :init-pose-function
          #'(lambda ()
              (send *robot* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm)
                    ;;:debug-view :no-message
                    :stop 200 :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0 0 0)
                    :target-centroid-pos (vector-mean (append (send *robot* :arms :end-coords :worldpos) (send *robot* :legs :end-coords :worldpos)))))
          :solve-angle-vector-args (list :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0 0 0))
          :default-zmp-offsets
          (list :rleg (float-vector 0 0 0) :lleg (float-vector 0 0 0)
                :rarm (float-vector 0 0 0) :larm (float-vector 0 0 0))
          :default-step-height 70)
    ))
(warn "(quad-walk-motion-for-sample-robot) for walking motion~%")

(defun walk-motion-for-robots ()
  (unless (boundp '*robots*)
    (setq *robots*
          (mapcar #'(lambda (form)
                      (load (format nil "models/~A-robot.l" form))
                      (funcall form))
                  '(h7 h6 kaz3))))
  (dolist (rb *robots*)
    (format t ";; ~A walking~%" (send rb :name))
    (walk-motion rb)
    ))
(warn "(walk-motion-for-robots) for walking motion for several robot models~%")