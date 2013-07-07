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