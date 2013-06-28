(load "sample-robot-model.l")

(defun tmp-walk-motion
  (robot &key (default-step-height 50))
  "walking motion using preview-control, gait-generator, and fullbody ik"
  (send *irtviewer* :title "walk-motion")
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (objects (list robot))
  (let ((footstep-list
         (list (send robot :rleg :end-coords :copy-worldcoords)
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate #f(50 0 0))
               (send (send robot :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0))
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate #f(150 0 0))
               (send (send robot :rleg :end-coords :copy-worldcoords) :translate #f(200 0 0))
               (send (send robot :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)))))
    ;; footstep-list is coordinates list.
    ;; This should have :l/r method or :l/r property.
    (dolist (x footstep-list)
      (send x :put :l/r (if (evenp (position x footstep-list)) :rleg :lleg)))
    (warn ";; test1 ;; specify footstep-list~%")
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :default-step-height default-step-height)
    )
  ;;
  (warn ";; test2 ;; calc footstep-list from go pos param~%")
  (objects (list robot))
  (send robot :calc-walk-pattern-from-footstep-list
        (send robot :go-pos-params->footstep-list
              500 150 45) ;; x[mm] y[mm] th[rad]
        :debug-view :no-message
        :default-step-height default-step-height)
  )

(defun walk-motion ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (tmp-walk-motion *robot*))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(walk-motion) for walking motion~%")

(defun walk-motion-for-robots ()
  (unless (boundp '*robots*)
    (setq *robots*
          (mapcar #'(lambda (form)
                      (load (format nil "models/~A-robot.l" form))
                      (funcall form))
                  '(h7 h6))))
  (dolist (rb *robots*)
    (format t ";; ~A walking~%" (send rb :name))
    (let ((leg-length (abs (elt (send (send (car (send rb :links)) :transformation (apply #'midcoords 0.5 (send rb :legs :end-coords))) :worldpos) 2))))
      (tmp-walk-motion
       rb
       :default-step-height (if (> leg-length 300) 50 10)))
    ))
(warn "(walk-motion-for-robots) for walking motion for several robot models~%")