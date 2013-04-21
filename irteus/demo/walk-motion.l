(load "sample-robot-model.l")

(defun walk-motion
  ()
  "walking motion using preview-control, gait-generator, and fullbody ik"
  (send *irtviewer* :title "walk-motion")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (objects (list *robot*))
  (let ((footstep-list
         (list (send *robot* :rleg :end-coords :copy-worldcoords)
               (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(50 0 0))
               (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0))
               (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(150 0 0))
               (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(200 0 0))
               (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)))))
    ;; footstep-list is coordinates list.
    ;; This should have :l/r method or :l/r property.
    (dolist (x footstep-list)
      (send x :put :l/r (if (evenp (position x footstep-list)) :rleg :lleg)))
    (warn ";; test1 ;; specify footstep-list~%")
    (send *robot* :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message)
    )
  ;;
  (warn ";; test2 ;; calc footstep-list from go pos param~%")
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (objects (list *robot*))
  (send *robot* :calc-walk-pattern-from-footstep-list
        (send *robot* :go-pos-params->footstep-list
              500 150 45) ;; x[mm] y[mm] th[rad]
        :debug-view :no-message)
  )

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(walk-motion) for walking motion~%")
