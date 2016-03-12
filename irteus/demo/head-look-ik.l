(load "sample-robot-model.l")

(defun head-look-ik
  (&rest args)
  "look target by directing head"
  ;; initialize robot
  (send *irtviewer* :title "head-look-ik")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  ;; fix leg
  (if (some #'null (send *robot* :legs))
      (send *robot* :newcoords (make-coords))
    (send *robot* :fix-leg-to-coords (make-coords)))
  (send *robot* :update-descendants)
  ;; generate object model
  (setq *obj* (make-cube 100 100 100))
  (send *obj* :locate (float-vector 250 -300 700) :world)
  (send *obj* :set-color :blue)
  (objects (list *obj* *robot*))
  ;; ik codes
  (let* ((look-at-face (elt (send *obj* :faces) 2))
         (result
          (send *robot* :fullbody-inverse-kinematics
                (list #'(lambda ()
                          (orient-coords-to-axis
                           (make-coords :pos (send *obj* :worldpos)
                                        :rot (copy-object (send *robot* :head :end-coords :worldrot))) ;; orient-coords-to-axis is destructive
                           (v- (send look-at-face :normal))))
                      (send *robot* :rleg :end-coords :copy-worldcoords)
                      (send *robot* :lleg :end-coords :copy-worldcoords))
                :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:head :rleg :lleg))
                :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:head :rleg :lleg))
                :rotation-axis '(:z t t)
                :translation-axis '(:z t t)
                :debug-view :no-message
                :centroid-thre 20)))
    (send look-at-face :draw-on :flush nil :color #f(1 0 0) :width 2)
    (send *irtviewer* :draw-objects)
    ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(head-look-ik) ;; ik to look target by directing head~%")


