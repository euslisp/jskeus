(load "sample-robot-model.l")

(defun shake-cocktail ()
  (send *irtviewer* :title "shake-cocktail")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))

  (setq *obj* (make-cylinder 20 100))
  (send *obj* :set-color #f(1 1 0))
  (send *robot* :reset-pose)
  (objects (list *robot* *obj*))

  (send *robot* :inverse-kinematics
        (list (make-coords :pos #f(400 0 0)))
        :move-target
        (list (send *robot* :larm :end-coords))
        :link-list
        (list (send *robot* :link-list
                    (send (send *robot* :larm :end-coords) :parent)
                    (car (send *robot* :larm :links))))
        :translation-axis (list t)
        :rotation-axis (list nil))

  (let* ((cnt 0.0))
    (do-until-key
     (incf cnt 0.1)
     (send *robot* :inverse-kinematics
           (list (make-coords :pos (float-vector (+ 400 (* 100 (sin cnt))) (* 50 (cos cnt)) 0))
                 #'(lambda ()
                     (send (send (send *robot* :larm :end-coords) :copy-worldcoords)
                           :translate #f(0 0 100) :local)))
           :move-target
           (list (send *robot* :larm :end-coords)
                 (send *robot* :rarm :end-coords))
           :link-list
           (list (send *robot* :link-list
                       (send (send *robot* :larm :end-coords) :parent)
                       (car (send *robot* :larm :links)))
                 (send *robot* :link-list
                       (send (send *robot* :rarm :end-coords) :parent)
                       (car (send *robot* :rarm :links))))
           :translation-axis (list :z t)
           :rotation-axis (list nil :z))
     (send *obj* :newcoords (send (send *robot* :larm :end-coords) :copy-worldcoords))
     (send *irtviewer* :draw-objects))))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(shake-cocktail) ;; generate shaking cocktail motion using functional target-coords.~%")
