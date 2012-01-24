(load "sample-robot-model.l")

(defun full-body-ik
  (&key (target (make-coords :pos (float-vector 0 0 1500)))
        (use-torso t) (use-leg))
  "full body ik"
    (send *irtviewer* :title "full-body-ik")
    (unless (boundp '*robot*)
      (setq *robot* (instance sample-robot :init)))
    (send *robot* :reset-pose)
    (if (= (length (car (send *robot* :legs))) 6)
	(send *robot* :legs :angle-vector #f(0 0 -10 20 0 -10)))
    (if (send *robot* :lleg)
        (send *robot* :newcoords (send (make-coords) :transform (send (send *robot* :lleg :end-coords) :transformation *robot*))) ;fix-leg
      (send *robot* :newcoords (make-coords)))
    (send *robot* :update-descendants)
    (let* ((move-target (send *robot* :larm :end-coords))
           (link-list (send *robot* :link-list
                            (send move-target :parent)
                            (cond
                             (use-leg (send *robot* :lleg :end-coords :parent))
                             (use-torso (send *robot* :torso :root-link))
                             (t (send *robot* :larm :root-link))))))
      (objects (list *robot*))
      ;;
      (do-until-key
       (let ((dif-pos (send move-target :difference-position target
                            :translation-axis t))
             (dif-rot (send move-target :difference-rotation target
                            :rotation-axis nil)))
         (send *robot* :inverse-kinematics-loop dif-pos dif-rot
               :link-list link-list
               :target-coords target
               :move-target (send *robot* :larm :end-coords)
               :rotation-axis nil :translation-axis t
               :debug-view :no-flush))
       (send *robot* :head :look-at
             (send *robot* :larm :end-coords :worldpos))
       (if (send *robot* :lleg)
           (send *robot* :newcoords (send (make-coords) :transform (send (send *robot* :lleg :end-coords) :transformation *robot*))) ;fix-leg
	 (send *robot* :newcoords (make-coords)))
       (if use-leg (send *irtviewer* :draw-objects :flush nil))
       (send *irtviewer* :viewer :viewsurface :color #f(1 1 1))
       (send *irtviewer* :viewer :viewsurface :line-width 2)
       (send *irtviewer* :viewer :viewsurface :3d-line
             (send *robot* :larm :end-coords :worldpos)
             (send *robot* :rarm :end-coords :worldpos))
       (send *irtviewer* :flush)
       )
      ))

;;
(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(full-body-ik) for humanoid~%")

