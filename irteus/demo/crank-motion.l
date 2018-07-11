(load "sample-robot-model.l")

(defclass sample-crank
  :super cascaded-link
  :slots (handle crank-joint)
  )

(defmethod sample-crank
  (:init
    (&rest args
     &key (root-bar-height 550) (handle-base-radius 100) (handle-bar-height 40)
     &allow-other-keys)
    (send-super* :init args)
    (setq handle nil)
    ;; 1. make links links and assoc all links
    (let ((rl (send self :make-root-link root-bar-height))
          (cl (send self :make-crank-link handle-base-radius handle-bar-height)))
      (send cl :translate (float-vector 0 0 root-bar-height) :world)
      ;; 2. assoc links
      ;;    Root link should be associated with "self".
      (send self :assoc rl)
      (send rl :assoc cl)
      ;; 3. make all joints
      ;;    Before making joints, you should :assoc all links.
      (setq crank-joint (instance rotational-joint :init
                                  :parent-link rl :child-link cl
                                  :name :crank-joint :axis :z
                                  :min *-inf* :max *inf*))
      ;; 4. define slots for robot class
      ;;    links and joint-list for cascaded-link.
      (setq links (list rl cl))
      (setq joint-list (list crank-joint))
      ;; 5. call :init-ending after defining links and joint-list and return "self"
      (send self :init-ending)
      self))
  ;; Methods to define robot links
  (:make-root-link
    (root-bar-height)
    (instance bodyset-link :init (make-cascoords)
              :bodies (list (make-cylinder 10 root-bar-height))
              :name :crank-root-link))
  (:make-crank-link
    (handle-base-radius handle-bar-height)
    (let* ((handle-base-thickness 10)
           (handle-bar-radius 15)
           (handle-base (make-cube (* handle-bar-radius 2) handle-base-radius handle-base-thickness))
           (handle-bar (make-cylinder handle-bar-radius handle-bar-height)))
      (send handle-bar :translate
            (float-vector 0 (- handle-bar-radius handle-base-radius) (/ handle-base-thickness 2.0)))
      (send handle-base :translate (float-vector 0 (/ handle-base-radius -2.0) 0))
      (send handle-base :assoc handle-bar)
      (send handle-base :translate (float-vector 0 0 (/ handle-base-thickness 2.0)))
      (let* ((br (instance bodyset-link :init (make-cascoords)
                           :bodies (list handle-base handle-bar) :name :crank-handle-link))
             (ahandle
              (make-cascoords :coords
                               (send (send handle-base :copy-worldcoords)
                                     :translate (float-vector 0 (* -0.5 handle-base-radius) (/ handle-bar-height 2.0)))
                               :name :crank-handle)))
        (send br :assoc ahandle)
        (push ahandle handle)
        br)))
  (:handle () handle)
  (:crank-handle () (car handle))
  (:crank-joint (&rest args) (forward-message-to crank-joint args))
  )

(defun crank-motion
  ()
  "crank motion using full body ik"
  (send *irtviewer* :title "crank-motion")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (unless (some #'null (send *robot* :legs))
    (send *robot* :fix-leg-to-coords (make-coords)))
  (send *robot* :update-descendants)

  (let ((crank (instance sample-crank :init)))
    (send crank :locate #f(350 100 0) :world)
    (objects (list crank *robot*))
    (let* ((cog-target-pos
            (if (some #'null (send *robot* :legs))
                (send (car (send *robot* :links)) :worldpos)
              (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))))
           (fix-leg-coords
            (unless (some #'null (send *robot* :legs))
              (send *robot* :legs :end-coords :copy-worldcoords)))
           ;; append legs' parameters for move-target, link-list, thre, rotation-axis, and target-coords
           ;;  all parameter list -> (list larm rleg lleg)
           (move-target (append (list (send *robot* :larm :end-coords))
                                (send *robot* :legs :end-coords)))
           (link-list (mapcar #'(lambda (l) (send *robot* :link-list (send l :parent)))
                              move-target))
           (thre (list 15 1 1))
           (rotation-axis (list nil t t))
           (fp (apply #'midpoint 0.5 (send-all fix-leg-coords :worldpos))))
      (do-until-key
       (send crank :crank-joint :joint-angle 15 :relative t)
       (let* ((target-coords (append (list (send crank :crank-handle)) fix-leg-coords)))
         (send *robot* :fullbody-inverse-kinematics target-coords
               :move-target move-target :link-list link-list
               :rotation-axis rotation-axis :thre thre
               :look-at-target t :centroid-thre 10.0
               :dump-command nil)
         ;; draw
         (send *irtviewer* :draw-objects :flush nil)
         (send (send *robot* :support-polygon '(:rleg :lleg)) :draw-on :flush nil :width 4)
         (mapcar #'(lambda (act ref)
                     (send act :draw-on :flush nil :size 100)
                     (send ref :draw-on :flush nil :color #f(1 0 0)))
                 (append (list (let ((ac (send *robot* :centroid)))
                                 (setf (elt ac 2) 0) ac))
                         (send-all move-target :worldpos))
                 (append (list cog-target-pos) target-coords))
         (send *robot* :draw-torque *viewer*)
         (send *irtviewer* :flush)
         (x::window-main-one)
         ))
      )))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(crank-motion) ;; for fullbody motion~%")
