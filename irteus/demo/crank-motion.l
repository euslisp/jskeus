(load "sample-robot-model.l")

(defclass sample-crank
  :super cascaded-link
  :slots (handles)
  )

(defmethod sample-crank
  (:init
    (&rest args)
    (send-super* :init args)
    (setq handles nil)
    ;; 1. make links links and assoc all links
    (let ((rl (send self :make-root-link))
          (cl (send self :make-crank-link)))
      (send cl :translate #f(0 0 500) :world)
      ;; 2. assoc links
      ;;    Root link should be associated with "self".
      (send self :assoc rl)
      (send rl :assoc cl)
      ;; 3. define slots for robot class
      ;;    links and joint-list for cascaded-link.
      (setq links (list rl cl))
      (setq joint-list (list (instance rotational-joint :init
                                       :parent-link rl :child-link cl
                                       :name :crank-joint :axis :z)))
      ;; 4. call :init-ending after defining links and joint-list and return "self"
      (send self :init-ending)
      self))
  ;; Methods to define robot links
  (:make-root-link
    ()
    (instance bodyset-link :init (make-cascoords)
              :bodies (list (make-cylinder 10 500))
              :name :crank-root-link))
  (:make-crank-link
    ()
    (let* ((b0 (make-cylinder 10 50))
           (b1 (make-cylinder 10 70))
           (b2 (make-cube 30 120 10)))
      (send b2 :translate (float-vector 0 -50 55))
      (send b1 :translate (float-vector 0 -100 60))
      (send b0 :assoc b1)
      (send b0 :assoc b2)
      (let* ((br (instance bodyset-link :init (make-cascoords)
                           :bodies (list b0 b1 b2) :name :crank-handle-link))
             (ahandle
              (make-cascoords :coords
                               (send (send b1 :copy-worldcoords) :translate (float-vector 0 0 50))
                               :name :crank-handle)))
        (send br :assoc ahandle)
        (push ahandle handles)
        br)))
  (:crank-handle () (car handles))
  )

(defun crank-motion
  ()
  "crank motion using full body ik"
  (send *irtviewer* :title "crank-motion")
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (if (= (length (car (send *robot* :arms))) 6)
      (send *robot* :arms :angle-vector #f(-20 20 0 -50 10 0 0)))
  (unless (some #'null (send *robot* :legs))
    (mapcar #'(lambda (l)
                (send *robot* l :inverse-kinematics
                      (send (send *robot* l :end-coords :copy-worldcoords) :translate #f(0 0 30))))
            '(:rleg :lleg))
    (send *robot* :transform (send (apply #'midcoords 0.5 (send *robot* :legs :end-coords)) :transformation (make-coords))))
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
       (send crank :rotate (deg2rad 15) :z)
       (let* ((target-coords (append (list (send crank :crank-handle)) fix-leg-coords)))
         (send *robot* :fullbody-inverse-kinematics target-coords
               :move-target move-target :link-list link-list
               :rotation-axis rotation-axis :thre thre
               :look-at-target t :centroid-thre 10.0
               :debug-view :no-flush :dump-command nil)
         ;; draw
         (send *irtviewer* :draw-objects :flush nil)
         (mapcar #'(lambda (act ref)
                     (send act :draw-on :flush nil :size 100)
                     (send ref :draw-on :flush nil :color #f(1 0 0)))
                 (append (list (let ((ac (send (car (send *robot* :links)) :get :c-til)))
                                 (setf (elt ac 2) 0) ac))
                         (send-all move-target :worldpos))
                 (append (list cog-target-pos) target-coords))
         (send *robot* :draw-torque *viewer*)
         (send *irtviewer* :flush)
         ))
      )))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(crank-motion) for fullbody motion~%")
