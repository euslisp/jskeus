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

(defun quad-walk-motion-for-sample-robot
  (gen-footstep-func &key (go-backward-over t))
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  ;; initial quad pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (send *robot* :rotate (if go-backward-over -pi/2 pi/2) :y)
  (let ((tc
         (if go-backward-over
             (list
              (make-coords :pos #f(300 -120 0))
              (make-coords :pos #f(300 120 0))
              (make-coords :pos #f(-200 -120 0) :rpy (list 0 pi/2 0))
              (make-coords :pos #f(-200 120 0) :rpy (list 0 pi/2 0)))
           (list
            (make-coords :pos #f(-300 -120 0))
            (make-coords :pos #f(-300 120 0))
            (make-coords :pos #f(200 -120 0) :rpy (list 0 pi/2 0))
            (make-coords :pos #f(200 120 0) :rpy (list 0 pi/2 0)))))
        (ik-args
         (list :min (float-vector -1000 -1000 -1000 -90 -90 -90)
               :max (float-vector  1000  1000  1000  90  90  90)
               :joint-args '(:absolute-p t)
               :additional-nspace-list
               (list
                (list (car (send *robot* :links))
                      #'(lambda () (send *robot* :joint-angle-limit-nspace-for-6dof :limbs '(:rarm :larm :rleg :lleg))))
                )
               :root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.5 0.5 0.5)
               :cog-gain 5.0 :stop 200 :centroid-thre 15
               ;;:debug-view :no-message
               :collision-avoidance-link-pair nil)))
    (with-move-target-link-list
     (mt ll *robot* '(:rleg :lleg :rarm :larm))
     (send* *robot* :fullbody-inverse-kinematics
            tc
            :move-target mt :link-list ll
            :target-centroid-pos (vector-mean (send-all tc :worldpos))
            ik-args))
    ;; prepare footsteps
    (let ((footstep-list (funcall gen-footstep-func)))
      (objects (list *robot*))
      ;; solve walk motion
      (send *robot* :calc-walk-pattern-from-footstep-list
            footstep-list :debug-view :no-message
            :all-limbs '(:rleg :lleg :rarm :larm)
            :init-pose-function
            #'(lambda ()
                (send* *robot* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm)
                       :target-centroid-pos
                       (vector-mean (append (send *robot* :arms :end-coords :worldpos) (send *robot* :legs :end-coords :worldpos)))
                       ik-args))
            :solve-angle-vector-args
            (append (list ;;:debug-view :no-message
                          :centroid-thre 100
                          :thre '(10 10 10 10)) ik-args)
            :default-zmp-offsets
            (list :rleg (float-vector 0 0 0) :lleg (float-vector 0 0 0)
                  :rarm (float-vector 0 0 0) :larm (float-vector 0 0 0))
            :default-step-height 70)
      )))

(defun trot-walk-motion-for-sample-robot
  (&key (go-backward-over t))
  (quad-walk-motion-for-sample-robot
   #'(lambda ()
       (let ((fs
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
         (dolist (x fs)
           (send (car x) :put :l/r (if (evenp (position x fs)) :rleg :lleg))
           (send (cadr x) :put :l/r (if (evenp (position x fs)) :larm :rarm)))
         fs))
   :go-backward-over go-backward-over))
(warn "(trot-walk-motion-for-sample-robot) for walking motion~%")

(defun crawl-walk-motion-for-sample-robot
  (&key (go-backward-over t))
  (quad-walk-motion-for-sample-robot
   #'(lambda ()
       (let ((fs
              (list (list (send *robot* :lleg :end-coords :copy-worldcoords))
                    (list (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world))
                    (list (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world))
                    (list (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world))
                    (list (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 50 0 0) :world))
                    )))
         (send (car (elt fs 0)) :put :l/r :lleg)
         (send (car (elt fs 1)) :put :l/r :larm)
         (send (car (elt fs 2)) :put :l/r :rarm)
         (send (car (elt fs 3)) :put :l/r :rleg)
         (send (car (elt fs 4)) :put :l/r :lleg)
         fs))
   :go-backward-over go-backward-over))
(warn "(crawl-walk-motion-for-sample-robot) for walking motion~%")

(defun walk-motion-for-robots ()
  (unless (boundp '*robots*)
    (setq *robots*
          (mapcar #'(lambda (form)
                      (load (format nil "models/~A-robot.l" form))
                      (funcall form))
                  '(h7 h6 kaz3))))
  (mapcar #'(lambda
              (rb)
              (format t ";; ~A walking~%" (send rb :name))
              (walk-motion rb))
          *robots*)
  )
(warn "(walk-motion-for-robots) for walking motion for several robot models~%")
