(load "sample-robot-model.l")

(defun walk-motion
  (robot)
  "walking motion using preview-control, gait-generator, and fullbody ik"
  (let* ((is-small-robot
          (< (abs (elt (send (send (car (send robot :links)) :transformation (apply #'midcoords 0.5 (send robot :legs :end-coords))) :worldpos) 2)) 400))
         (default-step-height (if is-small-robot 10 50)))
  (send robot :reset-pose)
  (if is-small-robot
      (send robot :legs :move-end-pos #f(0 0 20)))
  (send robot :fix-leg-to-coords (make-coords))
  (objects (list robot))
  (warn ";; test1 ;; specify footstep-list~%")
  (let* ((test1-scale (if is-small-robot 0.25 1.0))
        (footstep-list
         (list (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(100 0 0))) :name :lleg)
               (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(200 0 0))) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(300 0 0))) :name :lleg)
               (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(400 0 0))) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(400 0 0))) :name :lleg))))
    (objects (append (list robot) footstep-list))
    ;; footstep-list is coordinates list.
    ;; This should have :l/r method or :l/r name.
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :default-step-height default-step-height)
    )
  ;;
  (warn ";; test2 ;; calc footstep-list from go pos param~%")
  (objects (list robot
                 (apply #'midcoords 0.5 (send robot :legs :end-coords))
                 (send (send (apply #'midcoords 0.5 (send robot :legs :end-coords)) :translate (if is-small-robot (float-vector 100 30 0) (float-vector 500 150 0))) :rotate (deg2rad 45) :z)))
  (if is-small-robot (send robot :gen-footstep-parameter :ratio 0.3))
  (send robot :calc-walk-pattern-from-footstep-list
        (if is-small-robot
            (send robot :go-pos-params->footstep-list
                  100 30 45) ;; x[mm] y[mm] th[deg]
          (send robot :go-pos-params->footstep-list
                500 150 45)) ;; x[mm] y[mm] th[deg]
        :debug-view :no-message
        :default-step-height default-step-height)
  ))

(defun walk-motion-single-support
  (robot)
  "walking motion example which start or end with single support phase"
  (let* ((is-small-robot
          (< (abs (elt (send (send (car (send robot :links)) :transformation (apply #'midcoords 0.5 (send robot :legs :end-coords))) :worldpos) 2)) 400))
         (default-step-height (if is-small-robot 10 50)))
  ;; At the initial posture, the COG is above the right foot.
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords))
  (send robot :move-centroid-on-foot :rleg '(:rleg :lleg))
  (objects (list robot))
  (warn ";; test1 ;; specify footstep-list~%")
  (let* ((test1-scale (if is-small-robot 0.25 1.0))
        (footstep-list
         (list (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(0 0 0))) :name :lleg)
               (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(0 0 100))) :name :rleg)
               )))
    (objects (append (list robot) footstep-list))
    ;; Start : COG is above the right foot.
    ;; End : COG is above the left foot.
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :default-step-height default-step-height
          :start-with-double-support nil
          :end-with-double-support nil
          :init-pose-function #'(lambda ()))
    )
  ))

(defun walk-motion-stair-climb
  (robot)
  "walking motion example for start climb"
  (let* ((is-small-robot
          (< (abs (elt (send (send (car (send robot :links)) :transformation (apply #'midcoords 0.5 (send robot :legs :end-coords))) :worldpos) 2)) 400))
         (default-step-height (if is-small-robot 10 50)))
  ;; At the initial posture, the COG is above the right foot.
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords))
  (send robot :legs :move-end-pos #f(0 0 50) :world)
  (send robot :fix-leg-to-coords (make-coords))
  (send robot :move-centroid-on-foot :rleg '(:rleg :lleg))
  (objects (list robot))
  (warn ";; test1 ;; upstair~%")
  (let* ((test1-scale (if is-small-robot 0.25 1.0))
         (footstep-list
          (list (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
                (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(150 0 50))) :name :lleg)
                (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(150 0 50))) :name :rleg)
                )))
    (objects (append (list robot) footstep-list))
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list
          :solve-angle-vector-args
          (list :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.0 0.0 0.5)
                :joint-args (list :absolute-p t
                                  :min (float-vector -500 -500  -1000 -20 -20 -10)
                                  :max (float-vector  500  500  1000  20  20  10))
                :cog-translation-axis t :debug-view nil)
          :default-step-height default-step-height
          :debug-view :no-message
          ;;:dt 0.01
          :init-pose-function #'(lambda ()))
    )
  ))

(defun walk-motion-for-sample-robot ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (walk-motion *robot*))

(defun walk-motion-single-support-for-sample-robot ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (walk-motion-single-support *robot*))

(defun walk-motion-stair-climb-for-sample-robot ()
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (walk-motion-stair-climb *robot*))
(warn "(walk-motion-for-sample-robot) ;; for walking motion~%")
(warn "(walk-motion-single-support-for-sample-robot) ;; for walking motion~%")
(warn "(walk-motion-stair-climb-for-sample-robot) ;; for walking motion~%")

(defun quad-walk-motion-for-sample-robot
  (gen-footstep-func &key (go-backward-over t) (rotation-axis (list t t t t)))
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  ;; initial quad pose
  (send *robot* :reset-pose)
  (send *robot* :move-coords
        (make-coords :rpy (list 0 (if go-backward-over -pi/2 pi/2) 0)
                     :pos (float-vector 0 0 300))
        (car (send *robot* :links)))
  (let ((tc
         (if go-backward-over
             (list
              (make-coords :pos #f(300 -120 0))
              (make-coords :pos #f(300 120 0))
              (make-coords :pos #f(-200 -120 0) :rpy (list 0 pi/2 0))
              (make-coords :pos #f(-200 120 0) :rpy (list 0 pi/2 0)))
           (list
            (make-coords :pos #f(-200 -120 0))
            (make-coords :pos #f(-200 120 0))
            (make-coords :pos #f(200 -120 0) :rpy (list 0 pi/2 0))
            (make-coords :pos #f(200 120 0) :rpy (list 0 pi/2 0)))))
        (ik-args
         (list :min (float-vector -1e10 -1e10 -1e10 -180 -180 -180)
               :max (float-vector  1e10  1e10  1e10  180  180  180)
               :joint-args '(:absolute-p t)
               :rotation-axis rotation-axis
               :additional-nspace-list
               (list
                (list (car (send *robot* :links))
                      #'(lambda () (send *robot* :joint-angle-limit-nspace-for-6dof :limbs '(:rarm :larm :rleg :lleg))))
                )
               ;;:min-loop 2 :cog-null-space nil
               :root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.5 0.5 0.5)
               :cog-gain 5.0 :centroid-thre 15
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
            ;;(append (list :debug-view :no-message) ik-args)
            ik-args
            :default-step-height 70)
      )))

(defun trot-walk-motion-for-sample-robot
  (&key (go-backward-over t) (rotation-axis (list t t t t)))
  (quad-walk-motion-for-sample-robot
   #'(lambda () (send *robot* :go-pos-quadruped-params->footstep-list 200 100 0 :type :trot))
   :go-backward-over go-backward-over
   :rotation-axis rotation-axis))
(warn "(trot-walk-motion-for-sample-robot) ;; for walking motion~%")

(defun crawl-walk-motion-for-sample-robot
  (&key (go-backward-over t))
  (quad-walk-motion-for-sample-robot
   #'(lambda () (send *robot* :go-pos-quadruped-params->footstep-list 50 0 0 :type :crawl))
   :go-backward-over go-backward-over))
(warn "(crawl-walk-motion-for-sample-robot) ;; for walking motion~%")

(defun walk-motion-for-robots ()
  (unless (boundp '*robots*)
    (setq *robots*
          (append
           (mapcar #'(lambda (form)
                       (load (format nil "models/~A-robot.l" form))
                       (funcall form))
                   '(h7 h6 kaz3))
           (list (progn (load (format nil "models/darwin.l")) (darwin))))))
  (mapcar #'(lambda
              (rb)
              (format t ";; ~A walking~%" (send rb :name))
              (walk-motion rb))
          *robots*)
  )
(warn "(walk-motion-for-robots) ;; for walking motion for several robot models~%")

;; preview control example
(defun test-preview-control-0
  (&key (preview-controller-class preview-controller)
        (q 1) (r 1e-6))
  "Example for preview controller in walking ZMP input."
  (let* ((dt 0.010) (max-tm 10.0)
         (pc (instance preview-control-cart-table-cog-trajectory-generator :init dt 800 :q q :r r :preview-controller-class preview-controller-class))
         (tm-list (mapcar #'(lambda (x) (* x dt))
                          (let ((i 0)) (mapcar #'(lambda (x) (incf i)) (make-list (round (/ max-tm dt)))))))
         (ref-zmp-list
          (mapcar #'(lambda (tm)
                      (cond ((< tm 2) #f(0 0 0))
                            ((< tm 4) #f(-20 20 0))
                            ((< tm 6) #f(20 -20 0))
                            (t #f(0 -20 0))))
                  tm-list))
         zmp-list cog-list ref-zmp-list2
         ret)
    (mapcar
     #'(lambda (ret tm)
         (push (elt (caddr ret) 0) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         (push (elt (cadr ret) 0) cog-list)                   ;; cog
         (push (elt (car ret) 0) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         )
     (send pc :pass-preview-controller ref-zmp-list) tm-list)
    (with-open-file
     (f "/tmp/test-preview-control-data.dat" :direction :output)
     (mapcar #'(lambda (tm zmp cog refzmp)
                 (format f "~A ~A ~A ~A~%" tm zmp cog refzmp))
             tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
     )
    (unless (or (null x::*display*) (= x::*display* 0))
      (let ((gp-command-list (list
                              "set xlabel 'Time [s]';"
                              "set ylabel 'ZMP X [mm]';"
                              "plot '/tmp/test-preview-control-data.dat' using 1:2 title 'cart zmp' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:3 title 'cog' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:4 title 'refzmp' with lines;"
                              "pause -1;")))
        (unix:system (format nil "gnuplot -e \"~A\""
                             (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
    (mapcar #'(lambda (tm zmp cog refzmp)
                (list :time tm :zmp zmp :cog cog :refzmp refzmp))
            tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
    ))

(defun test-preview-control-1
  (&key (preview-controller-class preview-controller)
        (q 1) (r 1e-6))
  "Example for preview controller in linear ZMP transition."
  (let* ((dt 0.010) (max-tm 8.0)
         (tm-list (mapcar #'(lambda (x) (* x dt))
                          (let ((i 0)) (mapcar #'(lambda (x) (incf i)) (make-list (round (/ max-tm dt)))))))
         (ref-zmp-list
          (mapcar #'(lambda (tm)
                      (cond ((< tm 2) (midpoint (/ tm 2.0) #f(100 0 0) #f(-20 20 0)))
                            ((< tm 6) (midpoint (/ (- tm 2) 4.0) #f(-20 20 0) #f(20 -20 0)))
                            (t #f(20 -20 0))))
                  tm-list))
         (pc (instance preview-control-cart-table-cog-trajectory-generator :init dt 800 :q q :r r :preview-controller-class preview-controller-class :init-xk (car ref-zmp-list)))
         zmp-list cog-list ref-zmp-list2
         ret)
    (mapcar
     #'(lambda (ret tm)
         (push (elt (caddr ret) 0) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         (push (elt (cadr ret) 0) cog-list)                   ;; cog
         (push (elt (car ret) 0) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         )
     (send pc :pass-preview-controller ref-zmp-list) tm-list)
    (with-open-file
     (f "/tmp/test-preview-control-data.dat" :direction :output)
     (mapcar #'(lambda (tm zmp cog refzmp)
                 (format f "~A ~A ~A ~A~%" tm zmp cog refzmp))
             tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
     )
    (unless (or (null x::*display*) (= x::*display* 0))
      (let ((gp-command-list (list
                              "set xlabel 'Time [s]';"
                              "set ylabel 'ZMP X [mm]';"
                              "plot '/tmp/test-preview-control-data.dat' using 1:2 title 'cart zmp' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:3 title 'cog' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:4 title 'refzmp' with lines;"
                              "pause -1;")))
        (unix:system (format nil "gnuplot -e \"~A\""
                             (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
    (mapcar #'(lambda (tm zmp cog refzmp)
                (list :time tm :zmp zmp :cog cog :refzmp refzmp))
            tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
    ))

(defun test-preview-control-2
  (&key (preview-controller-class preview-controller)
        (q 1) (r 1e-6)
        (ext-force-time 0.04) ;; [s]
        (ext-force-height 800) ;; [m]
        (ext-force-x -50)) ;; [N]
  "Example for preview controller to keep balance against for impulsive external force."
  (let* ((dt 0.010) (max-tm 5.0)
         (tm-list (mapcar #'(lambda (x) (* x dt))
                          (let ((i 0)) (mapcar #'(lambda (x) (incf i)) (make-list (round (/ max-tm dt)))))))
         (ref-zmp-list
          (mapcar #'(lambda (tm)
                      (v+
                       (float-vector (cond
                                      ((< tm 2.0) 0)
                                      ((< tm (+ ext-force-time 2.0))
                                       (* -1 (* 1e-3 ext-force-height) ext-force-x)) ;; ZMP difference according to disturbance force
                                      (t 0))
                                     0 0)
                       (float-vector 100 100 0) ;; neutoral zmp
                       ))
                  tm-list))
         (pc (instance preview-control-cart-table-cog-trajectory-generator :init dt 800 :q q :r r :preview-controller-class preview-controller-class :init-xk (car ref-zmp-list)))
         zmp-list cog-list ref-zmp-list2
         ret)
    (mapcar
     #'(lambda (ret tm)
         (push (elt (caddr ret) 0) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         (push (elt (cadr ret) 0) cog-list)                   ;; cog
         (push (elt (car ret) 0) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         )
     (send pc :pass-preview-controller ref-zmp-list) tm-list)
    (with-open-file
     (f "/tmp/test-preview-control-data.dat" :direction :output)
     (mapcar #'(lambda (tm zmp cog refzmp)
                 (format f "~A ~A ~A ~A~%" tm zmp cog refzmp))
             tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
     )
    (unless (or (null x::*display*) (= x::*display* 0))
      (let ((gp-command-list (list
                              "set xlabel 'Time [s]';"
                              "set ylabel 'ZMP X [mm]';"
                              "plot '/tmp/test-preview-control-data.dat' using 1:2 title 'cart zmp' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:3 title 'cog' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:4 title 'refzmp' with lines;"
                              "pause -1;")))
        (unix:system (format nil "gnuplot -e \"~A\""
                             (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
    (mapcar #'(lambda (tm zmp cog refzmp)
                (list :time tm :zmp zmp :cog cog :refzmp refzmp))
            tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
    ))

(defun test-preview-control-3
  (&key (preview-controller-class preview-controller)
        (q 1) (r 1e-6))
  "Example for preview controller in long walking ZMP input."
  (let* ((dt 0.010) (max-tm 20.0)
         (pc (instance preview-control-cart-table-cog-trajectory-generator :init dt 800 :q q :r r :preview-controller-class preview-controller-class))
         (tm-list (mapcar #'(lambda (x) (* x dt))
                          (let ((i 0)) (mapcar #'(lambda (x) (incf i)) (make-list (round (/ max-tm dt)))))))
         (tmp-zmp (float-vector 0 0 0))
         (step-tm 1.0)
         (ref-zmp-list
          (mapcar #'(lambda (tm)
                      (if (and (eps= (- (/ tm step-tm) (round (/ tm step-tm))) 0.0) (< tm 19.0))
                          (setq tmp-zmp (v+ tmp-zmp (float-vector 100 50 0))))
                      tmp-zmp)
                  tm-list))
         zmp-list cog-list ref-zmp-list2
         ret)
    (mapcar
     #'(lambda (ret tm)
         (push (elt (caddr ret) 0) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         (push (elt (cadr ret) 0) cog-list)                   ;; cog
         (push (elt (car ret) 0) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         )
     (send pc :pass-preview-controller ref-zmp-list) tm-list)
    (with-open-file
     (f "/tmp/test-preview-control-data.dat" :direction :output)
     (mapcar #'(lambda (tm zmp cog refzmp)
                 (format f "~A ~A ~A ~A~%" tm zmp cog refzmp))
             tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
     )
    (unless (or (null x::*display*) (= x::*display* 0))
      (let ((gp-command-list (list
                              "set xlabel 'Time [s]';"
                              "set ylabel 'ZMP X [mm]';"
                              "plot '/tmp/test-preview-control-data.dat' using 1:2 title 'cart zmp' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:3 title 'cog' with lines;"
                              "replot '/tmp/test-preview-control-data.dat' using 1:4 title 'refzmp' with lines;"
                              "pause -1;")))
        (unix:system (format nil "gnuplot -e \"~A\""
                             (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
    (mapcar #'(lambda (tm zmp cog refzmp)
                (list :time tm :zmp zmp :cog cog :refzmp refzmp))
            tm-list (reverse zmp-list) (reverse cog-list) (reverse ref-zmp-list2))
    ))

(warn "(test-preview-control-0) ;; ~A~%" (documentation 'test-preview-control-0))
(warn "(test-preview-control-1) ;; ~A~%" (documentation 'test-preview-control-1))
(warn "(test-preview-control-2) ;; ~A~%" (documentation 'test-preview-control-2))
(warn "(test-preview-control-3) ;; ~A~%" (documentation 'test-preview-control-3))

;; dynamics filter by using preview control
;;   input motion : control ZMP at 0 based on COG model
;;   output motion : control ZMP at 0 based on multi-body model
(defun test-preview-control-dynamics-filter
  (robot &key (preview-controller-class preview-controller) (cog-method :move-base-pos) (dt 0.025))
  (let ((avs))
    (objects (list robot))
    ;; generate input motion control ZMP at 0, which corresponds to COG at 0 in this case
    (send robot :reset-pose)
    (send robot :fix-leg-to-coords (make-coords))
    (dotimes (i 180)
      (send robot :arms :shoulder-p :joint-angle (+ -20 (* -45 (sin (* 6 (deg2rad i))))))
      (send robot :move-centroid-on-foot :both '(:lleg :rleg))
      (push (list :angle-vector (send robot :angle-vector)
                  :root-coords (send (car (send robot :links)) :copy-worldcoords)
                  :zmp (apply #'midpoint 0.5 (send robot :legs :end-coords :worldpos)))
            avs))
    (setq avs (reverse avs))
    ;; filtering
    (let ((data (subseq (send robot :preview-control-dynamics-filter dt avs :preview-controller-class preview-controller-class) 4)))
      (with-open-file
       (f "/tmp/test-preview-control-data-2.dat" :direction :output)
       (mapcar #'(lambda (tm ozmp-x izmp-x ocog-x icog-x ozmp-y izmp-y ocog-y icog-y)
                   (format f "~A ~A ~A ~A ~A ~A ~A ~A ~A~%" tm ozmp-x izmp-x ocog-x icog-x ozmp-y izmp-y ocog-y icog-y))
               (mapcar #'(lambda (x) (cadr (memq :time x))) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :output-zmp x)) 0)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :input-zmp x)) 0)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :output-cog x)) 0)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :input-cog x)) 0)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :output-zmp x)) 1)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :input-zmp x)) 1)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :output-cog x)) 1)) data)
               (mapcar #'(lambda (x) (elt (cadr (memq :input-cog x)) 1)) data)
               ))
      (unless (or (null x::*display*) (= x::*display* 0))
        (let ((gp-command-list (list
                                "set xlabel 'Time [s]';"
                                "set ylabel 'ZMP X [mm]';"
                                "plot '/tmp/test-preview-control-data-2.dat' using 1:2 title 'output zmp' with lines;"
                                "replot '/tmp/test-preview-control-data-2.dat' using 1:3 title 'input zmp' with lines;"
                                "replot '/tmp/test-preview-control-data-2.dat' using 1:4 title 'output cog' with lines;"
                                "replot '/tmp/test-preview-control-data-2.dat' using 1:5 title 'input cog' with lines;"
                                "pause -1;")))
          (unix:system (format nil "gnuplot -e \"~A\""
                               (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
      data
      )))

(defun test-preview-control-dynamics-filter-for-sample-robot
  (&key (preview-controller-class preview-controller))
  (unless (boundp '*robot*)
    (setq *robot* (instance sample-robot :init)))
  (test-preview-control-dynamics-filter *robot* :preview-controller-class preview-controller-class)
  )
(warn "(test-preview-control-dynamics-filter-for-sample-robot) ;; Example for dynamics filter using preview controller~%")
