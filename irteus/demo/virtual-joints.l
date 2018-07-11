(defun test-virtual-joint-for-rotation-axis
  (&key (limb :rarm) (robot)
        (virtual-joint-min-angle -50.0)
        (virtual-joint-max-angle 50.0))
  "Test virtual joint used for rotation-axis with min and max.
   virtual-joint-min-angle is min range for virtual joint [deg].
   virtual-joint-max-angle is max range for virtual joint [deg].
   If virtual-joint-min-angle = -180 and virtual-joint-max-angle = 180, behavior is similar to rotation-axis :z in this cae.
   If virtual-joint-min-angle = virtual-joint-max-angle = 0, behavior is similar to rotation-axis t."
  (unless (or robot (boundp '*robot*))
    (load "irteus/demo/sample-robot-model.l")
    (objects (list (setq *robot* (instance sample-robot :init)))))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* ((link-list (list (send *robot* :link-list (send *robot* limb :end-coords :parent))))
         (move-target (send *robot* limb :end-coords))
         ;; Virtual joint parameter
         ;; (car ret) is link-list including virtual link
         ;; (cadr ret) is move-target for virtual link
         (ret (append-obj-virtual-joint
               link-list (list move-target)
               :vclink-coords (send *robot* limb :end-coords :copy-worldcoords)
               :vplink (send *robot* limb :end-coords :parent)
               :joint-class rotational-joint
               :joint-args (list :min virtual-joint-min-angle
                                 :max virtual-joint-max-angle
                                 :axis :z)))
         (link-list-with-virtual-link (car ret))
         (move-target-with-virtual-joint (cadr ret))
         (target-coords
          (case limb
            (:rarm (make-coords :pos (float-vector 300 -200 500) :rpy (list (deg2rad 70) 0 0)))
            (:larm (make-coords :pos (float-vector 300 200 500) :rpy (list (deg2rad 70) 0 0)))
            (:rleg (make-coords :pos (float-vector 50 -100 0) :rpy (list (deg2rad 70) 0 0)))
            (:lleg (make-coords :pos (float-vector 50 100 0) :rpy (list (deg2rad 70) 0 0)))
            ))
         ;; Just for visualize virtual range.
         (b (make-fan-cylinder 300 50
                               :angle (deg2rad (- virtual-joint-min-angle virtual-joint-max-angle))
                               :mid-angle (deg2rad virtual-joint-min-angle))))
    (setf (get b :face-color) :blue)
    ;;(gl::transparent b 0.4)
    (send b :newcoords target-coords)
    (objects (list *robot* b))
    ;; Solve inverse kinematics with virtual joint
    (warn ";; Test virtual joint for rotation-axis (~A)~%" limb)
    (warn ";;   Virtual joint min max (~A,~A)[deg]~%" virtual-joint-min-angle virtual-joint-max-angle)
    (warn ";;   Original link-list          ~A~%" (send-all (car link-list) :name))
    (warn ";;   link-list with virtual link ~A~%" (send-all (car link-list-with-virtual-link) :name))
    (warn ";;   Original move-target           ~A~%" move-target)
    (warn ";;   move-target with virtual joint ~A~%" (car move-target-with-virtual-joint))
    (prog1
        (send *robot* :inverse-kinematics
              (list target-coords)
              :move-target move-target-with-virtual-joint
              :link-list link-list-with-virtual-link
              ;; Increase virtual joint weight
              :additional-weight-list (list (list (car (last (car link-list-with-virtual-link))) 100))
              :debug-view :no-message)
      ;; Display
      (send *irtviewer* :draw-objects :flush nil)
      (send target-coords :draw-on :flush nil :color #F(1 0 0) :size 100 :width 4)
      (send (send *robot* limb :end-coords) :draw-on :flush t :color #f(1 1 1) :size 200 :width 4)
      )))

(defun test-virtual-joint-for-rotation-axis-all
  ()
  (and
   ;; rotation-axis t
   (test-virtual-joint-for-rotation-axis
    :virtual-joint-max-angle 0 :virtual-joint-min-angle 0)
   ;; rotation-axis nil
   (test-virtual-joint-for-rotation-axis
    :virtual-joint-max-angle 180 :virtual-joint-min-angle -180)
   (every #'identity
          (mapcar #'(lambda (limb)
                      (test-virtual-joint-for-rotation-axis :limb limb))
                  (list :rarm :larm :rleg :lleg)))
   ))

(warn "(test-virtual-joint-for-rotation-axis) ;; test for virtual joint~%")
(warn "(test-virtual-joint-for-rotation-axis-all) ;; test for virtual joint for several situations~%")
