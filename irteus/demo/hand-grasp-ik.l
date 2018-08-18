(load "sample-hand-model.l")

(defun link-descendants (l &optional (r (list l)) rr)
  (let ((cl (send l :child-links))) 
    (unless cl
      (push r rr)
      (return-from link-descendants rr))
    (dolist (ll cl)
      (setq r (append r (list ll)))
      (setq rr (link-descendants ll r rr)))
    rr))

(defun hand-grasp nil
  "hand grasp ik"
  (send *irtviewer* :title "hand-grasp-ik")
  (let (i r k ln (av0 #f(10 45 10 45 -0 -60 40)) av1 av
          move-target target-coords link-list)
    (setq h0 (instance sample-hand :init :name "sample hand"))
    (send h0 :angle-vector av0)
    (setq c0 (make-gdome (make-icosahedron 40)))
    (send c0 :locate #f(60 0 60))
    (send c0 :set-color :green)
    (objects (list h0 c0))
    (setq i 0)
    (setq move-target (mapcar #'(lambda (x) (send h0 x)) '(:thumb-end-coords :index-end-coords :middle-end-coords))
          target-coords (mapcar #'(lambda (x) (send h0 x)) '(:index-end-coords :thumb-end-coords :thumb-end-coords))
          link-list (mapcar #'(lambda (mt) (send h0 :link-list (send mt :parent))) move-target))
    (do-until-key
     (when (= (mod i 800) 0)
       (setq av0 (send h0 :angle-vector))
       (dotimes (i 10)
         (let ((dif-pos (mapcar #'(lambda (x y z)
                                    (send x :difference-position y
                                          :translation-axis z))
                                move-target target-coords '(t t t)))
               (dif-rot (mapcar #'(lambda (x y z)
                                    (send x :difference-rotation y
                                          :rotation-axis z))
                                move-target target-coords '(nil nil nil))))
           (send h0 :inverse-kinematics-loop dif-pos dif-rot
                 :move-target move-target
                 :link-list link-list
                 :target-coords target-coords
                 :collision-avoidance-link-pair nil
                 :rotation-axis '(nil nil nil) :translation-axis '(t t t)
                 :thre '(1 1 1) :rthre '(0.1 0.1 0.1))))
       (setq av1 (send h0 :angle-vector))
       (send *irtviewer* :draw-objects :flush t)
       (x::window-main-one)
       (send h0 :angle-vector av0)
       (setq av (scale 0.01 (v- av1 av0)))
       )
     (setq ln nil)
     (dotimes (k (length (send h0 :joint-list)))
       (setq j (elt (send h0 :joint-list) k))
       (setq r
             (sort
              (mapcar #'(lambda (l)
                          (pqp-collision-distance l c0))
                      (flatten (link-descendants (send j :child-link))))
              #'<= #'car))
       (if (evenp (/ i 400))
           (if (> (caar r) 5)
               (send j :joint-angle (elt av k) :relative t))
         (send j :joint-angle -0.1 :relative t))
       (push (cdar r) ln)
       )
     (mapcar #'(lambda (l) (warn "~7,3f " (apply #'distance l))) ln)
     (warn "~%")
     (send *irtviewer* :draw-objects :flush nil)
     (send *irtviewer* :viewer :viewsurface :color #f(1 1 1))
     (send *irtviewer* :viewer :viewsurface :line-width 4)
     (dolist (l ln)
       (send* *irtviewer* :viewer :draw-line l))
     (send *irtviewer* :flush)
     (x::window-main-one)
     (incf i)
     )
    ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(hand-grasp) ;; for hand model~%")
