(load "sample-arm-model.l")
(load "hanoi.l")

(defmethod cascaded-link
  (:pick
   (obj &rest args)
   (send self :open-hand)
   (prog1
       (send* self :inverse-kinematics
              obj
              :link-list (send self :link-list (send self :end-coords :parent))
              :move-target (send self :end-coords)
              args)
     (send self :close-hand obj))
   )
  (:place
   (obj &rest args)
   (prog1
       (send* self :inverse-kinematics
              obj
              :link-list (send self :link-list (send self :end-coords :parent))
              :move-target (send self :end-coords)
              args)
     (send self :open-hand)))
  )

(defun move-disk (d from to) ;; re-define
  (let (pav)
    (if (eq from 'ground)
        (move-on d (table-top to))
      (progn
        (move-arm d 'pick)
        (move-arm (get (table-top to) :top-coords) 'place)))
    (setf (get from :disks) (cdr (get from :disks)))
    (setf (get to :disks) (cons d (get to :disks)))
    ))

(defun move-arm (to pick/place)
  (let ()
    (unless
        (send *sarm* (if (eq pick/place 'pick) :pick :place)
              to :rotation-axis :z
              :obstacles
              (remove-if #'(lambda (x) (< (norm (send x :difference-position to :translation-axis :z)) 50)) *tables*)
              :stop 500
;; collision free motion generation stucks...
;              :avoid-collision-distance   200 ;; 200 
              :avoid-collision-joint-gain 2.0 ;; 1.0
              :avoid-collision-null-gain  1.0 ;; 1.0
              :debug-view :no-message
	      :dump-command nil
              )
      (break))
    (send *irtviewer* :draw-objects)))

;; sarm sample
(defun hanoi-arm nil
  (send *irtviewer* :title "hanoi-arm")
  (setq *sarm* (instance sarmclass :init))
  (send *sarm* :reset-pose)
  (objects (list *sarm*))

  (send *sarm* :reset-pose)
  (send *irtviewer* :draw-objects)
  (send *sarm* :inverse-kinematics
        (make-cascoords :pos #f(500 0 100))
        :link-list (send *sarm* :link-list (send *sarm* :end-coords :parent))
        :move-target (send *sarm* :end-coords)
        :avoid-collision-null-gain  0.0 ;; 1.0
        :rotation-axis t :debug-view nil  :dump-command nil)
  (send *sarm* :open-hand)
  (send *irtviewer* :draw-objects)

  (setq *table-a* (hanoi-table :name "table-a" :height 350 :pos #f(300 200 0)))
  (setq *table-b* (hanoi-table :name "table-b" :height 400 :pos #f(350 0 0)))
  (setq *table-c* (hanoi-table :name "table-b" :height 300 :pos #f(300 -200 0)))
  (setq *tables* (list *table-a* *table-b* *table-c*))
  (hanoi-init 3 *table-a*)
  (setq *disk-1* (car *disks*))
  (setq *disk-2* (cadr *disks*))
  (setq *disk-3* (caddr *disks*))
  (send *sarm* :reset-pose)
  (objects (flatten (list  *tables* *disks* *sarm*)))

  (dolist (action (hanoi-program (length *disks*)))
    (print action)
    (eval action)
    (unix:usleep (* 100 1000))
    ))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(hanoi-arm) ;; for arm solving hanoi tower~%")





