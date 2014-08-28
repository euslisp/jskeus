;;;
;;; add sample arm robot which we can configure links and joints parameters
;;;
(defclass sample-multidof-arm-robot
  :super robot-model
  :slots ()
  )

(defmethod sample-multidof-arm-robot
  (:init
   (&key (axis-order (list :z :y :x))
         (joint-class rotational-joint)
         (arm-length 50.0)
         (joint-class-list (make-list (length axis-order) :initial-element joint-class))
         (arm-length-list (make-list (1+ (length axis-order)) :initial-element arm-length)))
   (send-super :init)
   (setq links
         (mapcar
          #'(lambda (al)
              (let ((b (make-cube 10 10 al)))
                (send b :set-color #F(0 1 0))
                (send b :translate (float-vector 0 0 (* 0.5 al)))
                (let ((bb
                       (instance bodyset-link :init
                                 (make-cascoords)
                                 :weight 1000
                                 :centroid (float-vector 0 0 (* 0.5 al))
                                 :inertia-tensor (scale-matrix 1000 (unit-matrix 3))
                                 :bodies (list b))))
                  bb)))
          arm-length-list))
   (let ((tpos 0))
     (dotimes (i (length links))
       (send (elt links i) :locate (float-vector 0 0 tpos) :world)
       (setq tpos (+ tpos (elt arm-length-list i))))
     (setq rarm-end-coords (make-cascoords :pos (float-vector 0 0 tpos))))
   (setq rarm links)
   (send (car (last links)) :assoc rarm-end-coords)
   (send self :assoc (car links))
   (setq joint-list
         (mapcar
          #'(lambda
              (pl cl ax jc)
              (send pl :assoc cl)
              (instance jc :init
                        :parent-link pl
                        :child-link cl
                        :max-joint-torque 10
                        :axis ax))
          (butlast links) (cdr links) axis-order joint-class-list))
   (mapcar #'(lambda (x) (send x :name (format nil "link~A" (position x links)))) links)
   (mapcar #'(lambda (x) (send x :name (format nil "joint~A" (position x joint-list)))) joint-list)
   (dolist (j (send self :joint-list))
     (eval `(defmethod ,(send (class self) :name)
              (,(read-from-string (format nil ":~A" (send j :name)))
               () (elt joint-list ,(position j joint-list))))))
   (send-super :init-ending)
   self)
  )
