;; create camera and camera viewer
(setq *camera-model*
      (make-camera-from-param :pwidth 640 :pheight 360
                              :fx 400 :fy 400
                              :cx 319.5 :cy 179.5 :name "camtest"
                              :create-viewer t))
;; move camera
(send *camera-model* :translate #f(0 100 0) :world)
(send *camera-model* :rotate 0.25 :x :world)

;; make objects
(setq *obj1* (make-cube 100 100 100))
(send *obj1* :translate #f(-50 0 235))
(send *obj1* :set-color #f(0 0 1))
(setq *obj2* (make-cube 100 100 100))
(send *obj2* :translate #f(50 0 265))
(send *obj2* :set-color #f(1 0 0))
(setq *obj3* (make-cube 100 100 100))
(send *obj3* :translate #f(0 100 250))
(send *obj3* :set-color #f(0 1 0))
(objects (list *obj1* *obj2* *obj3* *camera-model*))


;; draw objects on camera viewer
(send *camera-model* :draw-objects (list *obj1* *obj2* *obj3*))

;; get image and point cloud
(let ((ret (send *camera-model* :get-image :with-points t :with-colors t)))
  (setq *image* (car ret))
  (setq *points* (cdr ret))
  )

;; transform origin of point cloud
(send *points* :transform (send *camera-model* :worldcoords))

(objects (list *points* *camera-model*))
