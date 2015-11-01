
;; traditional way of displaying camera image
(defun sample-get-camera-image-1 ()
  (load "irteus/demo/sample-robot-model.l")
  (setq *robot* (instance sample-robot :init))
  (setq *cam* (send *robot* :camera "left-camera"))
  (setq *obj* (make-cube 10 1000 1000 :pos #f(1500 30 350)))
  (send *obj* :set-color #f(0 0 1))
  (objects (list *robot* *obj*))

  (setq *camview*
        (view
         :x 320 :y 240
         :viewing (send *cam* :viewing)
         :viewsurface
         (instance gl::glviewsurface :create
                   :glcon ((send *irtviewer* :viewer :viewsurface) . gl::glcon)
                   :title "view_test" :width 320 :height 240)
         :title "view_test"
         ))

  (send *cam* :create-viewer *camview*)
  (send *cam* :draw-objects (list *obj*))
;  (do-until-key
;    (send (*cam* . img-viewer) :viewsurface :flush)
;    (send *cam* :draw-objects (list *obj*))
;    (x::window-main-one)
;    )
  (image::write-pnm-file "sample-get-camera-image-1.ppm" (send *cam* :get-image))

  )

(defun sample-get-camera-image-2 ()
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
  (image::write-pnm-file "sample-get-camera-image-2a.ppm" (send *camera-model* :get-image))
  (send *camera-model* :draw-objects (list *obj1* *obj2* *obj3*)) ;; need to run twise to display image, this may BUG
  (image::write-pnm-file "sample-get-camera-image-2a.ppm" (send *camera-model* :get-image))

  ;; get image and point cloud
  (let ((ret (send *camera-model* :get-image :with-points t :with-colors t)))
    (setq *image* (car ret))
    (setq *points* (cdr ret))
    )

  ;; transform origin of point cloud
  (send *points* :transform (send *camera-model* :worldcoords))

  (objects (list *points* *obj1* *obj2* *obj3* *camera-model*))
  (send *camera-model* :draw-objects (list *points* *obj1* *obj2* *obj3*))
  (send *camera-model* :draw-objects (list *points* *obj1* *obj2* *obj3*))
  (image::write-pnm-file "sample-get-camera-image-2b.ppm" (send *camera-model* :get-image))
  )


(warn ";; run demo program~%")
(warn "(sample-get-camera-image-1)~%")
(warn "(sample-get-camera-image-2)~%")
