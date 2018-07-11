;; scene.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

;; load model files
(load "models/karimoku-1200-desk-object.l")
(load "models/petbottle-object.l")
(load "models/sushi-cup-object.l")
(load "models/foldable-desk-object.l")

;; define scene class
(defclass sample-room-scene
  :super scene-model
  :slots ())
(defmethod sample-room-scene
  (:init
   (&rest args &key (name "sample-room"))
   (let ()
     (send-super
      :init :name name :objects
      (list
       (send (karimoku-1200-desk) :transform
             (make-coords :pos (float-vector 1000 500 0)))
       (send (foldable-desk) :transform
             (make-coords :pos (float-vector 1000 -500 0)))
       (send (sushi-cup) :transform
             (make-coords :pos (float-vector 850 200 700)))
       (make-cascoords :name "table-front" :pos (float-vector 0 500 0))
       )))))

(defun sample-room () (setq *sample-room* (instance sample-room-scene :init)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sample usage
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; instantiate scene
(sample-room)
(objects (list *sample-room*))

;; show all objects in scene
(send *sample-room* :objects)

;; find object
(setq *cup* (send *sample-room* :object "sushi-cup"))
;; you can transform object
(send *cup* :translate #f(100 0 0))
;; :object method returns nil if object specified is not found
(send *sample-room* :object "object-not-found") ;; => nil

;; add object to scene
(setq *petbottle* (petbottle))
(send *petbottle* :name "irohasu")
(send *petbottle* :translate #f(850 300 700))
(send *sample-room* :add-object *petbottle*)
(objects (list *sample-room*))

;; retrieve object added to scene
(setq obj (send *sample-room* :object "irohasu"))
(equal obj *petbottle*) ;; => t

;; remove object from scene
(send *sample-room* :remove-object "irohasu") ;; remove by name
(send *sample-room* :remove-object *cup*) ;; remove by object
(objects (list *sample-room*))

;; add multiple objects
(send *sample-room* :add-objects (list *cup* *petbottle*))
;; remove multiple objects
(send *sample-room* :remove-objects (list *cup* *petbottle*))

;; you can also use methods for spot
(send *sample-room* :spots)
(setq *table-front* (send *sample-room* :spot "table-front"))
(send *sample-room* :remove-spot *table-front*)
(send *sample-room* :add-spot *table-front*)
