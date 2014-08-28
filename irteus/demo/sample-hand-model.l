(defclass sample-hand
  :super robot-model
  :slots (thumb-end-coords middle-end-coords index-end-coords
                           j00 j01 j10 j11 j20 j21 j22))
(defmethod sample-hand
  (:init
   (&rest args)
   (let (b0 b00 b01 b10 b11 b20 b21 b22)
     (send-super* :init args)
     (setq b0 (make-cube 40 75 114))
     (setq b0 (instance bodyset-link :init (make-cascoords) 
			:bodies (list b0)
			:name "root-link"))

     ;;koyubi
     (setq b00 (make-cylinder 11 75))
     (send b00 :set-color :gray60)
     (setq b00 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b00)
			 :name "koyubi-link-0"))
     (send b00 :locate #f(-3 -22 57))
     (setq b01 (make-cylinder 11 45))
     (send b01 :set-color :gray60)
     (setq b01 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b01)
			 :name "koyubi-link-1"))
     (send b01 :locate #f(-3 -22 132))
     (setq middle-end-coords 
       (make-cascoords :pos (float-vector -3 -22 (+ 45 132))))
     (send b01 :assoc middle-end-coords)

     ;;hitosashi
     (setq b10 (make-cylinder 11 75))
     (send b10 :set-color :gray60)
     (setq b10 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b10)
			 :name "hitosashi-link-0"))
     (send b10 :locate #f(-3  18  57))
     (setq b11 (make-cylinder 11 45))
     (send b11 :set-color :gray60)
     (setq b11 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b11)
			 :name "hitosashi-link-1"))
     (send b11 :locate #f(-3  18 132))
     (setq index-end-coords 
       (make-cascoords :pos (float-vector -3 18 (+ 45 132))))
     (send b11 :assoc index-end-coords)

     ;;oyayubi
     (setq b20 (make-cylinder 11 27))
     (send b20 :set-color :gray60)
     (setq b20 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b20)
			 :name "oyayubi-link-0"))
     (send b20 :locate #f(30 23  -18))
     (send b20 :rotate pi/2 :y)

     (setq b21 (make-cylinder 11 40))
     (send b21 :set-color :gray60)
     (setq b21 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b21)
			 :name "oyayubi-link-1"))
     (send b21 :locate #f(57 25  -18))
     (send b21 :rotate pi :z)
     (setq b22 (make-cylinder 11 40))
     (send b22 :set-color :gray60)
     (setq b22 (instance bodyset-link :init (make-cascoords)
			 :bodies (list b22)
			 :name "oyayubi-link-2"))
     (send b22 :locate #f(57 25 25))
     (send b22 :rotate pi :z)
     (setq thumb-end-coords 
       (make-cascoords :pos (float-vector 47 25 (+ 55 20))))
     (send b22 :assoc thumb-end-coords)
     ;;
     (send self :assoc b0)
     (send b0 :assoc b00)
     (send b00 :assoc b01)
     (send b0 :assoc b10)
     (send b10 :assoc b11)
     (send b0 :assoc b20)
     (send b20 :assoc b21)
     (send b21 :assoc b22)
     (send b00 :rotate 0.14889 :x)

     (setq j00 (instance rotational-joint :init :name :j00
			 :child-link b00 :parent-link b0 :axis :y
			 :min 00 :max 120))
     (setq j01 (instance rotational-joint :init :name :j01
			 :child-link b01 :parent-link b00 :axis :y
			 :min 0 :max 120))
     (setq j10 (instance rotational-joint :init :name :j10
			 :child-link b10 :parent-link b0 :axis :y
			 :min 0 :max 120))
     (setq j11 (instance rotational-joint :init :name :j11
			 :child-link b11 :parent-link b10 :axis :y
			 :min 0 :max 120))
     (setq j20 (instance rotational-joint :init :name :j10
			 :child-link b20 :parent-link b0 :axis :x
			 :min -90 :max 45))
     (setq j21 (instance rotational-joint :init :name :j11
			 :child-link b21 :parent-link b20 :axis :y
			 :min -90 :max 0))
     (setq j22 (instance rotational-joint :init :name :j11
			 :child-link b22 :parent-link b21 :axis :y
			 :min 0 :max 120))
     
     ;;
     (setq links (list b0 b00 b01 b10 b11 b20 b21 b22))
     (setq joint-list (list j00 j01 j10 j11 j20 j21 j22))

     (send self :init-ending)
     self))
  (:thumb-end-coords () thumb-end-coords)
  (:middle-end-coords () middle-end-coords)
  (:index-end-coords () index-end-coords)
  )

;; user-defined joint
(defmethod sample-hand
  (:j00 (&rest args) (forward-message-to j00 args))
  (:j01 (&rest args) (forward-message-to j01 args))
  (:j10 (&rest args) (forward-message-to j10 args))
  (:j11 (&rest args) (forward-message-to j11 args))
  (:j10 (&rest args) (forward-message-to j20 args))
  (:j11 (&rest args) (forward-message-to j21 args))
  (:j11 (&rest args) (forward-message-to j22 args))
  )
