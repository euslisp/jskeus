;;;
;;;	Simulation Sample
;;;     Soft-III  2009.11.16 M.I
;;;
(defparameter *gravity* (float-vector 0 0 -0.5))

(defmacro v-x (v) `(elt ,v 0))
(defmacro v-y (v) `(elt ,v 1))
(defmacro v-z (v) `(elt ,v 2))
;;;
(defclass particle
  :super body
  :slots (pname
	  mass
	  velocity
	  acceleration))

(defmethod particle
  (:init (&key shape)
	 (if (derivedp shape body)
	     (replace-object self shape))
	 (setq pname nil
	       mass 1
	       velocity (float-vector 0 0 0)
	       acceleration *gravity*)
	 self)
  (:pname (&optional v)
	  (if v (setq pname v))
	  pname)
  (:mass  (&optional v)
	  (if v (setq mass v)) mass)
  (:position
   (&optional v)
   (when v 
     (send self :locate v :world)
     (send self :bounce-check))
   (send self :worldpos))
  (:position-x
   (&optional v)
   (if v (setf (v-x (send self :position)) v))
   (v-x (send self :position)))
  (:position-y
   (&optional v)
   (if v (setf (v-y (send self :position)) v))
   (v-y (send self :position)))
  (:position-z
   (&optional v)
   (if v (setf (v-z (send self :position)) v))
   (v-z (send self :position)))
  (:velocity (&optional (v velocity))
	     (setq velocity v))
  (:velocity-x (&optional (x (v-x velocity)))
	       (setf (elt velocity 0) x))
  (:velocity-y (&optional (y (v-y velocity)))
	       (setf (v-y velocity) y))
  (:velocity-z (&optional (z (v-z velocity)))
	       (setf (v-z velocity) z))
  (:acceleration
   (&optional (a acceleration))
   (setq acceleration a))
;;;
  (:new-position
   () (send self :position
	    (v+ (send self :position)
		(send self :velocity))))
  (:new-velocity
   () (send self :velocity
	    (v+ (send self :velocity)
		(send self :acceleration))))
  (:status
   ()
   (format t
	   "~%~a: P:~a V:~a A:~a"
	   (send self :pname)
	   (send self :position)
	   (send self :velocity)
	   (send self :acceleration)))
  (:bounce-x
   nil
   (send self :velocity-x
	 (- (send self :velocity-x))))
  (:bounce-y
   nil
   (send self :velocity-y
	 (- (send self :velocity-y))))
  (:bounce-z
   nil
   (send self :velocity-z
	 (- (send self :velocity-z))))
  (:bounce-check
   ()
   (let ((x (send self :position-x))
	 (y (send self :position-y))
	 (z (send self :position-z)))
     (cond
	 ((< x (v-x *space-box-min*))
	  (send self :position-x (v-x *space-box-min*))
	  (send self :bounce-x))
       ((> x (v-x *space-box-max*))
	(send self :position-x (v-x *space-box-max*))
	(send self :bounce-x)))
     (cond
	 ((< y (v-y *space-box-min*))
	  (send self :position-y (v-y *space-box-min*))
	  (send self :bounce-y))
       ((> y (v-y *space-box-max*))
	(send self :position-y (v-y *space-box-max*))
	(send self :bounce-y)))
     (cond
	 ((< z (v-z *space-box-min*))
	  (send self :position-z (v-z *space-box-min*))
	  (send self :bounce-z))
       ((> z (v-z *space-box-max*))
	(send self :position-z (v-z *space-box-max*))
	(send self :bounce-z)))
     )
   )
  )
;;;
(defclass rocket
  :super particle
  :slots (motor-force))
;;;
(defmethod rocket
  (:init (&rest args &key shape &allow-other-keys)
	 (send-super* :init args)
	 (setq motor-force (float-vector 0 0 0))
	 self)
  (:motor
   (&optional f)
   (cond (f
	  (setq motor-force f)
	  (send self :acceleration
		(v+ *gravity* motor-force)))
     (t motor-force)))
  (:status
   ()
   (send-super :status)
   (format t " M:~a" (send self :motor)))
  )

;;;
(defun make-something (typ particle-info)
  (let* ((b (make-gdome (make-icosahedron 15)))
	 x)
    (setq x (instance typ :init :shape b))
    (send x :pname (elt particle-info 0))
    (send x :position (elt particle-info 1))
    (send x :velocity (elt particle-info 2))
    (if (> (length particle-info) 3)
	(send x :set-color (elt particle-info 3))
      (send x :set-color :white))
    x)
  )

(defun init (&optional (a 400))
  (unless (boundp '*irtviewer*)
    (make-irtviewer :hither 10))
  (send *irtviewer* :title "particle")
  (setq *space-box* (make-cube a a a))
  (send *space-box* :set-color :yellow)
  (gl::transparent *space-box* 0.2)
  (setq *space-box-min*
	(send (send *space-box* :box) :minpoint))
  (setq *space-box-max*
	(send (send *space-box* :box) :maxpoint))
  (setq *p1*
	(make-something
	 particle '(p1 #f(10 -40 30) #f(2 0 5))))
  (setq *p2*
	(make-something
	 particle '(p2 #f(-50 50 100) #f(0 4 1))))
  (setq *p3*
	(make-something
	 particle '(p3 #f(80 0 10) #f(-4 -5 4))))
  (setq *r1*
	(make-something
	 rocket '(r1 #f(20 0 30) #f(4 3 3) :blue)))
  (setq *r2*
	(make-something
	 rocket '(r2 #f(70 50 50) #f(2 -2 1) :blue)))
  (setq *r3*
	(make-something
	 rocket '(r3 #f(20 100 40) #f(4 0 2) :blue)))
  (setq *particles*
	(list *p1* *p2* *p3* *r1* *r2* *r3*))
  (objects (append *particles* (list *space-box*)))
  t)

(defun particle-update ()
  (dolist (o *particles*)
    (send o :new-position)
    (send o :new-velocity)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)))

(defun add-motor (r m)
  (send r :set-color :yellow)
  (send r :motor m)
  (send r :status))

(defun all-status ()
  (dolist (p *particles*) (send p :status)))

(defun test (&optional (cnt 100))
  (do-until-key (particle-update)))

(defun sim-start ()
  (setq *top-selector-interval* 0.01)
  (pushnew 'particle-update *timer-job*)
  t)

(defun sim-stop ()
  (setq *top-selector-interval* 2)
  (setq *timer-job*
	(remove 'particle-update *timer-job*))
  nil)
;;;

(defun particle ()
  (format t "(init) (test) (sim-start) (sim-stop) (all-status)~%")
  (init)
  (test)
  (sim-start)
  (sim-stop)
  (all-status))

(warn "(particle) ;; for particle simulation~%")

