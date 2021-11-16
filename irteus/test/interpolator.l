(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest linear-interpolator
  (let* ((l (instance linear-interpolator :init))
         (p0 #f(1 2 3)) (t0 0.10)
         (p1 #f(3 4 5)) (t1 0.18))
    (send l :reset :position-list (list p0 p1 p0) :time-list (list t0 t1))
    (send l :start-interpolation)
    (do ((i 0 (+ i 0.02)))
        ((eps>= i t0))
        (send l :pass-time 0.02)
        (assert (eps-v= (send l :position) (midpoint (/ i t0) p0 p1)))
        (print (list i (send l :position) (midpoint (/ i t0) p0 p1)))
        )
    (do ((i t0 (+ i 0.02)))
        ((eps> i t1))
        (send l :pass-time 0.02)
        (assert (eps-v= (send l :position) (midpoint (/ (- i t0) (- t1 t0)) p1 p0)))
        (print (list i (send l :position) (midpoint (/ (- i t0) (- t1 t0)) p1 p0)))
        )
    (assert (null (send l :interpolatingp)))
    ))

(deftest minjerk-interpolator
  (let* ((l (instance minjerk-interpolator :init))
         (p0 #f(1 2 3)) (t0 0.10)
         (p1 #f(3 4 5)) (t1 0.18))
    (send l :reset :position-list (list p0 p1 p0) :time-list (list t0 t1))
    (send l :start-interpolation)
    (do ((i 0 (+ i 0.02)))
        ((eps> i t0))
        (send l :pass-time 0.02)
        )
    (assert (eps-v= (send l :position) p1))
    (print (list (send l :position) p1))
    ;;
    (do ((i t0 (+ i 0.02)))
        ((eps> i t1))
        (send l :pass-time 0.02)
        )
    (assert (eps-v= (send l :position) p0))
    (print (list (send l :position) p0))
    ;;
    (assert (null (send l :interpolatingp)))
    ))

(deftest linear-interpolator-SO3
  (let* ((l (instance linear-interpolator-SO3 :init))
         (p0 (rpy-matrix -pi/2 0 -pi/2)) (t0 0.10)
         (p1 (rpy-matrix 0 pi/2 pi/2)) (t1 0.18))
    (send l :reset :position-list (list p0 p1 p0) :time-list (list t0 t1))
    (send l :start-interpolation)
    (do ((i 0 (+ i 0.02)))
        ((eps>= i t0))
      (send l :pass-time 0.02)
      (assert (eps= 0 (matrix-log (m* (transpose (send l :position)) (midrot (/ i t0) p0 p1)))))
      (print (list i (send l :position) (midrot (/ i t0) p0 p1)))
      )
    (do ((i t0 (+ i 0.02)))
        ((eps> i t1))
      (send l :pass-time 0.02)
      (assert (eps= 0 (matrix-log (m* (transpose (send l :position)) (midrot (/ (- i t0) (- t1 t0)) p0 p1)))))
      (print (list i (send l :position) (midrot (/ (- i t0) (- t1 t0)) p0 p1)))
      )
    (assert (null (send l :interpolatingp)))
    ))

(deftest minjerk-interpolator-SO3
  (let* ((l (instance minjerk-interpolator-SO3 :init))
         (p0 (rpy-matrix -pi/2 0 -pi/2)) (t0 0.10)
         (p1 (rpy-matrix 0 pi/2 pi/2)) (t1 0.18))
    (send l :reset :position-list (list p0 p1 p0) :time-list (list t0 t1))
    (send l :start-interpolation)
    (do ((i 0 (+ i 0.02)))
        ((eps> i t0))
      (send l :pass-time 0.02)
      )
    (assert (eps= 0 (matrix-log (m* (transpose (send l :position)) p1))))
    (print (list (send l :position) p1))
    ;;
    (do ((i t0 (+ i 0.02)))
        ((eps> i t1))
      (send l :pass-time 0.02)
      )
    (assert (eps= 0 (matrix-log (m* (transpose (send l :position)) p0))))
    (print (list (send l :position) p0))
    ;;
    (assert (null (send l :interpolatingp)))
    ))

;; copy from https://github.com/jsk-ros-pkg/euslib/blob/master/tests/test-virtual-interpolator.l
;; pos interpolation function
;;   sample : euslib/test/test-virtual-interpolator.l
(defun pos-list-interpolation
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (interpolator-class minjerk-interpolator)
        ((:interpolator ip) (instance interpolator-class :init))
        (initial-time 0.0) (neglect-first) (vel-vector-list) (acc-vector-list))
  (let* ((data-list) (tm-list) (vel-data-list) (acc-data-list))
    (send* ip :reset
           :position-list pos-list
           :time-list (let (r) (dolist (n time-list) (push (+ n (if r (car r) 0)) r)) (nreverse r)) ;; list of time[sec] from start for each control point
           (append
            (if vel-vector-list (list :vel-vector-list vel-vector-list))
            (if acc-vector-list (list :acc-vector-list acc-vector-list))))
    (send ip :start-interpolation)
    (while (send ip :interpolatingp)
      (push (if (send ip :interpolatingp)
                (+ initial-time (send ip :time))
              (+ dt (car tm-list))) tm-list)
      (send ip :pass-time dt)
      (push (send ip :position) data-list)
      (if (find-method ip :velocity) (push (send ip :velocity) vel-data-list))
      (if (find-method ip :acceleration) (push (send ip :acceleration) acc-data-list))
      )
    (append
     (list :data (if neglect-first (cdr (reverse data-list)) (reverse data-list))
           :time (if neglect-first (cdr (reverse tm-list)) (reverse tm-list)))
     (if (find-method ip :velocity)
         (list :velocity (if neglect-first (cdr (reverse vel-data-list)) (reverse vel-data-list))))
     (if (find-method ip :acceleration)
         (list :acceleration (if neglect-first (cdr (reverse acc-data-list)) (reverse acc-data-list))))
     )))

(defun test-interpolators
  (&optional (ip-class linear-interpolator) (mode))
  (let* ((dt 0.01) ;; step time [s]
         (time-len 1.0) ;; total time [s]
         (ret-list
          (apply
           #'pos-list-interpolation
           (list (float-vector 0.0) (float-vector 30.0) (float-vector 90.0) (float-vector -90.0) (float-vector 0.0))
           (list (round (/ time-len dt 4)) (round (/ time-len dt 4)) (round (/ time-len dt 4)) (round (/ time-len dt 4)))
           dt
           :interpolator-class ip-class
           (if mode
               (list :interpolator (let ((ip (instance ip-class :init :sec/step dt)))
                                     (send ip :mode mode)
                                     ip)))
           ))
         (ret-list2 (mapcar #'(lambda (x) (elt x 0)) (cadr (memq :data ret-list)))))
    ;; (unless (or (null x::*display*) (= x::*display* 0))
    ;;   (graph-view
    ;;    (list ret-list2)
    ;;    (cadr (memq :time ret-list))
    ;;    :title (format nil "~A interpolator" (send ip-class :name))
    ;;    :xlabel "time [s]" :keylist (list "")))

    ;; check velocitiy
    (when (assoc :velocity (send ip-class :methods))
      (let ((position (cadr (memq :data ret-list)))
            (velocity (cadr (memq :velocity ret-list)))
            real-vel calc-vel)
        (dotimes (i (1- (length position)))
          (setq real-vel (scale (/ 1.0 dt) (v- (elt position (1+ i)) (elt position i)))
                calc-vel (elt velocity i))
          (assert (< (norm (v- real-vel calc-vel)) (if (memq :word-size=64 *features*) 0.01 0.15))
                  (format nil "pos: ~A, vel:~A, vel:~A, diff:~A~%" (elt position i) real-vel calc-vel (norm (v- real-vel calc-vel)))))
      ))

    ;; check acceleration
    (when (assoc :acceleration (send ip-class :methods))
      (let ((velocity (cadr (memq :velocity ret-list)))
            (acceleration (cadr (memq :acceleration ret-list)))
            real-acc calc-acc)
        (dotimes (i (1- (length velocity)))
          (setq real-acc (scale (/ 1.0 dt) (v- (elt velocity (1+ i)) (elt velocity i)))
                calc-acc (elt acceleration i))
        (assert (< (norm (v- real-acc calc-acc)) (if (memq :word-size=64 *features*) 0.01 0.15))
                (format nil "vel: ~A, acc:~A, acc:~A, diff:~A~%" (elt velocity i) real-acc calc-acc (norm (v- real-acc calc-acc)))))
      ))

    ;; check overshoot
    (assert
     (and
      (> (reduce #'(lambda (x y) (min x y)) ret-list2) (- -90 *epsilon*))
      (< (reduce #'(lambda (x y) (max x y)) ret-list2) (+ 90 *epsilon*))))
    ))

(deftest test-linear-interpolator ()
  (let ((res (test-interpolators linear-interpolator)))))

(deftest test-minjerk-absolute-interpolator ()
  (let ((res (test-interpolators minjerk-interpolator)))))

(defun test-interpolators-SO3
  (&optional (ip-class linear-interpolator-SO3) (mode))
  (let* ((dt 0.01) ;; step time [s]
         (time-len 10.0) ;; total time [s]
         (ret-list
          (pos-list-interpolation
           (list (rpy-matrix 0 0 0) (rpy-matrix 0 0.7 0) (rpy-matrix 0 pi/2 0) (rpy-matrix -pi/2 0 pi/2) (rpy-matrix 0 0 0))
           (list (/ time-len 4) (/ time-len 4) (/ time-len 4) (/ time-len 4))
           dt
           :interpolator-class ip-class
           )))
    ;; (unless (or (null x::*display*) (= x::*display* 0))
    ;;   (graph-view
    ;;    (list ret-list2)
    ;;    (cadr (memq :time ret-list))
    ;;    :title (format nil "~A interpolator" (send ip-class :name))
    ;;    :xlabel "time [s]" :keylist (list "")))

    ;; check velocitiy
    (when (assoc :velocity (send ip-class :methods))
      (let ((position (cadr (memq :data ret-list)))
            (velocity (cadr (memq :velocity ret-list)))
            real-vel calc-vel)
        (dotimes (i (1- (length position)))
          (setq real-vel (scale (/ 1.0 dt) (matrix-log (m* (transpose (elt position i)) (elt position (1+ i)))))
                calc-vel (elt velocity i))
          (assert (< (norm (v- real-vel calc-vel)) (if (memq :word-size=64 *features*) 0.1 0.15))
                  (format nil "pos: ~A, vel:~A, vel:~A, diff:~A~%" (elt position i) real-vel calc-vel (norm (v- real-vel calc-vel)))))
        ))

    ;; check acceleration
    (when (assoc :acceleration (send ip-class :methods))
      (let ((velocity (cadr (memq :velocity ret-list)))
            (acceleration (cadr (memq :acceleration ret-list)))
            real-acc calc-acc)
        (dotimes (i (1- (length velocity)))
          (setq real-acc (scale (/ 1.0 dt) (v- (elt velocity (1+ i)) (elt velocity i)))
                calc-acc (elt acceleration i))
          (assert (< (norm (v- real-acc calc-acc)) (if (memq :word-size=64 *features*) 0.1 0.15))
                  (format nil "vel: ~A, acc:~A, acc:~A, diff:~A~%" (elt velocity i) real-acc calc-acc (norm (v- real-acc calc-acc)))))
        ))
    ))

(deftest test-linear-interpolator-SO3 ()
  (let ((res (test-interpolators-SO3 linear-interpolator-SO3)))))

(deftest test-minjerk-interpolator-SO3 ()
  (let ((res (test-interpolators-SO3 minjerk-interpolator-SO3)))))

#|
(load "~/prog/euslib/jsk/gnuplotlib.l")
(setq r (pos-list-interpolation (list #f(0) #f(30) #f(90) #f(-90) #f(0)) (list 0.25 0.25 0.25 0.25) 0.01))
(setq r-pos (mapcar #'(lambda (x) (elt x 0)) (cadr (memq :data r))))
(setq r-vel (mapcar #'(lambda (x) (/ (elt x 0)  10)) (cadr (memq :velocity r))))
(setq r-acc (mapcar #'(lambda (x) (/ (elt x 0) 100)) (cadr (memq :acceleration r))))
(graph-view (list r-pos r-vel r-acc) (cadr (memq :time r)) :keylist (list "position" "velocity" "acceleration"))
(print r-pos)
(print (length r-pos))
|#

(eval-when (load eval)
  (run-all-tests)
  (exit))
