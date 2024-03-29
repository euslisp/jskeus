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
    (assert (= (length pos-list) (1+ (length time-list)))
            (format nil "check length of pos-list(~A) and tm-list(~A)"
                    (length pos-list) (length time-list)))
    (setq vel-vector-list
          (reverse
           (do ((i 0 (1+ i)) (vel-list))
               ((> i (length time-list)) vel-list)
               (if (or (= i 0) (= i (length time-list)))
                   (push (instantiate float-vector (length (car pos-list))) vel-list)
                 (let* ((v0 (scale (/ 1.0  (elt time-list (1- i)))
                                   (v- (elt pos-list i) (elt pos-list (1- i)))))
                        (v1 (scale (/ 1.0 (elt time-list i))
                                   (v- (elt pos-list (1+ i)) (elt pos-list i))))
                        (v (scale 0.5 (v+ v0 v1))))
                   (dotimes (i (length v)) (if (< (* (elt v0 i) (elt v1 i)) 0) (setf (elt v i) 0)))
                   (push v vel-list))))))
    (setq acc-vector-list
          (reverse
           (do ((i 0 (1+ i)) (acc-list))
               ((> i (length time-list)) acc-list)
               (if (or (= i 0) (= i (length time-list)))
                   (push (instantiate float-vector (length (car vel-vector-list))) acc-list)
                 (let* ((v0 (scale (/ 1.0  (elt time-list (1- i)))
                                   (v- (elt vel-vector-list i) (elt vel-vector-list (1- i)))))
                        (v1 (scale (/ 1.0 (elt time-list i))
                                   (v- (elt vel-vector-list (1+ i)) (elt vel-vector-list i))))
                        (v (scale 0.5 (v+ v0 v1))))
                   (dotimes (i (length v)) (if (< (* (elt v0 i) (elt v1 i)) 0) (setf (elt v i) 0)))
                   (push v acc-list))))))
    (format t "=INPUT~%")
    (format t "time ~A~%" time-list)
    (format t " pos ~A~%" pos-list)
    (format t " vel ~A~%" vel-vector-list)
    (format t " acc ~A~%" acc-vector-list)
    (send* ip :reset
           :position-list pos-list
           :time-list (let (r) (dolist (n time-list) (push (+ n (if r (car r) 0)) r)) (nreverse r)) ;; list of time[sec] from start for each control point
           (append
            (if vel-vector-list (list :velocity-list vel-vector-list))
            (if acc-vector-list (list :acceleration-list acc-vector-list))))
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
    (format t "=OUTPUT~%")
    (if (and vel-data-list acc-data-list)
        (mapcar #'(lambda (tm pos vel acc)
                    (format t "~7,5f ~7,3f ~13,1f ~13,1f~%"
                            tm (elt pos 0) (elt vel 0) (elt acc 0)))
                (reverse tm-list) (reverse data-list) (reverse vel-data-list) (reverse acc-data-list)))
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
    (when (and (not (or (null x::*display*) (= x::*display* 0)))
               (functionp 'graph-view))
      (let ((r-pos (mapcar #'(lambda (x) (elt x 0)) (cadr (memq :data ret-list))))
            (r-vel (mapcar #'(lambda (x) (* 10 (elt x 0))) (cadr (memq :velocity ret-list))))
            (r-acc (mapcar #'(lambda (x) (* 100 (elt x 0))) (cadr (memq :acceleration ret-list)))))
        (graph-view (list r-pos r-vel r-acc) (cadr (memq :time ret-list))
                    :keylist (list "position" "velocity" "acceleration")
                    :mode "linespoints")
        ))

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

;; https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/457
(defun test-interpolators-457
  (&optional (ip-class linear-interpolator) (dt 0.0001))
  (let* ((ret-list
          (pos-list-interpolation
           (list #f(0) #f(1) #f(2) #f(3) #f(4) #f(5) #f(6) #f(7) #f(8) #f(9))
           (list 0.001 0.001 0.001 0.001 0.001 0.001 0.001 0.001 0.001)
           dt
           :interpolator-class ip-class
           )))
    ;; check velocitiy
    (let ((position (cadr (memq :data ret-list)))
          (velocity (cadr (memq :velocity ret-list)))
          real-vel calc-vel)
      ;; check first and last velocity
      (assert (eps= (norm (car velocity)) 0.0 (if (memq :word-size=64 *features*) *epsilon* 100))
	      (format nil "vel:~A~%" (car velocity)))
      (assert (eps= (norm (car (last velocity))) 0.0 (if (memq :word-size=64 *features*) *epsilon* 100))
	      (format nil "vel:~A~%" (car (last velocity))))
      ;; check velocity of stable region
      (dotimes (i (1- (length position)))
        (when (and (> i (max 2 (* 0.3 (length position))))
                   (< i (min (- (length position) 3) (* 0.7 (length position)))))
          (setq real-vel (scale (/ 1.0 dt) (v- (elt position (1+ i)) (elt position i)))
                calc-vel (elt velocity i))
          (assert (eps= (norm (v- real-vel calc-vel)) 0 (if (memq :word-size=64 *features*) *epsilon* 100))
                  (format nil "~A pos: ~A, vel:~A, vel:~A, diff:~A~%" i (elt position i) real-vel calc-vel (norm (v- real-vel calc-vel)))))))

    ;; check acceleration
    (let ((velocity (cadr (memq :velocity ret-list)))
          (acceleration (cadr (memq :acceleration ret-list)))
          real-acc calc-acc)
      ;; check first and last acceleration
      (assert (eps= (norm (car acceleration)) 0.0 (if (memq :word-size=64 *features*) *epsilon* 100))
	      (format nil "acc:~A~%" (car acceleration)))
      (assert (eps= (norm (car (last acceleration))) 0.0 (if (memq :word-size=64 *features*) *epsilon* 100))
	      (format nil "acc:~A~%" (car (last acceleration))))
      ;; check maximum acceleration
      (dotimes (i (1- (length velocity)))
        (when (and (> i (max 2 (* 0.3 (length velocity))))
                   (< i (min (- (length velocity) 3) (* 0.7 (length velocity)))))
          (setq real-acc (scale (/ 1.0 dt) (v- (elt velocity (1+ i)) (elt velocity i)))
                calc-acc (elt acceleration i))
          (assert (eps= (norm (v- real-acc calc-acc)) 0 (if (memq :word-size=64 *features*) *epsilon* 100))
                  (format nil "~A vel: ~A, acc:~A, acc:~A, diff:~A~%" i (elt velocity i) real-acc calc-acc (norm (v- real-acc calc-acc)))))
        ))

    (when (and (not (or (null x::*display*) (= x::*display* 0)))
               (functionp 'graph-view))
      (let ((r-pos (mapcar #'(lambda (x) (elt x 0)) (cadr (memq :data ret-list))))
            (r-vel (mapcar #'(lambda (x) (* 0.001 (elt x 0))) (cadr (memq :velocity ret-list))))
            (r-acc (mapcar #'(lambda (x) (* 0.000001 (elt x 0))) (cadr (memq :acceleration ret-list)))))
        (graph-view (list r-pos r-vel r-acc) (cadr (memq :time ret-list))
                    :keylist (list "position" "velocity" "acceleration")
                    :mode "linespoints")
        (unix:usleep (* 500 1000))
        ))
    ))

;; https://github.com/euslisp/jskeus/pull/625
(defun test-interpolators-625
  (&optional (ip-class linear-interpolator))
  (let ((ip (instance ip-class :init)))
    (send ip :reset :position-list (list #f(1 2 3) #f(3 4 5) #f(1 2 3)) :time-list (list 1000 1800))
    (send ip :start-interpolation)
    (send ip :pass-time 200)
    (assert (send ip :interpolatingp))))

(deftest test-linear-interpolator ()
  (let ((res (test-interpolators linear-interpolator)))))

(deftest test-minjerk-absolute-interpolator ()
  (let ((res (test-interpolators minjerk-interpolator)))))

(deftest test-minjerk-absolute-interpolator-457 ()
  (let ((res (test-interpolators-457 minjerk-interpolator)))))

(deftest test-minjerk-absolute-interpolator-457-00013 ()
  (let ((res (test-interpolators-457 minjerk-interpolator 0.00013)))))

(deftest test-minjerk-absolute-interpolator-457-0013 ()
  (let ((res (test-interpolators-457 minjerk-interpolator 0.0013)))))

(deftest test-minjerk-absolute-interpolator-457-0005 ()
  (let ((res (test-interpolators-457 minjerk-interpolator 0.0005)))))

(deftest test-linear-interpolator-625 ()
  (let ((res (test-interpolators-625 linear-interpolator)))))

(deftest test-minjerk-interpolator-625 ()
  (let ((res (test-interpolators-625 minjerk-interpolator)))))


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
