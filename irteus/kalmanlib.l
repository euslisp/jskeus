;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id $
;;;
;;; Copyright (c) 1987- JSK, The University of Tokyo.  All Rights Reserved.
;;;
;;; This software is a collection of EusLisp code for robot applications,
;;; which has been developed by the JSK Laboratory for the IRT project.
;;; For more information on EusLisp and its application to the robotics,
;;; please refer to the following papers.
;;;
;;; Toshihiro Matsui
;;; Multithread object-oriented language euslisp for parallel and
;;;  asynchronous programming in robotics
;;; Workshop on Concurrent Object-based Systems,
;;;  IEEE 6th Symposium on Parallel and Distributed Processing, 1994
;;;
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;
;;;
;;;

(list "$Id: kalmanlib.l 59620 2013-09-25 03:50:20Z nozawa $")

(defclass kalman-filter
  :super propertied-object
  :slots (A B H I Q R x_k u_k z_k P_k x_k-1 u_k-1 P_k-1 K S vlst))

(defmethod kalman-filter
  (:init 
   (&key (state-dim 2) (measure-dim 2) (q-variance 1e-5) (r-variance 0.01))
   (setq A (unit-matrix state-dim)) ;;(make-matrix state-dim state-dim))
   (setq B (unit-matrix state-dim))
   (setq H (make-matrix measure-dim state-dim))
   (setq I (unit-matrix state-dim))
   (dotimes (i (apply #'min (array-dimensions (make-matrix measure-dim state-dim))))
     (setf (aref H i i) 1))
   (setq Q (scale-matrix q-variance (unit-matrix state-dim)))
   (setq R (scale-matrix r-variance (unit-matrix measure-dim)))
   (setq z_k (make-array measure-dim :element-type :float))
   (setq u_k (make-array state-dim :element-type :float))
   (setq x_k (transform (transpose H) z_k))
   (setq P_k (unit-matrix state-dim))
   self)
  (:A (&optional m) (if m (setq A m) A))
  (:B (&optional m) (if m (setq B m) B))
  (:H (&optional m) (if m (setq H m) H))
  (:P () P_k)
  (:Q (&optional m) (if m (setq Q m) Q))
  (:R (&optional m) (if m (setq R m) R))
  (:x (&optional v) (if v (setq x_k v) x_k))
  (:u (&optional v) (if v (setq u_k v) u_k))
  (:predict 
   () 
   (setq x_k-1 x_k)
   (setq u_k-1 u_k)
   (setq P_k-1 P_k)
   (setq x_k (v+ (transform A x_k-1) (transform B u_k-1)))
   (setq P_k (m+ (m* A (m* P_k-1 (transpose A))) Q))
   x_k)
  (:correct 
   (z) ; measurement
   (setq z_k z)
   ;; S = HPH^ + R
   (setq S (m+ (m* H (m* P_k (transpose H))) R))
   ;; K = P H^ S-1
   (setq K (m* (m* P_k (transpose H)) (inverse-matrix S)))
   ;; x = x + K y
   ;; y = z - H x
   (setq x_k (v+ x_k (transform K (v- z_k (transform H x_k)))))
   ;; P = (I - K H) P
   (setq P_k (m* (m- I (m* K H)) P_k))
   t)
  (:proc
   (z &key (adaptive nil) (m 15)) ;; use m times observed data in adaptive kalman filter
   (send self :predict)
   (send self :correct z)
   (if adaptive
       (cond
        ((< (length vlst) m)
         (setq vlst (append vlst (list (v- z (transform H x_k)))))
         )
        (t
         (pop vlst)
         (setq vlst (append vlst (list (v- z (transform H x_k)))))
         (let ((cov-matrix (scale-matrix 0 R)))
           (dolist (v vlst nil)
             (setq cov-matrix (m+ cov-matrix (m* (transpose (matrix v)) (matrix v)))))
           (setq cov-matrix (scale-matrix (/ 1.0 m) cov-matrix))
           ;; (ros::ros-debug "cov-mat:~A" cov-matrix)
           (setq R (m+ cov-matrix (m* H (m* P_k (transpose H)))))
           ))))
   (transform H x_k)
   )
  (:error
   nil
   (let ((z (v- z_k (transform H x_k))))
     (* (sqrt (matrix-determinant S)) (exp (* 0.5 (v. (transform z (inverse-matrix S)) z))))))
  )

(defun kalman-demo (&optional demo-type)
  "Function to execute demo using kalman-filter and visualize the results.
   Args:
    demo-type (optional, keyword): :line-pos :line-pos-vel :sin-pos :sin-pos-vel :adaptive :non-linear
  "
  (let ((demo-types (list :line-pos :line-pos-vel :sin-pos :sin-pos-vel :adaptive :non-linear))
        (suffix (string-downcase demo-type))
        x y z kf)
    ;; x = sequential linear values
    ;; y = observable values at each timepoint x
    ;; z = estimated values from y
    (unless demo-type
    (format t ";; kalman-demo~%;; args: ~A~%" (append demo-types (list :all)))
    (return-from kalman-demo nil))
    (case demo-type
      (:all
       (dolist (typ demo-types)
         (kalman-demo typ)
         (unix:sleep 1))
       (return-from kalman-demo t))
      (:line-pos
       (format t ";; data -> line, kalman -> Position model~%"))
      (:line-pos-vel
       (format t ";; data -> line, kalman -> Position-Velocity model~%"))
      (:sin-pos
       (format t ";; data -> sin,  kalman -> Position model~%"))
      (:sin-pos-vel
       (format t ";; data -> sin,  kalman -> Position-Velocity model~%"))
      (:adaptive
       (format t ";; data -> sin, kalman -> Adaptive Position-Velocity model~%"))
      (:non-linear
       (format t ";; data -> a*x^2+c,  kalman -> Position-Velocity model~%"))
      (t
       (format t ";; kalman-demo~%;; args: ~A~%" (append demo-types (list :all)))
       (return-from kalman-demo nil)))

    ;; make dataset
    (setq x (make-sequence float-vector 100))
    (setq y (make-sequence float-vector 100))
    (dotimes (i (length x))
      (setf (elt x i) i)
      (setf (elt y i)
            (case demo-type
              ((:line-pos :line-pos-vel)
               (+ -0.37727 (random-gauss 0 0.1)))
              ((:sin-pos :sin-pos-vel :adaptive)
               (+ -0.37727 (random-gauss 0 0.05) (* 0.2 (sin (/ i 5.0)))))
              (:non-linear
               (+ -0.37727 (random-gauss 0 0.05) (* i i 5e-5)))
              (t (error "invalid demo-type")))))

    ;; initialize kalman filter
    (case demo-type
      ((:line-pos :sin-pos)
       (setq kf (instance kalman-filter :init :state-dim 2)))
     (t
      (setq kf (instance kalman-filter :init :state-dim 4 :r-variance 0.001))
      (send kf :A #2f((1 0 1 0)(0 1 0 1)(0 0 1 0)(0 0 0 1)))))
    (when (eq demo-type :non-linear)
      (send kf :u #f(0 0 0 1e-4)))

    ;; write groundtruth data
    (with-open-file
        (f (format nil "data_~a.dat" suffix) :direction :output)
      (dotimes (i (length x))
        (format f "~A ~A~%" (elt x i) (elt y i))))

    ;; write estimated data
    (with-open-file
        (f (format nil "kalman_result_~a.dat" suffix) :direction :output)
      (dotimes (i (length x))
        (case demo-type
          (:adaptive
           (setq z (send kf :proc (float-vector (elt x i) (elt y i)) :adaptive t)))
          (t
           (setq z (send kf :proc (float-vector (elt x i) (elt y i))))))
        (format f "~A ~A ~A ~A~%" (elt x i) (elt z 1) (send kf :error) (aref (send kf :P) 1 1))))

    ;; write graph data
    (let ((fname (format nil "visualize_~A.sh" suffix)))
      (format t ";; Writing gnuplot script to ~s~%" fname)
      (with-open-file
          (f fname :direction :output :permission #o744)
        (format f "#!/usr/bin/env gnuplot~%")
        (format f "~%")
        (format f "set title \"~A\"~%" suffix)
        (format f "plot [0:100][-0.6:0] 'data_~a.dat' title \"observed\" with linespoints~%" suffix)
        (format f "set y2tics~%")
        (format f "set y2range [0:0.01]~%")
        (format f "replot 'kalman_result_~a.dat' title \"estimated\" with linespoints~%" suffix)
        (format f "replot 'kalman_result_~a.dat' using 3 axes x1y2 title \"error\" with linespoints~%" suffix)
        (format f "replot 'kalman_result_~a.dat' using 4 axes x1y2 title 'P covariance' with linespoints~%" suffix)
        (format f "replot -0.37727 title \"baseline\"~%")
        (format f "pause -1~%"))
      (piped-fork (concatenate string (unix:getwd) "/" fname))
    t))
