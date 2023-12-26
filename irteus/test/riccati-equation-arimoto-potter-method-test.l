(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(defun eps-matrix= (m1 m2 &optional (eps *epsilon*))
   (eps= (distance (array-entity m1) (array-entity m2)) 0.0 eps))

(defun set-ABQR-sample1 ()
  ;; Hamilton matrix of this (A, B, Q, R) has only real number eigenvalue
  ;; About Hamilton matrix : http://stlab.ssi.ist.hokudai.ac.jp/yuhyama/lecture/linearsys/st8-8up.pdf
  (let* (A B Q R)
    (setq A (matrix
             (float-vector 0 1)
             (float-vector -10 -1)))
    (setq B (matrix
             (float-vector 0)
             (float-vector 1)))
    (setq Q (diagonal (float-vector 300 60)))
    (setq R (diagonal (float-vector 0.8)))
    (list A B Q R)
    ))

(defun set-ABQR-sample2 ()
  ;; Hamilton matrix of this (A, B, Q, R) has complex number eigenvalue
  ;; About Hamilton matrix : http://stlab.ssi.ist.hokudai.ac.jp/yuhyama/lecture/linearsys/st8-8up.pdf
  ;; Sample data : https://qiita.com/taka_horibe/items/5d053cdaaf4c886d27b5
  (let* (A B Q R)
    (setq A (matrix
             (float-vector 0 1 0 0)
             (float-vector 0 -15 10 0)
             (float-vector 0 0 0 1)
             (float-vector 0 0 0 -15)))
    (setq B (matrix
             (float-vector 0)
             (float-vector 10)
             (float-vector 0)
             (float-vector 1)))
    (setq Q (diagonal (float-vector 1 0 1 0)))
    (setq R (diagonal (float-vector 1)))
    (list A B Q R)
    ))

(defun lqr-two-wheeled-inverted-pendulum (&key
                                          (m-w 4.0) ;; mass of wheels [kg]
                                          (m-b 59.9271) ;; mass of base [kg]
                                          (r-w 0.0447) ;; radius of wheels [m]
                                          (L 0.905396) ;; distance between CoM of base and wheels [m]
                                          (I-w 0.002644) ;; inertia of wheels [kg*m^2]
                                          (I-b 11.3257) ;; inertia of base [kg*m^2]
                                          (g 9.8) ;; gravitational acceleration [m/s^2]
                                          (Q (diagonal (float-vector 10 0.8 90 5))) ;; weight parameter Q of LQR control
                                          (R (diagonal (float-vector 7))) ;; weight parameter R of LQR control
                                          )
  ;; Inverted pendulum model refers to the following paper
  ;; "TWIMP: Two-Wheel Inverted Musculoskeletal Pendulum as a Learning Control Platform in the Real World with Environmental Physical Contact" (published in Humanoids2018)
  ;; https://ieeexplore.ieee.org/document/8624923
  (let* ((a (+ (* (+ m-b m-w) (expt r-w 2)) I-w))
         (b (* m-b r-w L))
         (c (+ (* m-b (expt L 2)) I-b))
         (d (* m-b g L))
         EE A0 B0 AA BB)
    (setq EE (matrix
              (float-vector 1 0 0 0)
              (float-vector 0 1 0 0)
              (float-vector 0 0 (+ a (* 2 b) c) (+ a b))
              (float-vector 0 0 (+ a b) a)))
    (setq A0 (matrix
              (float-vector 0 0 1 0)
              (float-vector 0 0 0 1)
              (float-vector d 0 0 0)
              (float-vector 0 0 0 0)))
    (setq B0 (matrix
              (float-vector 0)
              (float-vector 0)
              (float-vector 0)
              (float-vector 1)))
    (setq AA (m* (inverse-matrix EE) A0))
    (setq BB (m* (inverse-matrix EE) B0))
    (send (instance riccati-equation-arimoto-potter-method :init AA BB Q R) :solve)
    ))

(deftest riccati-equation-arimoto-potter-method-test
  (let* ((ABQR1 (set-ABQR-sample1))
         (solution-ABQR1-arimoto-potter (send (instance riccati-equation-arimoto-potter-method :init (elt ABQR1 0) (elt ABQR1 1) (elt ABQR1 2) (elt ABQR1 3)) :solve))
         (solution-ABQR1-matlab-lqr (list
                                     #2f((165.9973 9.4356)
                                         (9.4356 7.1835)) ;; riccati solution P
                                     #2f((11.7945 8.9794)) ;; feedback gain K
                                     ))
         ;;
         (ABQR2 (set-ABQR-sample2))
         (solution-ABQR2-arimoto-potter (send (instance riccati-equation-arimoto-potter-method :init (elt ABQR2 0) (elt ABQR2 1) (elt ABQR2 2) (elt ABQR2 3)) :solve))
         (solution-ABQR2-matlab-lqr (list
                                     #2f((1.57106564 0.10038768 0.00582396 -0.00387677)
                                         (0.10038768 0.00655059 0.00038843 -0.00025543)
                                         (0.00582396 0.00038843 15.06666504 0.99999249)
                                         (-0.00387677 -0.00025543 0.99999249 0.06652974)) ;; riccati solution P
                                     #2f((1.000000 0.065250 1.003877 0.063975)) ;; feedback gain K
                                     ))
         ;;
         (solution-InvPend-arimoto-potter (lqr-two-wheeled-inverted-pendulum))
         (solution-InvPend-matlab-lqr (list
                                       #2f((31440.35317 131.47107 10757.90072 381.66021)
                                           (131.47107 2.60486 46.41979 1.74082)
                                           (10757.90072 46.41979 3759.68634 134.96830)
                                           (381.66021 1.74082 134.96830 5.03633)) ;; riccati solution P
                                       #2f((-180.17687 -0.33806 -55.55667 -1.10076)) ;; feedback gain K
                                       ))
         )
    ;;;;; Sample1: ABQR1 ;;;;;
    (assert (eps-matrix= (car solution-ABQR1-arimoto-potter) (car solution-ABQR1-matlab-lqr)) "[ABQR1] compare riccati solution P between riccati-equation-arimoto-potter-method and matlab-lqr")
    (assert (eps-matrix= (cadr solution-ABQR1-arimoto-potter) (cadr solution-ABQR1-matlab-lqr)) "[ABQR1] compare feedback gain K between riccati-equation-arimoto-potter-method and matlab-lqr")

    ;;;;; Sample2: ABQR2 ;;;;;
    (assert (eps-matrix= (car solution-ABQR2-arimoto-potter) (car solution-ABQR2-matlab-lqr)) "[ABQR2] compare riccati solution P between riccati-equation-arimoto-potter-method and matlab-lqr")
    (assert (eps-matrix= (cadr solution-ABQR2-arimoto-potter) (cadr solution-ABQR2-matlab-lqr)) "[ABQR2] compare feedback gain K between riccati-equation-arimoto-potter-method and matlab-lqr")

    ;;;;; Sample: InvPend ;;;;;
    (assert (eps-matrix= (car solution-InvPend-arimoto-potter) (car solution-InvPend-matlab-lqr)) "[InvPend] compare riccati solution P between riccati-equation-arimoto-potter-method and matlab-lqr")
    (assert (eps-matrix= (cadr solution-InvPend-arimoto-potter) (cadr solution-InvPend-matlab-lqr)) "[InvPend] compare feedback gain K between riccati-equation-arimoto-potter-method and matlab-lqr")
    ))

(eval-when (load eval)
  (run-all-tests)
  (exit))
