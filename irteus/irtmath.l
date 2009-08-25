;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
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

(defun inverse-matrix (mat) ;; redefined
  (let* ((dim (array-dimension mat 0))
	 (work (unit-matrix dim))
	 (perm (lu-decompose2 mat work))
	 (rvec)
         (result (make-matrix dim dim))
	 (vec (instantiate float-vector dim))
	 (i 0))
     (when (null perm)
       (warn ";; could not solve inverse-matrix~%")
       (return-from inverse-matrix work))
     (dotimes (i dim)
       (setq perm (lu-decompose2 mat work))
       (setf (aref vec i) 1.0)
       (setq rvec (lu-solve2 work perm vec))
       (dotimes (j dim) (aset result j i (aref rvec j) ))
       (setf (aref vec i) 0.0))
     result))

(defun diagonal (v)
  "make diagonal matrix from given vecgtor, diagonal #f(1 2) ->#2f((1 0)(0 2))"
  (let* ((size (length v))
	 (m (make-matrix size size)))
    (dotimes (i size)
      (setf (aref m i i) (elt v i)))
    m))

(defun minor-matrix (m ic jc)
  " return a matrix removing ic row and jc col elements from m"
  (let*
      ((row (1- (array-dimension m 0)))
       (col (1- (array-dimension m 1)))
       (mat (make-matrix row col)))
    (do ((r 0 (1+ r)) (i 0 (1+ i)))
	((= r row))
      (if (= i ic) (setq i (1+ i)))
      (do ((c 0 (1+ c)) (j 0 (1+ j)))
	  ((= c col))
	(if (= j jc) (setq j (1+ j)))
	(setf (aref mat r c) (aref m i j))))
    mat)
  )

(defun atan2 (y x)
  (let* ((eps 1.0e-10) (-eps (- eps)))
    (if (> x eps)
	(return-from atan2 (atan (/ y x))))
    (if (< x -eps)
	(if (> y 0)
	    (return-from atan2 (+ pi (atan (/ y x))))
	  (return-from atan2 (- (atan (/ y x)) pi)))
      )
    (if (< (abs y) eps) (return-from atan2 0.0))
    (if (> y 0) pi/2 -pi/2)
    ))

(defun outer-product-matrix (v &optional (ret (unit-matrix 3)))
  ;; 外積の行列を作る
  ;; matrix(a) v = a * v 
  ;;  0 -w2 w1
  ;;  w2 0 -w0
  ;; -w1 w0  0
  (let ()
    (setf (aref ret 0 0) 0)
    (setf (aref ret 0 1) (- (aref v 2)))
    (setf (aref ret 0 2) (aref v 1))
    ;;
    (setf (aref ret 1 0) (aref v 2))
    (setf (aref ret 1 1) 0)
    (setf (aref ret 1 2) (- (aref v 0)))

    (setf (aref ret 2 0) (- (aref v 1)))
    (setf (aref ret 2 1) (aref v 0))
    (setf (aref ret 2 2) 0)
    ret))


(defun matrix2quaternion (m)
  (let (q0 q1 q2 q3  mq^2
	   (q0^2 (/ (+ 1 (aref m 0 0) (aref m 1 1) (aref m 2 2)) 4))
	   (q1^2 (/ (+ 1 (aref m 0 0) (- (aref m 1 1)) (- (aref m 2 2))) 4))
	   (q2^2 (/ (+ 1 (- (aref m 0 0)) (aref m 1 1) (- (aref m 2 2))) 4))
	   (q3^2 (/ (+ 1 (- (aref m 0 0)) (- (aref m 1 1)) (aref m 2 2)) 4)))
    (setq mq^2 (max q0^2 q1^2 q2^2 q3^2))
    (cond ((eps= mq^2 q0^2)
	   (setq q0 (sqrt q0^2))
	   (setq q1 (/ (- (aref m 2 1) (aref m 1 2)) (* 4 q0)))
	   (setq q2 (/ (- (aref m 0 2) (aref m 2 0)) (* 4 q0)))
	   (setq q3 (/ (- (aref m 1 0) (aref m 0 1)) (* 4 q0))))
	  ((eps= mq^2 q1^2)
	   (setq q1 (sqrt q1^2))
	   (setq q0 (/ (- (aref m 2 1) (aref m 1 2)) (* 4 q1))) 
	   (setq q2 (/ (+ (aref m 0 1) (aref m 1 0)) (* 4 q1)))
	   (setq q3 (/ (+ (aref m 0 2) (aref m 2 0)) (* 4 q1))))
	  ((eps= mq^2 q2^2)
	   (setq q2 (sqrt q2^2))
	   (setq q0 (/ (- (aref m 0 2) (aref m 2 0)) (* 4 q2)))
	   (setq q1 (/ (+ (aref m 0 1) (aref m 1 0)) (* 4 q2)))
	   (setq q3 (/ (+ (aref m 1 2) (aref m 2 1)) (* 4 q2))))
	  ((eps= mq^2 q3^2)
	   (setq q3 (sqrt q3^2))
	   (setq q0 (/ (- (aref m 1 0) (aref m 0 1)) (* 4 q3)))
	   (setq q1 (/ (+ (aref m 0 2) (aref m 2 0)) (* 4 q3)))
	   (setq q2 (/ (+ (aref m 1 2) (aref m 2 1)) (* 4 q3))))
	  (t
	   (warn ";; matrix2quaternion error~%")))
    (float-vector q0 q1 q2 q3)))

(defun quaternion2matrix (q)
  (let ((q0 (elt q 0)) (q1 (elt q 1)) (q2 (elt q 2)) (q3 (elt q 3)))
    (make-matrix 
     3 3
     (list
      (float-vector (+ (* q0 q0) (* q1 q1) (- (* q2 q2)) (- (* q3 q3)))
		    (* 2 (- (* q1 q2) (* q0 q3)))
		    (* 2 (+ (* q1 q3) (* q0 q2))))
      (float-vector (* 2 (+ (* q1 q2) (* q0 q3)))
		    (+ (* q0 q0) (- (* q1 q1)) (* q2 q2) (- (* q3 q3)))
		    (* 2 (- (* q2 q3) (* q0 q1))))
      (float-vector (* 2 (- (* q1 q3) (* q0 q2)))
		    (* 2 (+ (* q2 q3) (* q0 q1)))
		    (+ (* q0 q0) (- (* q1 q1)) (- (* q2 q2)) (* q3 q3))
		    )))
    ))

(defun matrix-log (m)
  (let* ((qq (matrix2quaternion m))
	 (q0 (elt qq 0))
	 (q (subseq qq 1 4))
	 th)
    (cond ((or (> q0 1.0e-10) (< q0 -1.0e-10))
	   (setq th (* 2 (atan (/ (norm q) q0)))))
	  ((> q0 0)(setq th pi/2))
	  (t (setq th -pi/2)))
    (scale th (normalize-vector q))
    ))

(defun matrix-exponent (omega &optional (p 1.0))
  (let (w amat)
    (setq w (norm omega))
    (setq amat (outer-product-matrix (normalize-vector omega)))
    (m+ (unit-matrix) 
	(m+ (scale-matrix (sin (* w p)) amat)
	    (scale-matrix (- 1.0 (cos (* w p))) (m* amat amat))))
    ))

(defun midrot (p r1 r2)
  (let (r omega)
    (setq r (m* (transpose r1) r2))
    (setq omega (matrix-log r))
    (setq r (matrix-exponent omega p))
    (m* r1 r)
    ))

;;
;;

(defun pseudo-inverse (mat &optional weight-vector ret wmat mat-tmp)
  (let* ((r (array-dimension mat 0)) (c (array-dimension mat 1))
	 a p tmp-mat)
    (if (null ret) (setq ret (make-matrix c r)))

    ;; without weight (use sv-decompose)
    (if (null weight-vector)
      (return-from pseudo-inverse (pseudo-inverse2 mat ret)))
    ;; with weight
    (setq a (max r c))
    (if (null wmat) (setq wmat (make-matrix a a)))
    (if (null mat-tmp) (setq mat-tmp (make-matrix r c)))
    (dotimes (i a) (setf (aref wmat i i) (sqrt (aref weight-vector i))))
    (if (<= r c)
	(progn
	  (setq p (pseudo-inverse2 (m* mat wmat mat-tmp) ret))
	  (if p (m* wmat p ret) nil))
      ;; (> r c)
      (progn
	(setq p (pseudo-inverse2 (m* wmat mat mat-tmp) ret))
	(if p (m* p wmat ret) nil)))))

(defun pseudo-inverse-org (m &optional ret winv mat-tmp-cr)
  (let (s r u w v x)
    (setq s (cadr (array-dimensions m)))
    (setq r (sv-decompose m)) ;; need buffer
    (unless r
      (warn ";; !!WARNING!! : sv-decompose failed.(~a)~%" m)
      (return-from pseudo-inverse-org nil))
    (setq u (elt r 0) w (elt r 1) v (elt r 2))
    ;;
    (if (null winv) (setq winv (make-matrix s s)))
    (if (null mat-tmp-cr) (setq mat-tmp-cr (make-matrix s (car (array-dimensions m)))))
    (if (null ret) (setq ret (make-matrix s (car (array-dimensions m)))))

    (fill (array-entity winv) 0)
    (setq x (aref w 0 0))
    (if (> x 0.0001)
	(dotimes (i s)
	  (if (> (/ (aref w i i) x) 0.0001)
	      (setf (aref winv i i) (/ 1.0 (aref w i i))))))
    ;;
    (setq ret (m* (m* v winv winv) (transpose u mat-tmp-cr) ret))
    ret))

(defun sr-inverse (mat &optional (k 1.0) weight-vector ret
		       wmat tmat umat umat2 mat-tmp
		       mat-tmp-rc mat-tmp-rr mat-tmp-rr2)
  (let* ((r (array-dimension mat 0)) (c (array-dimension mat 1)))
    ;; mat (r c)
    (if (null tmat) (setq tmat (make-matrix c r)))

    ;; without weight
    (when (null weight-vector)
      (if (null mat-tmp-rr) (setq mat-tmp-rr (make-matrix r r)))
      (if (null mat-tmp-rr2) (setq mat-tmp-rr2 (make-matrix r r)))
      (return-from sr-inverse (sr-inverse-org mat k
					      mat-tmp-rr tmat
					      mat-tmp-rr2)))

    (if (null ret) (setq ret (make-matrix c r)))
    (if (null wmat) (setq wmat (make-matrix c c)))
    (if (null mat-tmp-rc) (setq mat-tmp-rc (make-matrix r c)))
    
    ;; k=0 => sr-inverse = pseudo-inverse
    (if (= k 0.0) 
	(return-from sr-inverse (pseudo-inverse mat weight-vector ret wmat mat-tmp-rc)))
    ;;
    (if (null umat)
	(setq umat (unit-matrix r))
      (progn (fill (array-entity umat) 0)
	     (dotimes (i r) (setf (aref umat i i) 1.0))))
    (if (null umat2) (setq umat2 (make-matrix r r)))
    (if (null mat-tmp) (setq mat-tmp (make-matrix c r)))
    ;; with weight
    (dotimes (i c) (setf (aref wmat i i) (aref weight-vector i)))
    (m* (m* wmat (transpose mat tmat) mat-tmp)
	(inverse-matrix (m+ (m* mat mat-tmp umat2)
			    (scale-matrix k umat umat) umat)) ret)
    ret
    ))

(defun sr-inverse-org (mat &optional (k 1)
			   me mat-tmp-cr mat-tmp-rr)
  (let* ((d (array-dimensions mat))
	 (r (car d)) (c (cadr d)))
    (if (null me)
	(setq me (unit-matrix r))
      (progn
	(fill (array-entity me) 0)
	(dotimes (i r) (setf (aref me i i) 1.0))))
    
    (if (null mat-tmp-cr) (setq mat-tmp-cr (make-matrix c r)))
    (if (null mat-tmp-rr) (setq mat-tmp-rr (make-matrix r r)))
    (setq mat-tmp-cr (transpose mat mat-tmp-cr))
    (m* mat-tmp-cr
	(inverse-matrix (m+ (m* mat mat-tmp-cr mat-tmp-rr) (scale-matrix k me me) mat-tmp-rr))
	mat-tmp-cr)
    ))

(defun manipulability (jacobi &optional tmp-mrr tmp-mcr)
  (let* ((r (array-dimension jacobi 0))
	 (c (array-dimension jacobi 1)))
    (if (null tmp-mrr) (setq tmp-mrr (make-matrix r r)))
    (if (null tmp-mcr) (setq tmp-mcr (make-matrix c r)))
    (sqrt (max 0.0 (matrix-determinant (m* jacobi (transpose jacobi tmp-mcr) tmp-mrr))))))

;;
;; re-define
;;
(unless (fboundp 'normalize-vector-org)
  (remprop 'normalize-vector 'compiler::builtin-function-entry)
  (setf (symbol-function 'normalize-vector-org) (symbol-function 'normalize-vector))
  (comp::def-builtin-entry 'NORMALIZE-VECTOR-ORG "VNORMALIZE"))
(defun normalize-vector (v &optional r)
  "normalize-vector #f(0 0 0)->#f(0 0 0)."
  (if (< (norm v) 1.0e-5)
      (if r (fill r 0)
        (instantiate float-vector (length v)))
    (if r (normalize-vector-org v r)
      (normalize-vector-org v))))

(provide :irtmath "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.9  2009-08-25 01:51:01  k-okada
;;; mat-tmp-cc, mat-tmp-cr nolonger used in pseudo-inverse
;;;
;;; Revision 1.8  2009/08/07 11:22:38  k-okada
;;; add pseudo-inverse2, use array-dimensions
;;;
;;; Revision 1.7  2009/03/02 15:44:53  k-okada
;;; speed up sr-inverse-org
;;;
;;; Revision 1.6  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.5  2008/12/26 11:20:19  k-okada
;;; fix matrix-log, use atan not atan2
;;;
;;; Revision 1.4  2008/11/11 11:10:25  k-okada
;;; error handling when normalize-vector #f(0 0 0), again
;;;
;;; Revision 1.3  2008/11/11 03:01:18  k-okada
;;; error handling when normalize-vector #f(0 0 0) -> 0, add VNORMALIZE in irtc.c remove defun normalize-vector from irtmath.l
;;;
;;; Revision 1.2  2008/09/22 06:15:46  k-okada
;;; update sr-inverse
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;

