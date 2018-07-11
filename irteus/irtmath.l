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
  "returns inverse matrix of mat"
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
  "return a matrix removing ic row and jc col elements from m"
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
  "returns atan2 of y and x (atan (/ y x))"
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
  "returns outer product matrix of given v \\\\
   {\\tt
   matrix(a) v = a * v \\\\
    0 -w2 w1\\\\
    w2 0 -w0\\\\
   -w1 w0  0\\\\
   }"
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
  "returns quaternion of given matrix"
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
  "returns matrix of given quaternion"
  (let ((q0 (elt q 0)) (q1 (elt q 1)) (q2 (elt q 2)) (q3 (elt q 3)))
    (unless (eps= (v. q q) 1.0 0.01)
        (warning-message 1 ";; quaternion2matrix : invalid input ~A, the norm is not 1~%" q))
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

(defun matrix-log (m) ;; matrix-log retuns [-pi, pi]
 "returns matrix log of given m, it returns [-pi, pi]"
 (let* ((qq (matrix2quaternion m))
        (q0 (elt qq 0))
        (q (subseq qq 1 4))
        (th (* 2.0 (atan (norm q) q0))))
   (cond ((> th pi)
          (setq th (- th 2pi)))
         ((< th -pi)
          (setq th (+ th 2pi))))
  (scale th (normalize-vector q))))

(defun matrix-exponent (omega &optional (p 1.0))
  "returns exponent of given omega"
  (let (w amat)
    (setq w (norm omega))
    (setq amat (outer-product-matrix (normalize-vector omega)))
    (m+ (unit-matrix) 
	(m+ (scale-matrix (sin (* w p)) amat)
	    (scale-matrix (- 1.0 (cos (* w p))) (m* amat amat))))
    ))

(defun midrot (p r1 r2)
  "returns mid (or p) rotation matrix of given two matrix r1 and r2"
  (let (r omega)
    (setq r (m* (transpose r1) r2))
    (setq omega (matrix-log r))
    (setq r (matrix-exponent omega p))
    (m* r1 r)
    ))

;;
;;

(defun pseudo-inverse (mat &optional weight-vector ret wmat mat-tmp)
  "returns pseudo inverse of given mat"
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
  "returns sr-inverse of given mat"
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
  "return manipulability of given matrix"
  (let* ((r (array-dimension jacobi 0))
	 (c (array-dimension jacobi 1)))
    (if (null tmp-mrr) (setq tmp-mrr (make-matrix r r)))
    (if (null tmp-mcr) (setq tmp-mcr (make-matrix c r)))
    (sqrt (max 0.0 (matrix-determinant (m* jacobi (transpose jacobi tmp-mcr) tmp-mrr))))))

;; random
(defun random-gauss (&optional (m 0) (s 1))
  "make random gauss, m:mean s:standard-deviation"
  (let ((x1 (random 1.0)) (x2 (random 1.0)))
    (+ (* (sqrt (* -2.0 s s (log x1))) (cos (* 2 pi x2))) m)))

(defun gaussian-random (dim &optional (m 0) (s 1))
  "make random gauss vector, replacement for quasi-random defined in matlib.c"
  (let ((res (instantiate float-vector dim)))
    (if (numberp m) (setq m (make-list dim :initial-element m)))
    (if (numberp s) (setq s (make-list dim :initial-element s)))
    (dotimes (i dim)
      (setf (aref res i) (random-gauss (elt m i) (elt s i))))
    res))


(defun eigen-decompose (m)
  ;; m is real symmetrix matrix we can use (ql-decompose m)
  (let* ((evalue (car (qr-decompose m)))
	 (l (length evalue))
	 (evector (make-matrix l l)) mm mmm tv ttv pv r w j (j-max 10) k d)
    (setq evalue (sort evalue #'>=))
    (dotimes (i l)
      (setq mm (m+ m (scale-matrix (- (elt evalue i)) (unit-matrix l))))
      (setq mmm (copy-matrix mm))
      ;; inverse iteration
      ;; using lu
      (when (setq pv (lu-decompose mm))
	;; make rondom vector
	(setq tv (instantiate float-vector l))
	;; generate non-zero random vector
	(while (eps= (norm tv)  0)
	  (dotimes (i l) (setf (elt tv i) (- (random 1.0) 0.5)))
	  (setq tv (normalize-vector tv)))
	;;
	(setq ttv tv j 0)
	(loop
	 (setq tv (lu-solve mm pv tv))
	 (setq tv (normalize-vector tv))
	 ;; exit loop when no updates
	 (if (or (>= (incf j) j-max) (eps= (distance ttv tv) 0))
	     (return-from nil))
	 ;; update eigen vector
	 (when (> j (/ j-max 2))
	   (setq d (/ 1.0
		      (v. (coerce pv float-vector)
			  (lu-solve mm pv tv)))) ;; non-normlaized value
	   ;;(setq k 1)
	   ;;(while (eps= (- (elt evalue i) (+ (elt evalue i) (* k d))) 0)
	   ;;  (incf k))
	   ;;(incf (elt evalue i) (* k d))
	   (incf (elt evalue i) d)
	   (setq mm (m+ m (scale-matrix (- (elt evalue i)) (unit-matrix l))))
	   (setq pv (lu-decompose mm))
	   (if (null pv) (return-from nil))
	   )
	 (setq ttv tv))
        (when (>= j j-max)
          ;;(warn ";; eigen decompose did not converge ~A:~A~%;; ~A~%" i (elt evalue i) m)
          (setq pv nil))
        )
      (unless pv
	(setq r (sv-decompose mmm))
	(setq w (elt r 1))
	(dotimes (j (length w))
	  (if (< (abs (elt w j)) 1e-4)
	      (setq tv (matrix-column (elt r 2) j))
	    )))
      (setf (matrix-column evector i) tv))
    (list evalue evector)))

;; lmeds
;;http://www-pse.cheme.kyoto-u.ac.jp/~kano/document/text-PCA.pdf
(defun lms (point-list)
  (let ((v^ (vector-mean point-list))
        (point-length (length point-list))
	delx x v eigen-res eigen-val eigen-vec min-lam min-vec)
    (setq x (make-matrix point-length (length v^)))
    (dotimes (i point-length)
      (setf (matrix-row x i) (v- (elt point-list i) v^)))
    (setq v  (scale-matrix (/ 1.0 (- point-length 1)) (m* (transpose x) x)))
    (setq eigen-res (eigen-decompose v))
    (setq eigen-val (car eigen-res) eigen-vec (cadr eigen-res))
    (setq min-lam (elt eigen-val 0)
          min-vec (matrix-column eigen-vec 0))
    (dotimes (i (length eigen-val))
      (when (> min-lam (elt eigen-val i))
        (setq min-lam (elt eigen-val i))
        (setq min-vec (matrix-column eigen-vec i))
        ))
    (list min-vec (- (v. v^ min-vec)))
    ))

(defun lms-estimate (res point-)
  (+ (v. point- (car res)) (cadr res))
  )

(defun lms-error (result point-list)
  (let ((ret-err 0) tmp-err)
    (dolist (l point-list)
      (setq tmp-err (lms-estimate result l))
      (incf ret-err (* tmp-err tmp-err))
      )
    (/ ret-err (length point-list))
    ))

;; choose num points randomly and apply lms to find the souliton with smallest errors
;; to use ransac ransac
;;   :lmeds-error-func -> set to ransac-error
;;   :ransac-threshold err^2 (square of the distance from the plane)
(defun lmeds (point-list &key (num 5) (err-rate 0.3) (iteration) (ransac-threshold)
			 (lms-func #'lms) (lmeds-error-func #'lmeds-error)
			 (lms-estimate-func #'lms-estimate))
  (let (point-num r result result-list error-list iter
		  comb-index comb-index-list point-list-tmp)
    ;; initialize variables
    (setq point-num (length point-list) num (min (max 2 num) point-num)
	  result-list nil error-list nil point-list-tmp nil)
    ;; make comb index list
    ;; (setq comb-index-list (cadr (combination num point-num)))
    ;; lmeds
    (if iteration (setq iter iteration)
      (setq iter (ceiling (/ (log 0.01) (log (- 1 (expt (- 1 err-rate) num)))))))
    (dotimes (j iter)
      (setq comb-index nil)
      (dotimes (i num)
	(setq r (random point-num))
	(while (find r comb-index)
	  (setq r (random point-num)))
	(push r comb-index))
      (push comb-index comb-index-list))
    ;; lms using comb-index-listload
    (dolist (ci comb-index-list)
      ;; make point-list-tmp
      (setq point-list-tmp nil)
      (dolist (c ci)
	(push (elt point-list c) point-list-tmp))
      (setq result (funcall lms-func point-list-tmp))
      (push result result-list)
      (if ransac-threshold
          (push (funcall lmeds-error-func result point-list ransac-threshold
                         :lms-estimate-func lms-estimate-func) error-list)
        (push (funcall lmeds-error-func result point-list
                       :lms-estimate-func lms-estimate-func) error-list))
      )
    (elt result-list
         (position (find-extream error-list #'(lambda (x) x) (if ransac-threshold #'> #'<)) error-list))
    ))

(defun lmeds-error (result point-list &key (lms-estimate-func #'lms-estimate))
  (let (tmp-err err)
    (dolist (l point-list)
      (setq tmp-err (funcall lms-estimate-func result l))
      (push (* tmp-err tmp-err) err)
      )
    (elt (sort err #'<) (/ (length err) 2))
    ))

(defun lmeds-error-mat (result mat &key (lms-estimate-func #'lms-estimate))
  (let ((size (array-dimension mat 0))
        (p (float-vector 0 0 0))
        tmp-err  err)
    (dotimes (i size)
      (c-matrix-row mat i p)
      (setq tmp-err (funcall lms-estimate-func result p))
      (push (* tmp-err tmp-err) err))
    (elt (sort err #'<) (/ (length err) 2))
    ))

(defun concatenate-matrix-column (&rest args)
  "Concatenate matrix in column direction."
  (let (m ml (size-of-column (array-dimension (car args) 1)))
    (dolist (mat args)
      (unless (= (array-dimension mat 1) size-of-column)
        (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args)))
      (setq m (if mat (length (matrix-column mat 0)) 0))
      (dotimes (i m)
        (push (matrix-row mat i) ml)))
    (when ml (apply #'matrix (reverse ml)))
    ))

(defun concatenate-matrix-row (&rest args)
  "Concatenate matrix in row direction."
  (let (m ml (size-of-column (array-dimension (car args) 0)))
    (dolist (mat args)
      (unless (= (array-dimension mat 0) size-of-column)
        (error ";; concatenate-matrix-row matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 0)) args)))
      (setq m (if mat (length (matrix-row mat 0)) 0))
      (dotimes (i m)
        (push (matrix-column mat i) ml)))
    (when ml (transpose (apply #'matrix (reverse ml))))
    ))

(defun concatenate-matrix-diagonal (&rest args)
  "Concatenate matrix in diagonal."
  (let (mat m ll ml vl)
    (dolist (mm args)
      (push (if mm (length (matrix-row mm 0)) 0) ll))
    (setq ll (reverse ll))
    (dotimes (i (length args))
      (setq mat (nth i args))
      (setq m (if mat (length (matrix-column mat 0)) 0))
      (dotimes (j m)
        (setq vl nil)
        (dotimes (k (length ll))
          (if (= i k) (push (matrix-row mat j) vl)
            (push (make-array (nth k ll)
                              :element-type float-vector
                              :initial-element 0) vl)))
        (push (apply #'concatenate
                     (cons float-vector (reverse vl))) ml)))
    (when ml (apply #'matrix (reverse ml)))
    ))

#|
(defun lms-draw (r point-list)
  (let (bs b str)
    (setq bs nil)
    (setq str "plot [0:80][0:50] '< echo \"")
    (dotimes (i (length point-list))
      (setq str (concatenate string str (format nil "~A ~A\\n"
						(elt (elt point-list i) 0)
						(elt (elt point-list i) 1)
						)))
      )
    (setq str (concatenate string str (format nil "\"' title \"data\"~%")))
    (format *gnuplot* "~a~%" str)

    (setq str (format nil "replot ~A * x + ~A title \"regression\""
		      (/ (elt (car r) 0) (- (elt (car r) 1)))
		      (/ (cadr r) (- (elt (car r) 1)))
		      ))
    (format *gnuplot* "~a~%" str)
    t))

(defun lms-demo (num)
  (let (r 
	(data1 '(#f(28.0 22.0) #f(46.0 36.0) #f(39.0 24.0) #f(25.0 22.0)
		   #f(34.0 27.0) #f(29.0 29.0) #f(38.0 26.0) #f(23.0 23.0)
		   #f(42.0 31.0) #f(27.0 24.0) #f(35.0 23.0) #f(39.0 27.0)
		   #f(38.0 31.0) #f(32.0 25.0) #f(25.0 23.0)))
	(data1o '(#f(28.0 22.0) #f(46.0 36.0) #f(39.0 24.0) #f(25.0 22.0)
		    #f(34.0 27.0) #f(29.0 29.0) #f(38.0 26.0) #f(23.0 23.0)
		    #f(72.0 2.0) #f(42.0 31.0) #f(27.0 24.0) #f(35.0 23.0)
		    #f(39.0 27.0) #f(38.0 31.0) #f(32.0 25.0)))
	)
    (when (not (boundp '*gnuplot*))
      (setq *gnuplot* (piped-fork "gnuplot")))

    (format t "apply lms with data~%")
    (setq r (lms data1))
    (format t "lms result~%")
    (lms-draw r data1)
    (format t ";; lms-error   ~10,3f, ~A~%" (lms-error r data1) r)
    (format t ";; >> input any key~%")
    (read-line)
    ;;
    (format t "apply lmeds with data~%")
    (setq r (lmeds data1))
    (format t "lmeds result~%")
    (lms-draw r data1)
    (format t ";; lmeds-error ~10,3f, ~A~%" (lmeds-error r data1) r)
    (format t ";; >> input any key~%")
    (read-line)
    ;;
    (format t "apply lms when outlier~%")
    (setq r (lms data1o))
    (format t "lms result when outlier~%")
    (lms-draw r data1o)
    (format t ";; lms-error   ~10,3f, ~A~%" (lms-error r data1o) r)
    (format t ";; >> input any key~%")
    (read-line)
    ;;
    (format t "apply lmeds when outlier~%")
    (setq r (lmeds data1o))
    (format t "lmeds result when outlier~%")
    (lms-draw r data1o)
    (format t ";; lmeds-error   ~10,3f, ~A~%" (lmeds-error r data1) r)
    ))
|#
;;
;; vector functions
;;
(defun vector-variance (vector-list)
  "returns vector, each element represents variance of elements in the same index of vector within vector-list"
  (let*  ((len (length vector-list))
          (m (make-matrix len (length (car vector-list)) vector-list))
          (ret (instantiate float-vector (length (car vector-list)))))
    (dotimes (i (length ret))
      (setf (elt ret i) (variance (matrix-column m i))))
    ret))

(defun covariance-matrix (vector-list)
  "make covariance matrix of given input vector-list"
  (let (m n)
    (setq n (length vector-list))
    (setq m (make-matrix n n))
    (dotimes (i n m)
      (dotimes (j n)
        (setf (aref m i j)
              (covariance (elt vector-list i)
                          (elt vector-list j)))
        ))
    ))

;;
;; re-define
;;
(unless (fboundp 'normalize-vector-org)
  (remprop 'normalize-vector 'compiler::builtin-function-entry)
  (setf (symbol-function 'normalize-vector-org) (symbol-function 'normalize-vector))
  (comp::def-builtin-entry 'NORMALIZE-VECTOR-ORG "VNORMALIZE"))
(defun normalize-vector (v &optional r (eps 1.0e-20))
  "calculate normalize-vector #f(0 0 0)->#f(0 0 0)."
  (if (< (norm v) eps)
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

