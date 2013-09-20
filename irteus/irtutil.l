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

(in-package "USER")

(export '(forward-message-to forward-message-to-all))

(defun forward-message-to (to args)
  (if to
      (if args (send* to args) to)
    (if *debug* (warn ";; (forward-message-to ~A ~A) ~%" to args))))

(defun forward-message-to-all (to args)
  (if to
      (if args (apply #'send-all to args) to)
    (if *debug* (warn ";; (forward-message-to-all ~A ~A) ~%" to args))))

(defmacro send-message* (&rest msgs)
  `(apply #'send-message . ,msgs)
  )

(defmacro do-until-key-with-check
  (check &rest forms)
  `(while (and (null (select-stream (list *standard-input*) 0.0000001))
               (eval ,check))
     ,@forms
     ))

(defmacro do-until-key (&rest forms)
  `(do-until-key-with-check t ,@forms))

(defmacro do-until-key-with-timer
  (time-limit &rest forms)
  (let ((tm (gensym)))
    `(let ((,tm (instance mtimer :init)))
       (do-until-key-with-check (< (send ,tm :stop) ,time-limit) ,@forms)
       )))

(defclass mtimer
  :super object
  :slots (buf))
(defmethod mtimer
  (:init () (send self :start) self)
  (:start () (setq buf (unix:gettimeofday)))
  (:stop ()
	 (let ((tmp nil))
	    (setq tmp (unix:gettimeofday))
	    (setq tmp (map cons #'- tmp buf))
	    (+ (car tmp) (/ (cadr tmp) 1000000f))))
  )

(defun permutation (lst n)
  (if (< n 1)
      '(())
    (mapcan #'(lambda (x)
                (mapcar #'(lambda (y) (cons x y))
                        (permutation (remove x lst :count 1) (1- n))))
            lst)))

(defun combination (lst n)
  (if (< n 1)
      '(())
    (mapcan #'(lambda (x)
                (mapcan #'(lambda (y)
                            (list (cons x y)))
                        (combination (cdr (member x lst)) (1- n))))
            lst)))

(defun mapjoin (expr seq1 seq2)
  (mapcan #'(lambda (arg1)
             (mapcar #'(lambda (arg2) (funcall expr arg1 arg2))
                     seq2))
             seq1))

(defun find-extreams (datum &key (key #'identity) (identity #'=) (bigger #'>))
  "returns the elementS of datum which maximizes key function"
  (let* ((max-value (funcall key (car datum)))
         (max-element (car datum))
         (results (list max-element)))
    (dolist (d (cdr datum))
      (let ((tmp (funcall key d)))
        (cond ((funcall bigger tmp max-value)
               (setq results (list d))
               (setf max-value tmp)
               (setf max-element d))
              ((funcall identity tmp max-value)
               (push d results)))))
    results))

;; thread pool functions
(defun need-thread (n &optional (lsize (* 512 1024)) (csize lsize))
 (let (num thrs)
   (setq num (- n (length (sys::free-threads))))
   (when (> num 0)
     (dotimes (i num)
       (sys:make-thread 1 lsize csize)
       (push (sys:thread #'(lambda ())) thrs))
     (dolist (thr thrs) (sys:wait-thread thr)))))

;; thread
(defun eus-server (&optional (port 6666) &key (host (unix:gethostname)))
  (let ((sp (make-socket-port
	     (make-socket-address
	      :host host
	      :domain af_inet
	      :port port))))
    (cond
     ((derivedp sp socket-port)
      (send *top-selector* :add-port sp
	    '(lambda (s)
	       (let ((strm (make-server-socket-stream s)))
		 (send *top-selector* :add-port strm
		       '(lambda (s)
			  (let ((c (read-char s nil nil)))
			    (cond
			     ((null c)
			      (send *top-selector* :remove-port s)
			      (close s))
			     (t
			      (unread-char c s)
			      (funcall 'lisp::repsel s
				       ;;"eus-server"
				       (gensym) nil nil)
			      ))))
		       strm)))
	    sp))
     (t
      (error (format nil "eus-server; port:~A host:~A" port host))
      )
     )
    ))

(defun connect-server-until-success (host port
                                     &key
                                     (max-port (+ port 20))
                                     (return-with-port nil))
  (let (s (p port))
    (while (and (not (streamp (setq s (connect-server host port))))
		(< port max-port))
      (incf port))
    (cond ((< port max-port)
	   (warn ";;; Connected server: ~c[31m~a@~a~c[m~%"
                 #x1b host port #x1b)
           (if return-with-port
               (list s port)
             s))
	  (t
	   (warn ";;; Can't connect server (~c[31m~a@~a-~a~c[m).~%"
                 #x1b host p (- max-port 1) #x1b
                 )
	   nil))
    ))

(defun format-array (arr &optional (header "") (in 7) (fl 3) (strm *error-output*) (use-line-break t))
  "arr &optional (header \"\") (in 7) (fl 3) (strm *error-output*);; for format-print array"
  (let* ((val-format (case (send arr :element-type)
                           (:integer (format nil "~~~dd " in))
                           (:float (format nil "~~~d,~df " in fl))))
         (str-format (format nil "~~~AA" in))
         (str-l (list (format nil str-format header))))
    (cond
     ((derivedp arr vector)
      (dotimes (i (length arr))
        (push (format nil val-format (elt arr i)) str-l))
      (if use-line-break (push "~%" str-l)))
     ((derivedp arr array)
      (dotimes (j (car (array-dimensions arr)))
        (if (/= j 0) (push (format nil str-format " ") str-l))
        (dotimes (i (cadr (array-dimensions arr)))
          (push (format nil val-format (aref arr j i)) str-l))
        (if use-line-break (push "~%" str-l)))))
    (let ((ret (format strm (apply #'concatenate string (nreverse str-l)))))
      (if strm arr ret))))

;; with-gensyms
(defmacro with-gensyms (syms &rest bodies)
  `(let ,(mapcar #'(lambda (s)
                     `(,s (gensym)))
                 syms)
     ,@bodies))

;; color utils
(defun his2rgb (h &optional i s ret)
  "(his2rgb h i s) ;; 0 <= h <= 360, 0.0 <= i <= 1.0, 0.0 <= s <= 1.0"
  (hvs2rgb h i s ret))
(defun hvs2rgb (h &optional i s ret)
  "(hsv2rgb h i s) ;; 0 <= h <= 360, 0.0 <= i <= 1.0, 0.0 <= s <= 1.0"
  (when (or (listp h) (vectorp h))
    (setq ret i)
    (setq s (elt h 2) i (elt h 1) h (elt h 0)))
  (if (>= h 360) (setq h (mod (round h) 360)))
  (if (< h    0) (setq h (- 360 (mod (round (- h)) 360))))
  (setq s (* 255 s) i (* 255 i))
  (let* ((hi (floor (/ h 60.0)))
	 (f (- (/ h 60.0) hi))
	 (p1 (* i (- 1.0 (/ s 255.0))))
	 (p2 (* i (- 1.0 (* f (/ s 255.0)))))
	 (p3 (* i (- 1.0 (* (- 1 f) (/ s 255.0)))))
	 r g b)
    (case hi
     ((0 6) (setq r i  g p3 b p1))
     (1 (setq r p2 g i  b p1))
     (2 (setq r p1 g i  b p3))
     (3 (setq r p1 g p2 b i))
     (4 (setq r p3 g p1 b i))
     (5 (setq r i  g p1 b p2))
     (t (warn "hri2rgb error")))
    (cond
     (ret
      (setf (elt ret 0) r)
      (setf (elt ret 1) g)
      (setf (elt ret 2) b)
      ret)
     (t (list r g b)))))

(defun rgb2his (r &optional g b ret)
  "(rgb2his r g b) ;; 0 <= r,g,b <= 255"
  (rgb2hvs r g b ret))
(defun rgb2hvs (r &optional g b ret)
  "(rgb2hvs r g b) ;; 0 <= r,g,b <= 255"
  (when (or (listp r) (vectorp r))
    (setq ret g)
    (setq g (elt r 1) b (elt r 2) r (elt r 0))
    )
  (let* ((max (max r g b))
	 (min (min r g b))
	 (d (float (- max min)))
	 (rt (- max (/ (* r 60.0) d)))
	 (gt (- max (/ (* g 60.0) d)))
	 (bt (- max (/ (* b 60.0) d)))
	 (i (/ max 255.0))
	 (s (if (= d 0.0) 0 (/ (* 255.0 d) max)))
	 (ht (if (= s 0) 0
	      (cond
	       ((= r max)(- bt gt))
	       ((= g max) (- (+ 120 rt) bt))
	       (t (- (+ 240 gt) rt)))))
	 (h (if (< ht 0.0) (+ ht 360) ht)))
    (setq s (/ s 255.0))
    (cond
     (ret
      (setf (elt ret 0) h)
      (setf (elt ret 1) i)
      (setf (elt ret 2) s)
      ret)
     (t (list h i s)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; macros for calculating execution time using mtimer
;; bench  : args -> arbitrary S-exp.
;;          return -> return value of args
;;          ex. $ (bench (unix:sleep 1))
;;                ;; time -> 1.0001[s]
;;                t
;;              $ (bench :print-string "hogehoge" (unix:usleep 500000))
;;                ;; hogehoge -> 0.500067[s]
;;                t
;; bench2 : args -> arbitrary S-exp.
;;          return -> execution time of args
;;          ex. $ (bench2 (unix:sleep 1))
;;                1.00024
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmacro bench (&rest args)
  (let ((tm (gensym))
        (str (gensym)))
    `(let ((,tm (instance mtimer :init))
           (,str ,(if (memq :print-string args)
                      (cadr (memq :print-string args))
                    "time")))
       (send ,tm :start)
       (prog1
           (progn ,@args)
         (format t ";; ~A -> ~A[s]~%" ,str (send ,tm :stop))))))

(defmacro bench2 (&rest args)
  (let ((tm (gensym)))
    `(let ((,tm (instance mtimer :init)))
       (send ,tm :start)
       (progn
         ,@args
         (send ,tm :stop)))))

(provide :irtutil "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.10  2010-02-05 13:34:33  ueda
;;; add :return-with-port keyword to connect-server-until-success
;;;
;;; Revision 1.9  2010/02/05 13:33:28  ueda
;;; add :max-port keyword to connect-server-until-success
;;;
;;; Revision 1.8  2010/02/04 05:18:40  k-okada
;;; move from jsk.l to irtutil.l, eus-server, connect-server-until-sucess
;;;
;;; Revision 1.7  2010/02/04 02:28:06  k-okada
;;; define need-thread in irtutil.l
;;;
;;; Revision 1.6  2009/11/27 11:06:45  k-okada
;;; add mapjoin
;;;
;;; Revision 1.5  2009/08/11 18:00:31  eus
;;; modify forward-message-to/to-all 's warning in case to is nil while only in *debug*
;;;
;;; Revision 1.4  2009/06/28 12:03:41  ueda
;;; add permutation and combination
;;;
;;; Revision 1.3  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.2  2008/10/09 15:09:46  k-okada
;;; add send-message
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
