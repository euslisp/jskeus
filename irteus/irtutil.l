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

(defmacro do-until-key (&rest forms)
  `(while (null (select-stream (list *standard-input*) 0.0000001))
     ,@forms
     )
  )

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
(defun eus-server (&optional (port 6666) &key host)
  (let ((sp (make-socket-port
	     (make-socket-address
	      :host (or host (unix:gethostname))
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
      (error "eus-server")
      )
     )
    ))

(defun connect-server-until-success (host port &key (max-port (+ port 20)))
  (let (s (p port))
    (while (and (not (streamp (setq s (connect-server host port))))
		(< port max-port))
      (incf port))
    (cond ((< port max-port)
	   (warn ";;; Connected server: ~c[31m~a@~a~c[m~%"
                 #x1b host port #x1b)
	   s)
	  (t
	   (warn ";;; Can't connect server (~c[31m~a@~a-~a~c[m).~%"
                 #x1b host p (- max-port 1) #x1b
                 )
	   nil))
    ))


(provide :irtutil "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.9  2010-02-05 13:33:28  ueda
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
