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
  "forward _args_ message to _to_ object"
  (if to
      (if args (send* to args) to)
    (if *debug* (warn ";; (forward-message-to ~A ~A) ~%" to args))))

(defun forward-message-to-all (to args)
  "forward _args_ message to all _to_ object"
  (if to
      (if args (apply #'send-all to args) to)
    (if *debug* (warn ";; (forward-message-to-all ~A ~A) ~%" to args))))

(defmacro send-message* (&rest msgs)
  `(apply #'send-message . ,msgs)
  )

(defmacro do-until-key-with-check
  (check &rest forms)
  `(prog1
       (while (and (null (select-stream (list *standard-input*) 0.0000001))
                   (eval ,check))
         ,@forms
         )
     (let ((strm (car (select-stream (list *standard-input*) 0.1)))) (if strm (read-line strm nil nil)))
     ))

(defmacro do-until-key (&rest forms)
  "iterate forms until any key input"
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
  (:init () "Initialize timer object." (send self :start) self)
  (:start () "Start timer." (setq buf (unix:gettimeofday)))
  (:stop () "Stop timer and returns elapsed time in seconds."
	 (let ((tmp nil))
	    (setq tmp (unix:gettimeofday))
	    (setq tmp (map cons #'- tmp buf))
	    (+ (car tmp) (/ (cadr tmp) 1000000f))))
  )

(defun permutation (lst n)
  "Returns permutation of given list"
  (if (< n 1)
      '(())
    (mapcan #'(lambda (x)
                (mapcar #'(lambda (y) (cons x y))
                        (permutation (remove x lst :count 1) (1- n))))
            lst)))

(defun combination (lst n)
  "Returns combination of given list"
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
  "Returns the elements of datum which maximizes key function"
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
  "Create euslisp interpreter server, data sent to socket is evaluated as lisp expression"
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
  "Connect euslisp interpreter server until success"
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
  "print formatted array"
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

;; interpolations
(defclass interpolator
  :super propertied-object
  :slots ((position-list :type cons) ;; list of control point
          (time-list :type cons)  ;; list of time[sec] from start for each control point
          ;;
          (position :type float-vector)   ;; current data point
          (time :type float)          ;; time [sec] from start
          (segment-num :type integer) ;; number of total segment
          (segment-time :type float)  ;; time[sec] with in each segment
          (segment :type integer)     ;; index of segment which is currently processing
          (interpolatingp :type symbol)
          ))

(defmethod interpolator
  (:init
   ()
   "Abstract class of interpolator"
   (setq time 0.0)
   (setq segment-time 0.0)
   (setq segment 0)
   (setq segment-num 0)
   (setq interpolatingp nil)
   self)
  (:reset
   (&rest
    args
    &key
    ((:position-list pl) (send self :position-list)) ;; list of control point
    ((:time-list tl) (send self :time-list))   ;; list of time from start for each control point, time in fisrt contrall point is zero, so length of this list is length of control point minus 1
    &allow-other-keys)
   "Initialize interpolator
   position-list: list of control point
   time-list: list of time from start for each control point, time in fisrt contrall point is zero, so length of this list is length of control point minus 1"
   (setq position-list pl)
   (setq time-list tl)
   (if (/= (length position-list) (1+ (length time-list)))
       (warning-message 1 "length of position-list must be length of time-list + 1"))
   (setq time 0.0)
   (setq segment-time 0.0)
   (setq segment 0)
   ;; segment-num must be length of position-list minus 1.
   (setq segment-num (1- (length position-list)))
   (setq interpolatingp nil))

  ;; accessor to slots
  (:position-list () "returns position list" position-list)
  (:position () "returns current position" position)
  (:time-list () "returns time list" time-list)
  (:time () "returns current time" time)
  (:segment-time () "returns time[sec] with in each segment" segment-time)
  (:segment () "returns index of segment which is currently processing" segment)
  (:segment-num () "returns number of total segment" segment-num)
  (:interpolatingp () "returns if it is currently processing" interpolatingp)

  ;; operation
  (:start-interpolation () "start interpolation" (setq interpolatingp t))
  (:stop-interpolation () "stop interpolation" (setq interpolatingp nil))
  (:pass-time
   (dt)
   "process interpolation for dt[sec]"
   (when interpolatingp
     (setq position (send self :interpolation))
     (incf time dt)
     (setq segment-time (- time (if (= segment 0) 0 (nth (1- segment) time-list))))
     (when (> time (nth segment time-list))
       (setq segment-time (- time (nth segment time-list)))
       (incf segment))
     (when (>= segment segment-num)
       (setq position (car (last position-list)))
       (send self :reset))
     position))
  )

(defclass linear-interpolator
  :super interpolator)
(defmethod linear-interpolator
  ;;
  (:interpolation
   ()
   "Linear interpolator"
   (let* ((v1 (nth segment position-list))
	  (v2 (nth (1+ segment) position-list))
	  (t1+t2 (- (nth segment time-list) (if (> segment 0) (nth (1- segment) time-list) 0))) ;; total time of segment
	  (t1 segment-time)
	  (t2 (- t1+t2 t1)))
     (setq v1 (scale (/ t2 t1+t2) v1))
     (setq v2 (scale (/ t1 t1+t2) v2))
     (v+ v1 v2)))
  )

;; Minimum Jerk trajectory generation, a.k.a Hoff & Arbib, described in the documents
;; http://mplab.ucsd.edu/tutorials/minimumJerk.pdf (you can find copy of this at http://www.shadmehrlab.org/book/minimumjerk.pdf)
;;
;; Hoff B, Arbib MA (1992) A model of the effects of speed, accuracy, and
;; perturbation on visually guided reaching. In: Control of arm movement
;; in space: neurophysiological and computational approaches
;; (Caminiti R, Johnson PB, Burnod Y, eds), pp 285-306.

(defclass minjerk-interpolator
  :super interpolator
  :slots (velocity acceleration velocity-list acceleration-list))
(defmethod minjerk-interpolator
  (:velocity () "returns current velocity" velocity)
  (:velocity-list () "returns velocity list" velocity-list)
  (:acceleration () "returns current acceleration" acceleration)
  (:acceleration-list () "returns acceleration list" acceleration-list)
  (:reset
   (&rest args
    &key
    ((:velocity-list vl) (send self :velocity-list))
    ((:acceleration-list al) (send self :acceleration-list))
    &allow-other-keys)
   "minjerk interopolator
    position-list : list of control point
    velocity-list : list of velocity in each control point
    acceleration-list : list of acceleration in each control point"
   (send-super* :reset args)
   (setq velocity-list (if vl vl (make-list (1+ segment-num) :initial-element (instantiate float-vector (length (car position-list))))))
   (setq acceleration-list (if al al (make-list (1+ segment-num) :initial-element (instantiate float-vector (length (car position-list))))))
   )

  (:interpolation
   ()
   "Minjerk interpolator, a.k.a Hoff & Arbib

    Example code is:
(setq l (instance minjerk-interpolator :init))
(send l :reset :position-list (list #f(1 2 3) #f(3 4 5) #f(1 2 3)) :time-list (list 0.1 0.18))
(send l :start-interpolation)
(while (send l :interpolatingp) (send l :pass-time 0.02) (print (send l :position)))
"
   (let* ((xi (nth segment position-list))
          (xf (nth (1+ segment) position-list))
          (vi (nth segment velocity-list))
          (vf (nth (1+ segment) velocity-list))
          (ai (nth segment acceleration-list))
          (af (nth (1+ segment) acceleration-list))
          ;;
          (t1+t2 (- (nth segment time-list) (if (> segment 0) (nth (1- segment) time-list) 0))) ;; total time of segment
          ;; A=(gx-(x+v*t+(a/2.0)*t*t))/(t*t*t)
          ;; B=(gv-(v+a*t))/(t*t)
          ;; C=(ga-a)/t
          (A  (scale (/ 1.0 (* t1+t2 t1+t2 t1+t2)) (v- xf (reduce #'v+ (list xi (scale t1+t2 vi) (scale (* t1+t2 t1+t2) (scale 0.5 ai)))))))
          (B  (scale (/ 1.0 (* t1+t2 t1+t2))      (v- vf (v+ vi (scale t1+t2 ai)))))
          (C  (scale (/ 1.0 (* t1+t2))           (v- af ai)))
          ;; a0=x
          ;; a1=v
          ;; a2=a/2.0
          ;; a3=10*A-4*B+0.5*C
          ;;; a4=(-15*A+7*B-C)/t
          ;; a5=(6*A-3*B+0.5*C)/(t*t)
          (a0 xi)
          (a1 vi)
          (a2 (scale 0.5 ai))
          (a3 (v+ (v- (scale 10 A) (scale 4 B)) (scale 0.5 C)))
          (a4 (scale (/ 1.0 t1+t2) (v- (v+ (scale -15 A) (scale 7 B)) C)))
          (a5 (scale (/ 1.0 t1+t2 t1+t2) (v+ (v+ (scale 6 A) (scale -3 B)) (scale 0.5 C))))
          )
     ;; x=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t
     ;; v=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t
     ;; a=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t
     (setq position
           (reduce #'v+ (list a0
                              (scale (expt segment-time 1) a1) (scale (expt segment-time 2) a2)
                              (scale (expt segment-time 3) a3) (scale (expt segment-time 4) a4)
                              (scale (expt segment-time 5) a5)))
           velocity
           (reduce #'v+ (list a1
                              (scale (expt segment-time 1) a2) (scale (expt segment-time 2) a3)
                              (scale (expt segment-time 3) a4) (scale (expt segment-time 4) a5)))
           acceleration
           (reduce #'v+ (list a2
                              (scale (expt segment-time 1) a3) (scale (expt segment-time 2) a4)
                              (scale (expt segment-time 3) a5))))
     position))
  ;
  )
#| example
(setq l (instance minjerk-interpolator :init))
(send l :reset :position-list (list #f(1 2 3) #f(3 4 5) #f(1 2 3)) :time-list (list 0.1 0.18))
(send l :start-interpolation)
(while (send l :interpolatingp) (send l :pass-time 0.02) (print (send l :position)))
|#

;; color utils
(defun his2rgb (h &optional (i 1.0) (s 1.0) ret)
  "convert his to rgb (0 <= h <= 360, 0.0 <= i <= 1.0, 0.0 <= s <= 1.0)"
  (hvs2rgb h i s ret))
(defun hvs2rgb (h &optional (i 1.0) (s 1.0) ret)
  "convert hvs to rgb (0 <= h <= 360, 0.0 <= i <= 1.0, 0.0 <= s <= 1.0)"
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
  "convert rgb to his (0 <= r,g,b <= 255)"
  (rgb2hvs r g b ret))
(defun rgb2hvs (r &optional g b ret)
  "convert rt to hvs (0 <= r,g,b <= 255)"
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

(defun color-category10 (i)
  "Choose good color from 10 colors"
  (his2rgb (* (mod i 10) 100) 1.0 0.8))
(defun color-category20 (i)
  "Choose good color from 20 colors"
  (his2rgb (* (mod i 20) 100) 1.0 0.8))

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
  "calculating execution time using mtimer, bench returns the value of args where the arbitrary lisp form
(bench (unix:sleep 1))
;; time -> 1.0001[s]
t
"
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
  "calculating execution time using mtimer, bench2 returns the execution time of args where the arbitrary lisp form
(bench2 (unix:sleep 1))
1.00024
"
  (let ((tm (gensym)))
    `(let ((,tm (instance mtimer :init)))
       (send ,tm :start)
       (progn
         ,@args
         (send ,tm :stop)))))

;; redirect euslisp standard-output and error-output (null-output and with-all-output->file)
;; null-ouput usage : (null-output (form1) (form2) .. )
(defmacro null-output (&rest bodies)
  `(with-all-output->file "/dev/null" ,@bodies))

;; with-all-output->file : (with-all-output->file "filename" (form1) (form2) .. )
(defmacro with-all-output->file (fname &rest bodies)
  (let ((tmp-stdout (gensym))
        (tmp-stderr (gensym))
        (null-port (gensym))
        (ret (gensym)))
    `(progn
       (let ((,tmp-stdout *standard-output*)
             (,tmp-stderr *error-output*)
             (,ret nil))
         (unwind-protect
             (with-open-file (,null-port ,fname :direction :output)
               (setq *standard-output* ,null-port)
               (setq *error-output* ,null-port)
               (setq ,ret (progn ,@bodies))
               )
           (progn
             (setq *standard-output* ,tmp-stdout)
             (setq *error-output* ,tmp-stdout)
             ,ret))))))

;; usage:
;; (read-char-case "y or n or r: "
;;  (#\y (format t "yes~%") t)
;;  (#\n (format t "no~%") t)
;;  (#\r (format t "retry~%") t)
;;  )
(defmacro read-char-case (str &rest clauses)
"read one charactor with lower caes
(read-char-case \"y or n or r: \"
 (#\y (format t \"yes~%\") t)
 (#\n (format t \"no~%\") t)
 (#\r (format t \"retry~%\") t))
"
  (let ((ch (gensym))
        (missed-return (gensym))
        (flag (gensym)))
    `(progn
       (let ((,flag ',missed-return))
         (while (eq ,flag ',missed-return)
           (format t ,str)
           (finish-output *standard-output*)
           (setq ,flag
                 (let ((,ch (read-char))) ;read-block
                   (case ,ch
                     ,@clauses
                     (t                 ;if does not match to anything
                      ',missed-return))))
           )
         ,flag))))


;;; from termbits.h
(defcstruct termios
  (c_iflag      :integer)
  (c_oflag      :integer)
  (c_cflag      :integer)
  (c_lflag      :integer)
  (c_line       :char)
  (c_cc         :char 19)
  (c_ispeed     :integer)
  (c_ospeed     :integer))

(defconstant TCSANOW 0)
(defconstant TCSADRAIN 1)
(defconstant TCSAFLUSH 2)
(defconstant ICANON #0x0002)
(defun kbhit ()
  "Checks the console for a keystroke. retuns keycode value if a key has been pressed, otherwise it returns nil. Note that this does not work well on Emacs Shell mode, run EusLisp program from terminal shell."
  (let ((attr-orig (unix::tcgetattr 0))
        (buf "0")
        attr ret)
    (setq attr (copy-list attr-orig))
    ;; 1000 1010 0011 1011
    (setf (termios-c_lflag attr) (logand (termios-c_lflag attr) (lognot ICANON)))
    (unix::tcsetattr 0 TCSANOW attr)
    (when (= (unix::select-read-fd 1 0.001) 1)
      (unix::uread 0 buf 1)
      (setq ret (elt buf 0)))
    (unix::tcsetattr 0 TCSANOW attr-orig)
    ret))
;;

;; piped fork returning result as list
(defun piped-fork-returns-list (cmd &optional args)
  "piped fork returning result as list"
  (let ((strm (apply #'piped-fork cmd args)))
    (unwind-protect
        (let ((ret) (buf))
          (while (setq buf (read-line strm nil))
            (push buf ret))
          (reverse ret))
      (close strm))))

(defun make-robot-model-from-name (name &rest args)
  "make a robot model from string: (make-robot-model \"pr2\")"
  (let ((name (string-downcase name)))
    (let ((klasses (flatten (send robot-model :hierarchy))))
      (dolist (klass klasses)
        (let ((klass-name (string-downcase (send klass :name))))
        (if (or (string= klass-name name)
                (string= (format nil "~A-robot" name) klass-name))
            (return-from make-robot-model-from-name (instance* klass :init args)))))
      nil)))


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
