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


(defclass gnuplot
  :super propertied-object
  :slots (strm data data-length last-command debug)
  )

(defmethod gnuplot
  (:init (host &key (clear t) ((:debug _debug)))
     "Initialize gnuplot interface object with given host name"
     (when (eq (unix:system "which gnuplot") 256)
       (setq strm t)
       (warning-message 1 "gnuplot is not installed. $ sudo apt-get install gnuplot~%")
       (return-from :init self))
	 (setq strm 
	       (cond
		((string= (unix:gethostname) host)
		 (piped-fork "gnuplot"))
		(t
		 (piped-fork
		  "rsh" host
		  (format nil "(setenv DISPLAY ~A:0 ; cd ~A ; gnuplot)"
			  (unix:gethostname) (pwd)))
		 )
		))
	 (setq data-length 10)
	 (if clear (send self :clear))
         (setq debug _debug)
	 self)
  (:clear
   ()
   "Clear graph"
   #-:cygwin
   (if (fboundp 'x::query-window-title-list)
       (let ((bef (x::query-window-title-list)) aft dif)
         (format strm "clear~%")
         (while (not dif)
           (setq aft (x::query-window-title-list))
           (setq dif (set-difference aft bef :test #'(lambda (a b) (= (cdr a) (cdr b))))))
         (setf (get self :win-id) (cdr (car dif)))
         )
     (format strm "clear~%"))
   #-:linux
   (format strm "clear~%")
   )
  ;; (send *G* :draw #f(0 1 2 3 4 5) #f(5 4 3 2 1 0) :xrange '(0 10) :yrange '(0 10) :title '("data1" "data2"))
  (:draw (&rest vs)
         "Draw graph with given float vectors,
:range, :xrange, :yrange, : range of each axis
:title : title of graph
:line-width : line width
:direction : direction of the graph (:right, :left)
:xscale, :xoffset : scale and offset for data
:y2tics : list variable to specify when y2 range is used
:y2range : set y2 tics and specify range
:type : specify type of the graph (:lines, :2dmap)
"
	 (setq last-command vs)
         (if debug (warn ";; :draw ~S~%" vs))
	 (let (str range xrange yrange title (clear nil) (line-width 1) (direction :right) y2tics y2range (xscale 1.0) (xoffset 0.0) (type :lines))
	   (dotimes (i (length vs))
	     (if (eq (elt vs i) :range) (setq range (elt vs (1+ i))))
	     (if (eq (elt vs i) :xrange) (setq xrange (elt vs (1+ i))))
	     (if (eq (elt vs i) :yrange) (setq yrange (elt vs (1+ i))))
	     (if (eq (elt vs i) :title) (setq title (elt vs (1+ i))))
	     (if (eq (elt vs i) :clear) (setq clear (elt vs (1+ i))))
	     (if (eq (elt vs i) :line-width) (setq line-width (elt vs (1+ i))))
	     (if (eq (elt vs i) :direction) (setq direction (elt vs (1+ i))))
	     (if (eq (elt vs i) :xscale) (setq xscale (elt vs (1+ i))))
	     (if (eq (elt vs i) :xoffset) (setq xoffset (elt vs (1+ i))))
	     (if (eq (elt vs i) :y2tics) (setq y2tics (elt vs (1+ i))))
	     (if (eq (elt vs i) :y2range) (setq y2range (elt vs (1+ i))))
             (if (eq (elt vs i) :type) (setq type (elt vs (1+ i))))
	     )
	   (setq vs (remove :range vs))
	   (setq vs (remove range vs :test #'equal))
	   (setq vs (remove :xrange vs))
	   (setq vs (remove xrange vs :test #'equal))
	   (setq vs (remove :yrange vs))
	   (setq vs (remove yrange vs :test #'equal))
	   (setq vs (remove :title vs))
	   (setq vs (remove title vs :test #'equal))
	   (setq vs (remove :clear vs))
	   (setq vs (remove clear vs :test #'equal))
	   (setq vs (remove :line-width vs))
	   (setq vs (remove line-width vs :test #'equal))
	   (setq vs (remove :direction vs))
	   (setq vs (remove direction vs :test #'equal))
           (setq vs (remove :xscale vs))
           (setq vs (remove xscale vs :test #'equal))
           (setq vs (remove :xoffset vs))
           (setq vs (remove xoffset vs :test #'equal))
           (setq vs (remove :y2tics vs))
           (setq vs (remove y2tics vs :test #'equal))
           (setq vs (remove :y2range vs))
           (setq vs (remove y2range vs :test #'equal))
           (setq vs (remove :type vs))
           (setq vs (remove type vs :test #'equal))
	   ;;
	   (if clear (send self :clear))
           (case type
                 (:lines ;; default
                  (when y2tics
                    (format strm "set y2tics~%")
                    (if y2range (format strm "set y2range [~A:~A]~%" (first y2range) (second y2range))))
                  (format strm "plot ")
                  (if (setq range (or range xrange))
                      (format strm "[~A:~A]" (first range) (second range))
                    (format strm "[]"))
                  (if yrange (format strm "[~A:~A]" (first yrange) (second yrange)))
                  (format strm " '-'")
                  (if title (format strm " title \"~A\"" (pop title)))
                  (format strm " w lp lw ~A" line-width)
                  (if (pop y2tics) (format strm " axis x1y2 "))
                  (dolist (v (cdr vs))
                    (format strm ", '-'")
                    (if title (format strm " title \"~A\"" (pop title)))
                    (format strm " w lp lw ~A" line-width)
                    (if (pop y2tics) (format strm " axis x1y2 "))
                    )
                  (format strm "~%")	   
                  (dolist (v vs)
                    (dotimes (i (length v))
                      (if (eq direction :left)
                          (format strm "~A ~A~%" (+ (* i xscale) xoffset) (elt v (1- (- (length v) i))))
                        (format strm "~A ~A~%" (+ (* i xscale) xoffset) (elt v i))))
                    (format strm "e~%"))
                  )
                 (:2dmap
                  (format strm "set pm3d map~%")
                  (format strm "unset ztics~%")
                  (if yrange (format strm "set cbrange [~A:~A]~%" (elt yrange 0) (elt yrange 1))
                    (format strm "set autoscale cb~%"))
                  (format strm "splot '-' with pm3d~%")
                  
                  (dotimes (i (length (car vs)))
                    (dotimes (ii 2)
                      (dotimes (j (length vs))
                        (dotimes (jj 2)
                          (let ((x (+ (* (+ i ii) xscale) xoffset))
                                (y (+ j jj)) ;; 0 1 1 2 2 3 ...
                                (z (if (eq direction :left) (elt (elt vs j) (1- (- (length (car vs)) i))) (elt (elt vs j) i)))
                                )
                            (format strm "~A ~A ~A~%" x y z)
                            )))
                      (format strm "~%")
                      ))
                  
                  (format strm "e~%")
                  )
                 (t (warn "unknown type ~A~%" type))
                 )
	   ))
  (:save (f &key (type "postscript eps color \"Times-Roman\" 24"))
       "save graph as eps file"
	 (format strm "set terminal ~A~%" type)
	 (format strm "set output ~s~%" f)
	 (if last-command (send-lexpr self :draw last-command)
	   (format strm "replot~%"))
	 (format strm "set output~%")
	 (format strm "set terminal x11~%")
	 (if last-command (send-lexpr self :draw last-command)
	   (format strm "replot~%"))
	 )
  (:replot () (format strm "replot~%"))
  (:reset () (format strm "reset~%"))
  (:command (msg) (format strm "~A~%" msg))
  (:quit () (format strm "quit~%"))
  ;; online processing
  (:proc-length (&optional n) (if n (setq data-length n)) data-length)
  (:proc-clear ()
	       (setq data nil))
  (:proc-one (vs &rest args)
	     (let ()
	       (if (< (length data) (length vs))
		   (while (< (length data) (length vs))
		     (setq data (append data (list nil)))))
	       (dotimes (i (length vs))
		 (push (elt vs i) (elt data i)))
	       (dotimes (i (length vs))
		 (if (>= (length (elt data i)) data-length)
		     (setf (elt data i) (butlast (elt data i) (- (length (elt data i)) data-length)))))
	       (setq last-command (append data args))
	       (send-lexpr self :draw last-command)
	       last-command))
  )

(defun gnuplot (&key (host (unix:gethostname)))
  "Returns gnuplot interface instance
ex)
(setq *g* (gnuplot))
(send *g* :draw #f(0 1 2 3 4 5) #f(5 4 3 2 1 0) :xrange '(0 10) :yrange '(0 10) :title '(\"data1\" \"data2\"))

see irteus/gnuplotlib.l for more info
"
  (INSTANCE gnuplot :init host))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; plot function for 2d or 3d plot
;; argument 
;;   ordinate-list ;; list of data for ordinate axis
;;     2D = (list (list y00 y01 ... y0n), ... (list ym0 ym1 ... ymn))
;;     3D = (list (list z00 z01 ... z0n), ... (list zm0 zm1 ... zmn))
;;   abscissa-list ;; list of data for abscissa axes
;;     2D = (list x0 x1 ... xn)
;;     3D = (list xylist0 ... xylistn) ;; xylist = (list x y)
;;   keylist ;; list of data's key
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun user::graph-view
  (ordinate-list
   &optional (abscissa-list (let ((idx -1)) (mapcar #'(lambda (x) (incf idx)) (make-list (length (car ordinate-list)))))) ;; range function
   &key (title "Graph") (xlabel "X") (ylabel "Y") (zlabel "Z")
        (dump-graph nil) (graph-fname (format nil "~A.eps" (substitute #\_ (elt " " 0) title)))
	;;(mode "points")
        (mode "lines")
	keylist xrange yrange zrange
	x11 additional-func
	no-dump ((:graph-instance gp) (if (boundp 'user::*gp*) user::*gp* (setq user::*gp* (gnuplot))))
	(fname (format nil "data~A" (sys::address gp))))
  "plot function for 2d or 3d plot
ordinate-list : list of data for ordinate axis
     2D = (list (list y00 y01 ... y0n), ... (list ym0 ym1 ... ymn))
     3D = (list (list z00 z01 ... z0n), ... (list zm0 zm1 ... zmn))
abscissa-list : list of data for abscissa axes
     2D = (list x0 x1 ... xn)
     3D = (list xylist0 ... xylistn) ;; xylist = (list x y)
:title : title of graph
:xlabel, :ylabel, zlabel : label for each axis
:keylist : legend of each data
:xrange, :yrange, :zrange : range of each axis
:mode : \"lines\" or \"points\"
"
  (labels ((gen-range-string
            (range)
            (if range (format nil "[~A:~A]" (car range) (cadr range)) "[]"))
           (2d-or-3d (r-2d r-3d) (if (atom (car abscissa-list)) (eval r-2d) (eval r-3d))))
    (unless keylist (setq keylist (let ((idx -1)) (mapcar #'(lambda (x) (incf idx)) (make-list (length ordinate-list))))))
    ;; dump dat file
    (unless no-dump
      (with-open-file
       (f (format nil "/tmp/~A.dat" fname) :direction :output)
       (format f (2d-or-3d "# x vals..~%" "# x y vals..~%"))
       (dotimes (i (length abscissa-list))
         (if (atom (car abscissa-list))
             (format f "~A " (elt abscissa-list i))
           (format f "~A ~A " (elt (elt abscissa-list i) 0) (elt (elt abscissa-list i) 1)))
         ;;(dolist (d ordinate-list) (format f "~A " (elt d i)))
	 (dolist (d ordinate-list)
	   (if (< i (length d))
	       (format f "~A " (elt d i))))
         (format f "~%")
         )
       )
      )
    
    ;; plot
    (mapcar #'(lambda (d1 d2)
                (send gp :command (format nil "set ~A \"~A\"" d1 d2)))
            '(user::title user::xlabel user::ylabel user::zlabel)
            (list title xlabel ylabel zlabel))
    (if additional-func (funcall additional-func))
    (dotimes (i (length ordinate-list))
      (send gp :command
            (format nil "~A \"/tmp/~A.dat\" using ~A title \"~A\" with ~A"
                    (case
                     i
                     (0 (apply #'format
                               (list nil
                                     (2d-or-3d "plot ~A ~A" "splot ~A ~A ~A")
                                     (gen-range-string xrange)
                                     (gen-range-string yrange)
                                     (2d-or-3d nil (gen-range-string zrange)))))
                     (t "replot"))
                    fname
                    (format nil "~A:~A" (2d-or-3d "1" "1:2") (+ i (2d-or-3d 2 3)))
                    (elt keylist i)
                    mode))
      )
    (if x11 (send gp :command "set terminal X11"))
    (when dump-graph
      (unix:usleep 200000)
      (send gp :save graph-fname)
      (unix:usleep 200000))
    ))

#|
;; examples 
(defun range (n) (do ((i 0 (incf i)) (ret nil)) ((>= i n) (nreverse ret)) (push i ret)))
(defun test-gnuplot-0 ()
  (unless (boundp '*gp*) (setq *gp* (gnuplot)))
  (let ((leng 360))
    (send *gp* :proc-length leng)
    (mapcar #'(lambda (s-data c-data)
		(send *gp* :proc-one (float-vector s-data c-data)
		      :direction :left
		      :title (list "sin(x)" "cos(x)")))
	    (mapcar #'(lambda (x) (sin (deg2rad x))) (range leng))
	    (mapcar #'(lambda (x) (cos (deg2rad x))) (range leng)))
    t))

;; examples for graph-view
(defun test2d-plot ()
  (graph-view (list (mapcar #'(lambda (x) (sin (deg2rad x))) (range 360)))
	      (range 360)
	      :xrange '(0 360) :yrange '(-2 2)
	      :ylabel "sin(x)" :title "plot sin test")
  )

(defun all-combination (lst)
 (reduce #'(lambda (prev target)
            (if (eq prev :nil)
                (progn
                  (mapcar #'list target))
              (progn
                (let ((ret nil))
                  (dolist (ta target)
                    (dolist (p prev)
                      (push (append p (list ta)) ret))
                    )
                  ret))))
        lst :initial-value :nil))
(defun test3d-plot ()
  (let ((xy-list (all-combination (list (range 100) (range 100)))))
    (graph-view (list (gaussian-frequency-function
		       (mapcar #'(lambda (x) (concatenate float-vector x)) xy-list)))
		xy-list
		:title "plot gauss test")
    ))
|#
