;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: $
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

(in-package "GEOMETRY")

(export '(stl2eus eus2stl))

;;
;; converting .stl file format object to eus
;;
(defun stl2eus (fname &key debug (color #f(0.8 0.8 0.8))
                      (coords) (scale 1000) (name "stlobject")
                      (use-edge t) (use-normal nil)
                      (return-type :faceset))
  (let ((str (make-string 80))
        afaceset fd size faces edges normals
        ht-norms ht-edgs ht-vrtx tmp vert-lst aface vs)

    (setq ht-norms (make-hash-table :test #'equal))
    (setq ht-edgs (make-hash-table :test #'equal))
    (setq ht-vrtx (make-hash-table :test #'equal))

    (with-open-file
     (f fname)

     (setq fd (send f :infd))
     (unix:uread fd str 80)

     (unix:uread fd str 4)
     (setq size (sys:peek str 0 :integer))

     (dotimes (i size)
       (setq tmp nil)
       (dotimes (j 4) ;; normal, vertex x 3
         (unix:uread fd str 12)
         (push (float-vector (sys:peek str 0 :float)
                             (sys:peek str 4 :float)
                             (sys:peek str 8 :float)) tmp)
         )
       (unix:uread fd str 2)
       (nreverse tmp)

       (setq vert-lst (mapcar #'(lambda (v)
                              (if scale (scale scale v v))
                              (if coords (send coords :rotate-vector v) v))
                          (cdr tmp)))

       ;; 2010/05/28 k-okada
       ;; vertices in (send <faceset-object> :faces) must be exactry same object
       ;; with (send <faceset-object) :vertices) when you call :transform or :move-to
       ;; FIXME : integrate with obj2eus
       (if (or use-normal use-edge (memq return-type '(:faceset :body))) ;; if hash tables are used
	   (dotimes (i (length vert-lst))
	     (if (gethash (elt vert-lst i) ht-vrtx)
		 (setf (elt vert-lst i) (gethash (elt vert-lst i) ht-vrtx)))
	     (setf (gethash (elt vert-lst i) ht-vrtx) (elt vert-lst i))))

       (push (car tmp) normals)
       (if use-normal
         (mapcar #'(lambda (v)
                     (sethash (list v vert-lst) ht-norms (car tmp)))
                 vert-lst))

       (setq aface (instance face :init :vertices vert-lst))
       (push aface faces)

       (when use-edge
         (let (f ee)
           (dolist (e (send aface :edges))
             (if (setq f (gethash (cons (e . nvert) (e . pvert)) ht-edgs))
                 (progn ;; if edge is in hash
                   (setq ee
                     (find e (send f :edges) ;; search same edge in f
                           :test #'(lambda (e1 e2)
                                     (and (v= (e1 . pvert) (e2 . nvert))
                                          (v= (e2 . pvert) (e1 . nvert))))))
                   (setf (elt (send aface :edges)
                              (position e (send aface :edges))) ee) ;; e <- ee
                   (when ht-vrtx
                     (setq (ee . pvert) (gethash (ee . pvert) ht-vrtx))
                     (setq (ee . nvert) (gethash (ee . nvert) ht-vrtx)))
                   (setq (ee . nface) aface))
               (progn ;; if edge is not in hash
                 (setf (gethash (cons (e . pvert) (e . nvert)) ht-edgs) aface)
                 (push e edges))
               )
             )))
       ) ;;/dotimes
     )
      ;;
    (if use-edge
        (nreverse edges)
      (setq edges (list t)))

    (case
     return-type
     ((:faceset :body)
      ;;
      (setq afaceset
        (instance (case return-type (:body body) (t faceset))
          :init :faces faces ;; faces
          :edges edges       ;; edges
          :vertices (send ht-vrtx :list-values)
          :primitive (list ':cube 100 100 100)) ;; ?
        )
      (send afaceset :name (format nil "~A" name))
      (setf (get afaceset :face-color) color)
      ;;(setf (get afaceset :gl-normal-jaggy) ht-norms) ;; ???
      (setf (get afaceset :gl-normal) ht-norms)
      ;;
      (unless use-normal
      ;;(unless t
        (when debug
          (format *error-output* ";; making normals..... ")
          (finish-output *error-output*))
        (dolist (f (send afaceset :faces))
          (setq vs (cdr (send f :vertices)))
          (dolist (v vs)
            (sethash (list v vs) ht-norms (f . model-normal))))
        (when debug (format *error-output* "done~%"))
        ))
     (:3dpointcloud
      (setq afaceset
        (instance 3dpointcloud :init :points
                  (remove-duplicates (flatten (send-all faces :vertices)))
                  )))
     (:glvertices
      ;;;
      (let* ((num (* 3 (length faces)))
             (vmat (make-matrix num 3))
             (nmat (make-matrix num 3))
             (cntr 0) n)
        (dolist (af faces)
          ;;(pprint af)
          (let ((vs (send af :vertices))
                (n (af . model-normal)))
            (c-matrix-row vmat cntr (elt vs 0) t)
            (c-matrix-row nmat cntr n t)
            (incf cntr)
            (c-matrix-row vmat cntr (elt vs 1) t)
            (c-matrix-row nmat cntr n t)
            (incf cntr)
            (c-matrix-row vmat cntr (elt vs 2) t)
            (c-matrix-row nmat cntr n t)
            (incf cntr)))
        (setq afaceset
              (instance gl::glvertices :init
                        (list (list
                               (list :vertices vmat)
                               (list :normals nmat)
                               (list :material (list (list :color color)))
                               (list :type :triangles)))
                              :filename fname))
        ))
     (:faces (setq afaceset faces))
     (t
      (warn ";;unknown return-type ~A~%" return-type)))
    afaceset))

(defun write-stl-header (f face-num)
  (let ((str (make-string 80)))
    (replace str "generated from euslisp")
    (unix:write f str 80)
    )
  (let ((str (make-string 4)))
    (sys:poke face-num str 0 :integer)
    (unix:write f str 4))
  )

(defun write-stl-binary-float-val (f val)
  (let ((str (make-string 4)))
    (sys::poke val str 0 :float)
    (unix:write f str 4)))

(defun write-stl-face (strm f &key (scale 0.001))
  (let ((normal (send f :normal))
        (vertices (cdr (send f :vertices))))
    (if (not (= (length vertices) 3))
        (error "face must be a triangle"))
    (write-stl-binary-float-val strm (elt normal 0))
    (write-stl-binary-float-val strm (elt normal 1))
    (write-stl-binary-float-val strm (elt normal 2))
    (dolist (v vertices)
      (write-stl-binary-float-val strm (* scale (elt v 0)))
      (write-stl-binary-float-val strm (* scale (elt v 1)))
      (write-stl-binary-float-val strm (* scale (elt v 2)))
      )
    (let ((dummy (make-string 2)))
      (unix:write strm dummy 2))
    ))

(defun eus2stl (fname _faceset &key (scale 0.001))
  (send _faceset :worldpos)
  (with-open-file (f fname :direction :output)
    (let ((faces (send _faceset :faces)))
      (write-stl-header f (length faces))
      (dolist (_face faces)
        (write-stl-face f _face :scale scale))))
  fname)

;;
;; Reading dump files from SolidEdge
;; Dump files made by SolidEdge macro
;;
(defun read-files-from-solidedge (fname &key (scale))
  (let ((assembly-config (with-open-file (f fname) (read f))))
    (read-assembly-config assembly-config :scale scale)))

(defun read-assembly-config (cfg &key (scale) (coords))
  (let (ret)
    (dolist (elm cfg)
      (case
       (car elm)
       (:parts
        (let* ((ang (cdr (assoc :angle elm)))
               (cd (make-coords :pos (cdr (assoc :trans elm))
                                :rot (rpy-matrix (elt_z ang) (elt_y ang) (elt_x ang))))
               (bdy (stl2eus (cdr (assoc :file elm))
                             :scale scale :name (cdr (assoc :name elm))))
               )
          (if coords (send cd :transform coords :world)) ;;??
          (send bdy :newcoords cd)
          ;;(send bdy :transform cd)
          ;;(send bdy :worldcoords)
          ;;(pprint (list bdy cd))
          (push bdy ret)
          ))
       (:assembly
        ;; (assoc :name elm)
        (let* ((ang (cdr (assoc :angle elm)))
               (cd (make-coords :pos (cdr (assoc :trans elm))
                                :rot (rpy-matrix (elt_z ang) (elt_y ang) (elt_x ang))))
               (sub (assoc :subassembly elm))
               )
          (if sub
              (push (read-assembly-config
                     (cdr sub) :coords (if coords (send cd :transform coords :world) cd))
                    ret))
          ))
       (t (warn "unknown element ~A~%" (car elm)))
       )
      )
    (nreverse ret)
    )
  )

;;
;; (cd "stlsample")
;; (setq bds (read-files-from-solidedge "lisp.txt"))
;; (pickview)
;; (objects (bds))
;;

;;
;; For large stl
;;
(defun stl-make-faces (fname &key (scale)(coords))
  (let ((str (make-string 80))
        tmp vert-lst aface faces fd size)

    (with-open-file
     (f fname)

     (setq fd (send f :infd))
     (unix:uread fd str 80)
     (format t "stl : ~S~%" str)
     (unix:uread fd str 4)
     (setq size (sys:peek str 0 :integer))
     (format t "size : ~A~%" size)
     (dotimes (i size)
       (setq tmp nil)
       (dotimes (j 4) ;; normal, vertex x 3
         (unix:uread fd str 12)
         (push (float-vector (sys:peek str 0 :float)
                             (sys:peek str 4 :float)
                             (sys:peek str 8 :float)) tmp)
         )
       (unix:uread fd str 2)
       (nreverse tmp)

       (setq vert-lst (mapcar #'(lambda (v)
                                (if scale (scale scale v v))
                                (if coords (send coords :rotate-vector v) v))
                            (cdr tmp)))

       (setq aface (instance face :init :vertices vert-lst))
       (push aface faces)
       )) ;;/dotimes

    faces))

(defun safe-vmax (lst)
  (safe-vmaxmin lst :func #'vmax))
(defun safe-vmin (lst)
  (safe-vmaxmin lst :func #'vmin))
(defun safe-vmaxmin (lst &key (func #'vmax) (size 32000)) ;; max stack is 65535
  (let (ret (len (length lst)))
    (if (< len size)
        (return-from safe-vmaxmin (apply func lst)))
    (setq ret (car lst))
    (dotimes (i (/ len size))
      (setq ret (apply func ret (subseq lst (* i size) (* (1+ i) size)))))
    (if (= (mod len size) 0)
        ret
      (apply func ret (subseq lst (* size (/ len size)) len)))
    ))

(defun devide-faces (fcs &key (threshould 0.0) (axis :x))
  (let ((pos (case axis (:x 0) (:y 1) (:z 2) (t axis)))
        ret0 ret1)
    (dolist (f fcs)
      (let ((v (apply #'vmax (send f :vertices))))
        (if (> (elt v pos) threshould)
            (push f ret0)
          (push f ret1))
        ))
    (list ret0 ret1)
    ))

(defun make-boundingbox-from-faces (fcs)
  (let (min-lst max-lst)
    (dolist (f fcs)
      (push (apply #'vmax (send f :vertices)) max-lst)
      (push (apply #'vmin (send f :vertices)) min-lst))
    (list (safe-vmax max-lst)
          (safe-vmin min-lst))
    ))

(defun small-square-axis (b-box)
  (let* ((v (apply #'v- b-box))
         (sz (* (elt_x v) (elt_y v)))
         (sy (* (elt_x v) (elt_z v)))
         (sx (* (elt_y v) (elt_z v)))
         (lst (list sx sy sz))
         )
    (position (apply #'min lst) lst :test #'=)
  ))

(defun make-devided-faces-lists (fcs &key (size 32000))
  (let ((fcs-lst (list fcs)))
    (while (> (length (car fcs-lst)) size)
      (let* ((b-box (make-boundingbox-from-faces (car fcs-lst)))
             (pos (small-square-axis b-box))
             (dvs (devide-faces (car fcs-lst)
                                :threshould (/ (elt (apply #'v+ b-box) pos) 2.0)
                                :axis pos)))
        (setq fcs-lst (nconc dvs (cdr fcs-lst)))
        (sort fcs-lst #'(lambda (x y) (>= (length x) (length y))))
        )
      )
    fcs-lst))

(defun make-faceset-from-faces (fcs) ;; vertex, edge, normal -> downsizing by remove duplicate
  (let ((ht-vrtx (make-hash-table :test #'equal))
        (ht-edgs (make-hash-table :test #'equal))
        edges
        ht-normal)
    ;; check vertices
    (dolist (f fcs)
      (let ((vtx (send f :vertices)))
        (dotimes (i (length vtx))
          (let ((v (elt vtx i)))
            (setf (elt (f . vertices) i)
                  (sethash v ht-vrtx v))
            )))
      )
    ;; check edges
    (dolist (aface fcs)
      (let (f ee)
        (dolist (e (send aface :edges))
          (if (setq f (gethash (cons (e . nvert) (e . pvert)) ht-edgs))
              (progn ;; if edge is in hash
                (setq ee
                  (find e (send f :edges) ;; search same edge in f
                        :test #'(lambda (e1 e2)
                                  (and (v= (e1 . pvert) (e2 . nvert))
                                       (v= (e2 . pvert) (e1 . nvert))))))
                (setf (elt (send aface :edges)
                           (position e (send aface :edges))) ee) ;; e <- ee
                (when ht-vrtx
                  (setq (ee . pvert) (gethash (ee . pvert) ht-vrtx))
                  (setq (ee . nvert) (gethash (ee . nvert) ht-vrtx)))
                (setq (ee . nface) aface))
            (progn ;; if edge is not in hash
              (setf (gethash (cons (e . pvert) (e . nvert)) ht-edgs) aface)
              (push e edges))
            )
          )))
    ;; check normals
    ;; nothing
    (instance faceset :init :faces fcs
              :edges edges
              :vertices (send ht-vrtx :list-values)
              :primitive (list ':cube 100 100 100))
    ))

(defun stl2eus-large (fname)
  (let* ((fcs (stl-make-faces fname))
         (dev-fcs (make-devided-faces-lists fcs)))
    (mapcar #'make-faceset-from-faces dev-fcs)
    ))

;; (setq dev-fcs (make-devided-faces-lists (stl-make-faces "/home/leus/2nd-spine.stl")) a nil)
;; (mapcar #'(lambda (x) (length x)) dev-fcs)


(provide :irtcad "$Id: ")
