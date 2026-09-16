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
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright notice,
;;;   this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright notice,
;;;   this list of conditions and the following disclaimer in the documentation
;;;   and/or other materials provided with the distribution.
;;; * Neither the name of JSK Robotics Laboratory, The University of Tokyo
;;;   (JSK) nor the names of its contributors may be used to endorse or promote
;;;   products derived from this software without specific prior written
;;;   permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
;;; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
;;; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
;;; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
;;; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
;;; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
;;; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;;; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
;;; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;

(require :irtmath)

(in-package "GEOMETRY")

(export '(body-to-faces body-to-triangles midcoords orient-coords-to-axis bodyset *g-vec*
                        make-sphere make-ring make-fan-cylinder x-of-cube y-of-cube z-of-cube
                        height-of-cylinder radius-of-cylinder radius-of-sphere
			matrix-to-euler-angle))

(defvar *g-vec* (float-vector 0 0 9806.65)) ;; [mm/s^2]

(defun midcoords (p c1 c2)
  "Returns mid (or p) coordinates of given two cooridnates c1 and c2"
  (let ()
    (make-coords :pos (midpoint p (send c1 :worldpos) (send c2 :worldpos))
		 :rot (user::midrot p (send c1 :worldrot) (send c2 :worldrot)))
    ))

(defmethod line
  (:worldcoords ()
    "Return a coordinates on the midpoint of the end points"
    (make-coords :pos (midpoint 0.5 nvert pvert))))

(defmethod coordinates
  (:axis (axis)
	 (send self :rotate-vector
	       (case axis
		     (:x #f(1 0 0))
		     (:y #f(0 1 0))
		     (:z #f(0 0 1))))
	 )
  (:difference-position
   (coords &key (translation-axis t))
   "return difference in position of given coords, translation-axis can take (:x, :y, :z, :xy, :yz, :zx)."
   (let ((dif-pos
	  (send self :inverse-transform-vector (send coords :worldpos))))
     (case
      translation-axis
      ((:x :xx) (setf (elt dif-pos 0) 0))
      ((:y :yy) (setf (elt dif-pos 1) 0))
      ((:z :zz) (setf (elt dif-pos 2) 0))
      ((:xy :yx) (setf (elt dif-pos 0) 0) (setf (elt dif-pos 1) 0))
      ((:yz :zy) (setf (elt dif-pos 1) 0) (setf (elt dif-pos 2) 0))
      ((:zx :xz) (setf (elt dif-pos 2) 0) (setf (elt dif-pos 0) 0)))
     dif-pos))
  (:difference-rotation
   (coords &key (rotation-axis t))
   "return difference in rotation of given coords, rotation-axis can take (:x, :y, :z, :xx, :yy, :zz, :xm, :ym, :zm)"
   (labels
    ((need-mirror-for-nearest-axis
      (coords0 coords1 axis)
      (let* ((a0 (send coords0 :axis axis))
             (a1 (send coords1 :axis axis))
             (a1m (v- a1))
             (dr1 (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))
             (dr1m (scale (acos (v. a0 a1m)) (normalize-vector (v* a0 a1m)))))
        (< (norm dr1) (norm dr1m)))))
    (let (dif-rotmatrix dif-rot a0 a1)
      (case
       rotation-axis
       ((:x :y :z)
        (setq a0 (send self :axis rotation-axis)
              a1 (send coords :axis rotation-axis))
        (setq dif-rot 
              (transform (transpose (send self :worldrot))
                         (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))))
       ((:xx :yy :zz)
        (let ((axis (case rotation-axis (:xx :x) (:yy :y) (:zz :z))) a0 a2)
          (setq a0 (send self :axis axis))
          (setq a2 (send coords :axis axis))
          (unless (need-mirror-for-nearest-axis self coords axis) (setq a2 (v- a2)))
          (setq dif-rot (transform (transpose (send self :worldrot))
                                   (scale (acos (v. a0 a2)) (normalize-vector (v* a0 a2)))))))
       ((:xm :ym :zm)
        (let ((rot (send coords :worldrot)))
          (unless (need-mirror-for-nearest-axis self coords (case rotation-axis (:xm :y) (:ym :z) (:zm :x)))
            (setq rot (rotate-matrix rot pi (case rotation-axis (:xm :x) (:ym :y) (:zm :z)))))
          (setq dif-rotmatrix (m* (transpose (send self :worldrot)) rot))
          (setq dif-rot (user::matrix-log dif-rotmatrix))
          ))
       (nil
        (setq dif-rot (float-vector 0 0 0)))
       (t
        (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
        (setq dif-rot (user::matrix-log dif-rotmatrix))
        ))
      dif-rot)))
  )


;; BEGIN:: Redefine for rotm3*
(defmethod coordinates
  (:move-to (c &optional (wrt :local) &aux cc)
     (unless (coordinates-p c) (error "coordinates expected for :move-to"))
     (cond ((or (memq wrt '(:local local)) (eq wrt self))
	    (setq cc (transform-coords self c))
	    (send self :newcoords cc))
	   ((or (memq wrt '(:parent parent :world world))
		(equal wrt *world-coords*))
	    (send self :newcoords c))
	   ((coordinates-p wrt)
	    (setq cc (transform-coords (send wrt :worldcoords) c))
	    (transform-coords (send (send self :parentcoords)
				    :inverse-transformation) cc cc)
	    (send self :newcoords cc))))
  (:transformation   (c2 &optional (wrt :local))
   (setq c2 (send c2 :worldcoords)) 
   (let* ((c1 (send self :worldcoords)) 
	  (inv (send c1 :inverse-transformation))
	  xw)
      (cond
         ((or (memq wrt '(:local local)) (eq wrt self))
	   (transform-coords inv c2 inv))
         ((or (memq wrt '(:parent parent :world world))
	      (eq wrt *world-coords*))
	   (transform-coords c2 inv inv))
         ((coordinates-p wrt)
	   (setq xw (send wrt :worldcoords))
	   (transform-coords c2 inv inv)
	   (transform-coords (send xw :inverse-transformation) inv inv)
	   (transform-coords inv xw inv))
         (t (send self :error ":transform wrt?" wrt)))
      inv))
  (:transform  (c &optional (wrt :local))
   (cond
      ((or (memq wrt '(local :local)) (eq wrt self))	;multiply c from the right
	(transform-coords self c self))
      ((or (memq wrt '(parent :parent world :world))
	   (eq wrt *world-coords*))
	(transform-coords c self self))	;;multiply c from the left
      ((coordinates-p wrt)
	(transform-coords (send wrt :inverse-transformation) self self)
	(transform-coords c self self)
	(transform-coords (send wrt :worldcoords) self self))
      (t (send self :error ":transform wrt?" wrt)))
   (send self :newcoords rot pos))
  (:move-coords
   (target at)
   "fix 'at' coords on 'self' to 'target'"
   (send self :transform (send at :transformation target) at)
   (send self :worldcoords)
   )
  )

(defmethod cascaded-coords 
 (:worldcoords  ()	;calculate rot and pos in the world
   (when changed
      (if parent
	  (transform-coords (send parent :worldcoords) self worldcoords)
	  (send worldcoords :replace-coords self))
      (send self :update)
      (setf changed nil))
   worldcoords)
 (:transformation (c2 &optional (wrt :local))
   (let* ((w2 (send c2 :worldcoords))
	  (w1 (send self :worldcoords))
	  (w1inv (send w1 :inverse-transformation))
	  (c1inv) (xw))
      (cond
        ((or (memq wrt '(:local local)) (eq wrt self))
	 (transform-coords w1inv w2 w1inv))
	((or (memq wrt '(:parent parent)) (eq wrt parent))	;nil?
	 (setq c1inv (send-super :inverse-transformation))
	 (transform-coords w2 c1inv c1inv)
	 (transform-coords w1inv c1inv c1inv)
	 (transform-coords self c1inv c1inv))
	((or (memq wrt '(world :world))  (equal wrt *world-coords*))
	 (transform-coords w2 w1inv w1inv))
        ((coordinates-p wrt)
	 (setq xw (send wrt :worldcoords))
	 (transform-coords w1inv xw w1inv)
	 (transform-coords w2 w1inv w1inv)
	 (transform-coords (send xw :inverse-transformation) w1inv w1inv))
      (t (send self :error ":transform wrt?" wrt)))))
 (:transform  (c &optional (wrt :local))
   (cond
    ((or (memq wrt '(:local local)) (eq wrt self))	;multiply c from the right
	(transform-coords self c self))
    ((or (memq wrt '(:parent parent)) (eq wrt parent))	;nil?
	(transform-coords c self self))	;;multiply c from the left
    ((or (memq wrt '(world :world))  (equal wrt *world-coords*))
	(let ((pc (send self :parentcoords)))
	   (transform-coords pc self self)
	   (transform-coords c self self)
	   (transform-coords (send pc :inverse-transformation) self self)))
    ((coordinates-p wrt)
	(let ((pc (send self :parentcoords)))
	   (transform-coords pc self self)
	   (transform-coords (send wrt :inverse-transformation) self self)
	   (transform-coords c self self)
	   (transform-coords (send wrt :worldcoords) self self)
	   (transform-coords (send pc :inverse-transformation) self self)))
    (t (send self :error ":transform wrt?" wrt)))
   (send self :newcoords rot pos))
 (:move-to (c &optional (wrt :local) &aux cc)
     (unless (coordinates-p c) (error "coordinates expected for :move-to"))
     (cond ((or (memq wrt '(:local local)) (eq wrt self))
	    (setq cc (transform-coords self c))
	    (send self :newcoords cc))
	   ((or (memq wrt '(:parent parent)) (eq wrt parent))
	    (send self :newcoords c))
	   ((or (memq wrt '(:world world)) (equal wrt *world-coords*))
	    (setq cc (transform-coords
			(send (send self :parentcoords)
				:inverse-transformation) c))
	    (send self :newcoords cc))
	   ((coordinates-p wrt)
	    (setq cc (transform-coords (send wrt :worldcoords) c))
	    (transform-coords (send (send self :parentcoords)
				    :inverse-transformation) cc cc)
	    (send self :newcoords cc))))
  )

(defun transform-coords (c1 c2 &optional
			    (c3 (let ((dim (send c1 :dimension)))
				  (instance coordinates 
				    :newcoords (unit-matrix dim)
				    (instantiate float-vector dim)))))
  (if (eq c1 c3)
      (v+ (coordinates-pos c1)
	  (transform (coordinates-rot c1) (coordinates-pos c2))
	  (coordinates-pos c3))
    (v+ (coordinates-pos c1)
	(transform (coordinates-rot c1) (coordinates-pos c2) (coordinates-pos c3))
	(coordinates-pos c3)))
  (user::rotm3* (coordinates-rot c1) (coordinates-rot c2) (coordinates-rot c3))
  c3)


;; END:: Redefine for rotm3*

;;;
(defmethod coordinates
  (:rotate-vector (v &optional r)
                  (if r (transform rot v r) (transform rot v)))
  (:inverse-rotate-vector (v &optional r)
                          (if r (transform v rot r) (transform v rot))))
(defmethod cascaded-coords
  (:rotate-vector (v &optional r)
                  (send (send self :worldcoords) :rotate-vector v r))
  (:inverse-rotate-vector (v &optional r)
                          (send (send self :worldcoords) :inverse-rotate-vector v r)))

(defmethod coordinates
  (:inverse-transform-vector
   (vec &optional v3a v3b m33)	;vec in world coordinates->local
   (let ((inv-rot (if m33 (transpose rot m33) (transpose rot))))
     (if (and v3a v3b m33)
	 (v- (transform inv-rot vec v3a) (transform inv-rot pos v3b) v3a)
       (v- (transform inv-rot vec) (transform inv-rot pos))))))
(defmethod cascaded-coords 
  (:inverse-transform-vector
   (v &optional v3a v3b m33)
   (send (send self :worldcoords) :inverse-transform-vector v v3a v3b m33)))


;; orient-coords-to-axis
;;   orient "axis" in "target-coords" to the direction specified by "v" destructively.
;;   "v" must be non-zero vector.
(defun orient-coords-to-axis (target-coords v &optional (axis :z) (eps *epsilon*))
  "orient 'axis' in 'target-coords' to the direction specified by 'v' destructively.
   'v' must be non-zero vector."
  (let* ((nv (normalize-vector v))
         (ax (send target-coords :rotate-vector ;; axis in the worldcoords
                   (case axis
                     (:x #f(1 0 0)) (:-x #f(-1 0 0))
                     (:y #f(0 1 0)) (:-y #f(0 -1 0))
                     (:z #f(0 0 1)) (:-z #f(0 0 -1))
                     (t axis))))
         (rot-axis (v* ax nv)) ;; axis to rotate ax -> nv
         (rot-angle-cos (v. nv ax))) ;; angle to rotate ax -> nv
    ;; check rot-angle-cos
    (cond
     ;; if th = 0[deg] -> no need to rotate target-coords
     ((eps= rot-angle-cos 1.0 eps)
      (return-from orient-coords-to-axis target-coords))
     ;; if th = 180[deg] -> previous rot-axis = 0 vector and rot-axis is ambiguous. overwrite rot-axis by the axis orthogonal to ax
     ((eps= rot-angle-cos -1.0 eps)
      (block :calc-for-th-180
        (dolist (rot-axis2 (list #f(1 0 0) #f(0 1 0)))
          (let ((rot-angle-cos2 (v. ax rot-axis2)))
            (unless (eps= (abs rot-angle-cos2) 1.0 eps)
              (setq rot-axis (v- rot-axis2 (scale rot-angle-cos2 ax))) ;; use only vertical component of rot-axis2 by removing parallel component of rot-axis2
              (return-from :calc-for-th-180 nil))
            ))))
     (t ))
    (send target-coords :rotate (acos rot-angle-cos) rot-axis :world)
    target-coords))

;;
;; bodyset class
;;
(defclass bodyset
  :super cascaded-coords
  :slots 
  ((bodies :type cons)
  ))

(defmethod bodyset
  (:init 
   (coords
    &rest args
    &key (name (intern (format nil "bodyset~A" (sys::address self)) "KEYWORD"))
    ((:bodies bs))
    &allow-other-keys)
   "Create bodyset object"
   (let ()
     (send-super* :init :name name args)
     (send coords :copy-worldcoords self)
     (setq bodies bs)
     (when bodies
       ;; モデリングにおける制約：bodiesの先頭にはツリーのルートを入れておく
       (send self :assoc (car bodies)))
     (send self :worldcoords)
     self))
  (:bodies (&rest args) (user::forward-message-to-all bodies args))
  (:faces () (flatten (send-all bodies :faces)))
  (:worldcoords
   ()
   (when changed
     (send-super :worldcoords)
     (dolist (b bodies) (send b :worldcoords)))
   worldcoords)
  (:draw-on (&rest args) (dolist (b bodies) (send* b :draw-on args)))
  )

;;
;; body->triangle
;;
(defun face-to-triangle-aux (f)
  "triangulate the face."
  (cond
   ((null f) nil)
   ((= (length (send f :edges)) 3) (list f))
   (t (face-to-triangle f))))

;;
;; face-to-triangle-aux is a hand-written ear-clipping algorithm
;; (bridging every hole into the outer contour, then clipping ears one
;; at a time, each validated against every remaining vertex): its cost
;; grows at least O(n^2) in the number of resulting vertices, which is
;; impractically slow for faces with many holes (a real CAD-derived
;; part with 24 holes/206 vertices took ~65s to triangulate this way).
;;
;; libGLU's sweep-line tessellator solves the same problem in a tiny
;; fraction of the time (the same face: ~0.003s), and is already
;; linked into eusgl for on-screen rendering -- but its GLU_BEGIN/
;; GLU_VERTEX/GLU_END callbacks were only ever wired to draw
;; immediately. eus_tess_reset/eus_tess_begin_cb/eus_tess_vertex_cb/
;; eus_tess_end_cb/eus_tess_error_cb (lisp/opengl/src/util.c,
;; registered the same way as alloctessinfo/tess_vertex_cb) record the
;; tessellated output into memory instead, so it can be read back as
;; data with no live GL context or open window required.
;;
;; Only the GL-enabled build (eusgl/irteusgl, via irtgl.l) links GLU at
;; all -- pqp.l's collision-model construction, which is what actually
;; benefits most from this, is also loaded by the plain (non-GL)
;; eus/irteus build, so face-to-triangle-fast below must degrade
;; safely to the original ear-clipping path when the collector isn't
;; available, rather than erroring.
(defvar *glu-tess-obj* nil)

(defun glu-tess-setup ()
  (unless *glu-tess-obj*
    (setq *glu-tess-obj* (gl::gluNewTess))
    (gl::gluTessCallbackl *glu-tess-obj* 100100 ;; GLU_BEGIN
			  (#'gl::eus_tess_begin_cb . entry) (#'gl::eus_tess_begin_cb . entry2))
    (gl::gluTessCallbackl *glu-tess-obj* 100101 ;; GLU_VERTEX
			  (#'gl::eus_tess_vertex_cb . entry) (#'gl::eus_tess_vertex_cb . entry2))
    (gl::gluTessCallbackl *glu-tess-obj* 100102 ;; GLU_END
			  (#'gl::eus_tess_end_cb . entry) (#'gl::eus_tess_end_cb . entry2))
    (gl::gluTessCallbackl *glu-tess-obj* 100103 ;; GLU_ERROR
			  (#'gl::eus_tess_error_cb . entry) (#'gl::eus_tess_error_cb . entry2))))

;; triangulates face f (which may have holes) via GLU, returning a
;; list of (#f(x0 y0 z0) #f(x1 y1 z1) #f(x2 y2 z2)) vertex triples, or
;; nil if GLU itself reported an error (caller should fall back to the
;; face object based method in that case).
(defun glu-triangulate-face (f)
  (glu-tess-setup)
  (gl::eus_tess_reset)
  (let* ((holes (send f :holes))
	 (l-tessinfo
	  (mapcar #'(lambda (p) (gl::alloctessinfo p 0 nil 0 nil))
		  (cdr (send f :vertices)))))
    (gl::gluTessBeginPolygon *glu-tess-obj* 0)
    (gl::gluTessBeginContour *glu-tess-obj*)
    (mapc #'(lambda (i) (gl::gluTessVertex *glu-tess-obj* i i)) l-tessinfo)
    (gl::gluTessEndContour *glu-tess-obj*)
    (dolist (hole holes)
      (let ((h-l-tessinfo
	     (mapcar #'(lambda (p) (gl::alloctessinfo p 0 nil 0 nil))
		     (send hole :vertices))))
	(gl::gluTessBeginContour *glu-tess-obj*)
	(gl::gluNextContour *glu-tess-obj* 100122) ;; GLU_INTERIOR
	(mapc #'(lambda (i) (gl::gluTessVertex *glu-tess-obj* i i)) h-l-tessinfo)
	(gl::gluTessEndContour *glu-tess-obj*)
	(nconc l-tessinfo h-l-tessinfo)))
    (gl::gluTessEndPolygon *glu-tess-obj*)
    (mapc #'unix:free l-tessinfo))
  (if (zerop (gl::eus_tess_get_error))
      (let* ((nv (let ((total 0))
		   (dotimes (b (gl::eus_tess_num_batches)) (incf total (gl::eus_tess_batch_count_at b)))
		   total))
	     (verts (make-array nv))
	     (tmp (float-vector 0 0 0)))
	(dotimes (i nv)
	  (gl::eus_tess_get_vertexfv i tmp)
	  (setf (aref verts i) (copy-seq tmp)))
	(let (triangles)
	  (dotimes (b (gl::eus_tess_num_batches))
	    (let* ((mode (gl::eus_tess_batch_mode_at b))
		   (start (gl::eus_tess_batch_start_at b))
		   (count (gl::eus_tess_batch_count_at b)))
	      (cond
	       ((= mode 4) ;; GL_TRIANGLES
		(do ((i 0 (+ i 3))) ((> (+ i 3) count))
		  (push (list (aref verts (+ start i)) (aref verts (+ start i 1)) (aref verts (+ start i 2))) triangles)))
	       ((= mode 6) ;; GL_TRIANGLE_FAN
		(let ((v0 (aref verts start)))
		  (do ((i 1 (1+ i))) ((>= (1+ i) count))
		    (push (list v0 (aref verts (+ start i)) (aref verts (+ start i 1))) triangles))))
	       ((= mode 5) ;; GL_TRIANGLE_STRIP
		(do ((i 0 (1+ i))) ((>= (+ i 2) count))
		  (if (evenp i)
		      (push (list (aref verts (+ start i)) (aref verts (+ start i 1)) (aref verts (+ start i 2))) triangles)
		    (push (list (aref verts (+ start i 1)) (aref verts (+ start i)) (aref verts (+ start i 2))) triangles))))
	       (t (warn ";; glu-triangulate-face: unknown primitive mode ~A~%" mode)))))
	  (nreverse triangles)))
    nil))

(defun triangle-to-face (v0 v1 v2)
  (make-face-from-vertices (list (cons v0 nil) (cons v1 nil) (cons v2 nil))))

;; computed once: does this build actually link GLU with the
;; tessellation-collector callbacks (only true for eusgl/irteusgl,
;; built from a core that has them -- see lisp/opengl/src/util.c)?
(defparameter *glu-tess-available*
  (and (fboundp 'gl::eus_tess_reset) (fboundp 'gl::gluNewTess)))

(defun face-to-triangle-fast (f)
  "Drop-in replacement for face-to-triangle-aux that triangulates via
libGLU when available (see the comment above), falling back to the
original ear-clipping implementation when it isn't (a non-GL build) or
when GLU reports an error for this particular face."
  (cond
    ((null f) nil)
    ((= (length (send f :edges)) 3) (list f))
    (t (let ((tris (and *glu-tess-available* (glu-triangulate-face f))))
	 (if tris
	     (mapcar #'(lambda (tr) (apply #'triangle-to-face tr)) tris)
	   (face-to-triangle-aux f))))))

;; --- face-to-triangle-delaunay: a from-scratch, dependency-free 2D
;; Delaunay triangulation (Bowyer-Watson incremental algorithm), added
;; purely to compare mesh quality against face-to-triangle-aux
;; (ear-clipping) and face-to-triangle-fast (GLU sweep-line) above --
;; not used by pqp.l or irtgl.l, and not intended as a third
;; production triangulation option. Delaunay triangulation maximizes
;; the minimum angle over all triangulations of a point set (the
;; "empty circumcircle" property), so it tends to avoid the long thin
;; slivers ear-clipping and GLU can both produce; see the area-quality
;; comparison test in test/test-triangulation.l.
;;
;; This is a plain double-precision implementation, not one using
;; adaptive/exact arithmetic predicates the way production Delaunay
;; libraries (e.g. Shewchuk's Triangle) do, and it is O(n^2) (no
;; spatial acceleration structure) -- it exists for the mesh-quality
;; comparison, not for speed: it is markedly slower than
;; face-to-triangle-aux on non-symmetric inputs and utterly
;; uncompetitive with face-to-triangle-fast (GLU), which remains the
;; right choice for pqp.l/irtgl.l's actual use case.

(defun cross2d (o a b)
  (- (* (- (car a) (car o)) (- (cadr b) (cadr o)))
     (* (- (cadr a) (cadr o)) (- (car b) (car o)))))

(defun tri-area-2d (tr)
  (abs (/ (cross2d (first tr) (second tr) (third tr)) 2.0)))

(defun ccw3 (a b c)
  ;; reorder a b c to be counter-clockwise (positive signed area)
  (if (> (cross2d a b c) 0) (list a b c) (list a c b)))

(defun in-circumcircle (a b c p)
  ;; a b c assumed CCW. t if p is inside their circumcircle.
  (let* ((ax (- (car a) (car p))) (ay (- (cadr a) (cadr p)))
         (bx (- (car b) (car p))) (by (- (cadr b) (cadr p)))
         (cx (- (car c) (car p))) (cy (- (cadr c) (cadr p)))
         (det (+ (* ax (- (* by (+ (* cx cx) (* cy cy))) (* cy (+ (* bx bx) (* by by)))))
                 (* (- ay) (- (* bx (+ (* cx cx) (* cy cy))) (* cx (+ (* bx bx) (* by by)))))
                 (* (+ (* ax ax) (* ay ay)) (- (* bx cy) (* by cx))))))
    (> det 1e-9)))

(defun edge-eq (e1 e2)
  (or (and (eq (first e1) (first e2)) (eq (second e1) (second e2)))
      (and (eq (first e1) (second e2)) (eq (second e1) (first e2)))))

;; symmetric input (e.g. an evenly-spaced ring of holes) feeds
;; Bowyer-Watson several co-circular points back to back, which is
;; exactly where its plain double-precision in-circumcircle test is
;; least reliable (observed: up to ~15% spurious extra area on an
;; 8-hole ring). Randomized incremental insertion order is the
;; standard textbook mitigation; face-to-triangle-delaunay below
;; reorders deterministically (multiplicative hash by a prime coprime
;; with n) instead of with actual randomness, so a result -- and any
;; test asserting against it -- stays reproducible, and retries with a
;; different prime (checking against the true, directly-computed
;; polygon area each time) on the rare input where one ordering still
;; isn't enough.
(defun deterministic-shuffle (lst prime)
  (let* ((n (length lst)) (i -1)
         (pairs (mapcar #'(lambda (x) (cons (mod (* (incf i) prime) n) x)) lst)))
    (mapcar #'cdr (sort pairs #'(lambda (a b) (< (car a) (car b)))))))

(defun bowyer-watson (pts &optional (prime 7919))
  "pts: list of (list x y), eq-distinct. returns list of CCW triangles (each a list of 3 points from PTS, super-triangle corners excluded)."
  (setq pts (deterministic-shuffle pts prime))
  (let* ((minx (reduce #'min (mapcar #'car pts))) (maxx (reduce #'max (mapcar #'car pts)))
         (miny (reduce #'min (mapcar #'cadr pts))) (maxy (reduce #'max (mapcar #'cadr pts)))
         (d (max (- maxx minx) (- maxy miny) 1.0))
         (midx (/ (+ minx maxx) 2.0)) (midy (/ (+ miny maxy) 2.0))
         (dd (* 20 d))
         (p1 (list (- midx dd) (- midy dd)))
         (p2 (list midx (+ midy dd dd)))
         (p3 (list (+ midx dd) (- midy dd)))
         (triangles (list (ccw3 p1 p2 p3))))
    (dolist (p pts)
      (let (bad-triangles good-triangles)
        (dolist (tr triangles)
          (if (in-circumcircle (first tr) (second tr) (third tr) p)
              (push tr bad-triangles)
            (push tr good-triangles)))
        (let (all-edges boundary)
          (dolist (tr bad-triangles)
            (push (list (first tr) (second tr)) all-edges)
            (push (list (second tr) (third tr)) all-edges)
            (push (list (third tr) (first tr)) all-edges))
          (dolist (e all-edges)
            (when (= 1 (count-if #'(lambda (e2) (edge-eq e e2)) all-edges))
              (push e boundary)))
          (setq triangles good-triangles)
          (dolist (e boundary)
            (push (ccw3 (first e) (second e) p) triangles)))))
    (remove-if #'(lambda (tr) (or (member p1 tr :test #'eq) (member p2 tr :test #'eq) (member p3 tr :test #'eq)))
               triangles)))

(defun point-in-polygon-2d (pt poly)
  ;; ray-casting / even-odd rule. avoids nth+mod index arithmetic
  ;; entirely (walks POLY paired with its own rotation-by-one instead)
  ;; after that form was observed to occasionally corrupt under EusLisp.
  (let ((qx (car pt)) (qy (cadr pt)) (inside nil))
    (when (< (length poly) 3) (return-from point-in-polygon-2d nil))
    (let ((poly-next (append (cdr poly) (list (car poly)))))
      (mapc #'(lambda (pa pb)
                (let ((xa (car pa)) (ya (cadr pa))
                      (xb (car pb)) (yb (cadr pb)))
                  (when (and (not (eq (> ya qy) (> yb qy)))
                             (< qx (+ xa (* (/ (- xb xa) (- yb ya)) (- qy ya)))))
                    (setq inside (not inside)))))
            poly poly-next))
    inside))

(defun shoelace-area-2d (poly)
  (let ((s 0.0) (poly-next (append (cdr poly) (list (car poly)))))
    (mapc #'(lambda (a b) (incf s (- (* (car a) (cadr b)) (* (car b) (cadr a)))))
          poly poly-next)
    (/ (abs s) 2.0)))

(defun dedupe-closed-polyline (verts)
  ;; face/hole :vertices repeats the first vertex as a closing last
  ;; vertex (confirmed on a plain square plate: 5 vertices for 4
  ;; corners). that exact-duplicate point breaks Bowyer-Watson's
  ;; circumcircle math (a zero-radius/degenerate case), so drop it.
  (if (and (cdr verts) (< (norm (v- (car verts) (car (last verts)))) 1e-6))
      (butlast verts)
    verts))

(defun face-to-triangle-delaunay (f)
  "Constrained-Delaunay-ish triangulation of face F (possibly with
holes): unconstrained Bowyer-Watson over all boundary+hole vertices,
then keep only triangles whose centroid falls inside the boundary and
outside every hole. Good enough for an area-distribution comparison;
not a rigorous CDT (a true CDT would force-insert boundary/hole edges
that Delaunay would otherwise flip away, which can matter for thin or
close-together holes)."
  (let* ((fn (send f :normal))
         (verts (dedupe-closed-polyline (send f :vertices)))
         (holes (send f :holes))
         (p0 (car verts))
         (tmp (if (> (abs (aref fn 0)) 0.9) (float-vector 0 1 0) (float-vector 1 0 0)))
         (u (normalize-vector (v* fn tmp)))
         (v (normalize-vector (v* fn u)))
         (proj #'(lambda (p) (list (v. u (v- p p0)) (v. v (v- p p0)))))
         (boundary-2d (mapcar proj verts))
         (holes-2d (mapcar #'(lambda (h) (mapcar proj (dedupe-closed-polyline (send h :vertices)))) holes))
         (all-3d (append verts (apply #'append (mapcar #'(lambda (h) (dedupe-closed-polyline (send h :vertices))) holes))))
         (all-2d (append boundary-2d (apply #'append holes-2d)))
         (map3d (make-hash-table :test #'eq)))
    (mapc #'(lambda (p3 p2) (setf (gethash p2 map3d) p3)) all-3d all-2d)
    (let ((expected-area (- (shoelace-area-2d boundary-2d)
                            (apply #'+ (mapcar #'shoelace-area-2d holes-2d))))
          (primes '(7919 104729 15485863 999983 32452867))
          kept)
      (dolist (prime primes)
        (let ((raw (bowyer-watson all-2d prime))
              (this-kept nil))
          (dolist (tr raw)
            (let* ((cx (/ (+ (car (first tr)) (car (second tr)) (car (third tr))) 3.0))
                   (cy (/ (+ (cadr (first tr)) (cadr (second tr)) (cadr (third tr))) 3.0))
                   (c (list cx cy)))
              (when (and (point-in-polygon-2d c boundary-2d)
                         (not (some #'(lambda (h2d) (point-in-polygon-2d c h2d)) holes-2d)))
                (push tr this-kept))))
          (setq kept this-kept)
          (when (< (abs (- (apply #'+ (mapcar #'tri-area-2d this-kept)) expected-area))
                   (* 1e-6 expected-area))
            (return))))
      (unless (< (abs (- (apply #'+ (mapcar #'tri-area-2d kept)) expected-area))
                 (* 1e-3 expected-area))
        (warn ";; face-to-triangle-delaunay: kept-triangle area still off by more than 0.1% of the true polygon area after trying ~A insertion orders -- this face's point layout is an unusually hard case for this plain double-precision Bowyer-Watson (see comment above deterministic-shuffle); result may be visibly wrong.~%"
              (length primes)))
      (mapcar #'(lambda (tr) (make-face-from-vertices
                              (list (cons (gethash (first tr) map3d) nil)
                                    (cons (gethash (second tr) map3d) nil)
                                    (cons (gethash (third tr) map3d) nil))))
              kept))))

(defun face-to-triangle (f)
  "convert face to set of triangles."
  (if (and (derivedp f face) (send f :holes))
      (when (send f :holes) (setq f (geo::face-to-triangle-make-simple f))))
  (let* ((start 0) 
	 (poly (face-to-tessel-triangle f start)))
    (while (null poly)
      (if (>= start (length (send f :vertices)))
	  (return-from face-to-triangle nil))
      (incf start)
      (setq poly (face-to-tessel-triangle f start)))
    (cons poly 
	  (face-to-triangle-aux (face-to-triangle-rest-polygon f start (send poly :edges))))))

;; return polygon if triangable ,unless return nil.
;; this function = sharpp + trianglable + insidep
;; input face should be simple. see
;;   Make Simple function in Triangulatio by Ear Clipping by David Eberly
;;   http://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
(defun face-to-tessel-triangle (f num &optional (*epsilon* 1e-10))
  "return polygon if triangable, return nil if it is not."
  #-(or :x86_64 :aarch64)
  (setq *epsilon* (* 10000.0 *epsilon*));; for 32bit system
  (let* ((vers (send f :vertices))
	 (len (length (cdr vers)))
	 (v0 (nth (mod (1- (+ num len)) len) vers)) ;; num - 1 th
	 (v1 (nth (mod num len) vers)) ;; v1: num th
	 (v2 (nth (mod (1+  num) len) vers)) );; v2: num + 1 th
    (unless (> (v. (v* (v- v1 v0) (v- v2 v1)) (send f :normal)) 0)
	    (return-from face-to-tessel-triangle nil))
    (let ((poly (make-face-from-vertices
                 (mapcar #'(lambda (v) (cons v (mapcan
                                                #'(lambda (e) (if (memq v (send e :vertices)) (list (instance edge :init :pvertex (send e :pvertex f) :nvertex (send e :nvertex f)))))
                                                (send f :edges)))) (list v0 v1 v2))))
	  (rvers (remove v0 (remove v1 (remove v2 vers)))) not-intersectp)
      (setq not-intersectp
            (and (every #'(lambda (x) (not (eq ':inside (send poly :insidep x)))) rvers) ;; all rvers are not :inside
                 (every #'(lambda (e) ;; all rvers are not on the edges of poly / allow the same pvert/nvert
                            (every #'(lambda (v)
                                       (let ((p (send e :colinear-point v *epsilon*)))
                                         (not (and p (eps-in-range 0.0 p 1.0 (- *epsilon*)))
                                              )))
                                   rvers)) (send poly :edges))
                 (if (> (length rvers) 3)
                     (every #'(lambda (e) ;; all edges in rest-polygon do not intersect with the edges of poly / allow the same edge (:colinear)
                                ;; if not intersect return t;
                                (let* ((e02 (make-line v0 v2))
                                       (r (send e :intersect-line e02)))
                                  (or (null r)
                                      (eq r :parallel)
                                      (eq r :outside)
                                      (and (consp r)
                                           (eq (car r) :colinear))
                                      (and (consp r)
                                           (eq (car r) :intersect)
                                           (not (eps-in-range 0.0 (elt r 1) 1.0 (- *epsilon*)))
                                           (not (eps-in-range 0.0 (elt r 2) 1.0 (- *epsilon*)))))))
                            (send (instance face :init :vertices (remove v1 (butlast vers))) :edges))
                   t)))
      (if not-intersectp
	  poly
	nil))))


(defun face-to-triangle-rest-polygon (f num edgs)
  (let* ((vers (butlast (send f :vertices)))
	 (len (length vers))
	 (rvers (remove (nth (mod num len) vers) vers)))
    (if (<= (length rvers) 2) (return-from face-to-triangle-rest-polygon nil))
    (make-face-from-vertices
     (mapcar #'(lambda (v)
                 (cons v (mapcan #'(lambda (e) (if (memq v (send e :vertices)) (list e)))
                                 (append (mapcar #'(lambda (e) (instance edge :init :pvertex (send e :pvertex f) :nvertex (send e :nvertex f))) (send f :edges)) edgs)))) rvers))
    ))

;;;
;;; Make Simple function in Triangulatio by Ear Clipping by David Eberly
;;; http://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
;;;
(defun geo::face-to-triangle-make-simple (f)
  (let (hs d vl mind minv minh r)
    (while (setq hs (send f :holes))
      (setq mind nil)
      (dolist (h hs)
        ;; f: outer polyugon, h: inner polygon
        (setq vl (user::mapjoin #'cons (cdr (send f :vertices)) (cdr (send h :vertices))))
        ;; finding mutually visible vertices
        (dolist (v vl)
          ;; check if there are no intersection vertices
          (setq
           r
           (mapcan #'(lambda (e)
                       (let ((p (geo::line-intersection3 (e . pvert) (e . nvert)
                                                         (car v) (cdr v))))
                         (cond ;; intersects with an edge. not mutually visible.
                               ((and (<= 0.0 (car p) 1.0)
                                     (< 0.0 (cadr p) 1.0))
                                (list e))
                               ;; vertex is on an edge. never happen?
                               ((and (< 0.0 (car p) 1.0)
                                     (or (= 0.0 (cadr p)) (= 1.0 (cadr p))))
                                nil) ;; not implemented yet
                               ;; multiple vertices exist at the same position. avoid crossing. See https://github.com/euslisp/jskeus/pull/585#issuecomment-709988453
                               ((and (or (= 0.0 (car p)) (= 1.0 (car p)))
                                     (or (= 0.0 (cadr p)) (= 1.0 (cadr p)))
                                     (not (or (memq (car v) (send e :vertices)) (memq (cdr v) (send e :vertices)))))
                                ;; v0, v1, v2: consecutive vertices in f or h. v1 is at the same position as one of v, but not the same object.
                                ;; vt: another point of v
                                ;; If located clockwise around v1 in the order of v2 vt v0, crossing will occur.
                                (let* ((v1 (if (= 0.0 (car p)) (e . pvert) (e . nvert)))
                                       (ein (find-if #'(lambda (e) (eq v1 (e . nvert))) (flatten (append (send-all hs :edges) (send f :edges)))))
                                       (v0 (ein . pvert))
                                       (eout (find-if #'(lambda (e) (eq v1 (e . pvert))) (flatten (append (send-all hs :edges) (send f :edges)))))
                                       (v2 (eout . nvert))
                                       (vt (if (= 0.0 (cadr p)) (cdr v) (car v))))
                                  (when (< (mod (+ (vector-angle (v- v2 v1) (v- vt v1) (send f :normal)) 2PI) 2PI)
                                           (mod (+ (vector-angle (v- v2 v1) (v- v0 v1) (send f :normal)) 2PI) 2PI))
                                    (list e)))))
                         ))
                   (flatten (append (send-all hs :edges) (send f :edges)))))
          (unless r
            ;; r is mutually visible vertices
            (if mind
                (if (> mind (setq d (distance (car v) (cdr v))))
                    (setq mind d minv v minh h))
              (setq mind (distance (car v) (cdr v)) minv v minh h)))
          ))
      (if mind
          (let ((fp (position (car minv) (send f :vertices)))
                (hp (position (cdr minv) (send minh :vertices))))
            (setq f
                  (instance face
                            :init :vertices
                            (nconc
                             (subseq (send f :vertices) 0 fp)
                             (list (copy-object (car minv)))
                             (subseq (butlast (send minh :vertices)) hp)
                             (subseq (send minh :vertices) 0 hp)
                             (list (copy-object (cdr minv)))
                             (subseq (butlast (send f :vertices)) fp))
                            :holes (remove minh hs))))
        (progn
          (warn ";; geo::face-ear-clipping could not find `mutually visible vertices' for ~A~%" f)
          (return)))
      )
    f))

;;
;; body-to-triangles
;;
(defun body-to-faces (abody)
  "return triangled faces of given body"
  (body-to-triangles abody nil))
(defun body-to-triangles (abody &optional (limit 50))
  (let (fs)
    (setq *triangle-points* nil)
    (setq *triangle-edges* nil)
    (dolist (f (send abody :faces))
      (setq fs
            (nconc
             fs
             (mapcan #'(lambda (f)
                         (if limit
                             (geo::triangle-to-triangle f limit)
                           (list f)))
                     (geo::face-to-triangle f)))))
    (instance faceset :init :faces fs)))

(defun triangle-to-triangle (aface &optional (limit 50))
  (let (max-edge vs v vm vp vn v/2 f)
    (dolist (e (send aface :edges)) (push e *triangle-edges*))
    (labels
     ((make-vertex-edge-list
       (p) (cons p (mapcan #'(lambda (e) (if (memq p (send e :vertices)) (list e))) *triangle-edges*))))
     (setq max-edge (find-extream (send aface :edges)
                                  #'(lambda (e) (send e :length)) #'>=))
     (if (or (null limit) (< (send max-edge :length) limit))
         (progn
           (return-from triangle-to-triangle (list aface)))
       (progn
         (setq vs (set-exclusive-or (send max-edge :vertices)
                                    (cdr (send aface :vertices)) :test #'v=))
         (unless (= (length vs) 1) (warn ";; Error wrong vs ~A~%" vs))
         (setq v  (make-vertex-edge-list (car vs))
               vp (make-vertex-edge-list (send max-edge :pvertex aface))
               vn (make-vertex-edge-list (send max-edge :nvertex aface))
               v/2 (send max-edge :point 0.5))

         (if (member v/2 *triangle-points* :test #'equal)
             (setq v/2 (car (member v/2 *triangle-points* :test #'equal))))

         (push v/2 *triangle-points*)
         (setq vm (make-vertex-edge-list v/2))

         (nconc
          (triangle-to-triangle
           (make-face-from-vertices (list vp vm v))
           limit)
          (triangle-to-triangle
           (make-face-from-vertices (list vn v vm))
           limit))))
     )))

;;
;; make-* functions
;;
(defun make-sphere (r &rest args)
  "make sphere of given r"
  (let ((obj (make-gdome (make-icosahedron r))))
    (send obj :newcoords (apply #'make-coords args))
    (send obj :worldpos)
    (nconc (send obj :csg) `((:sphere ,r)))
    obj))

(defun make-ring (ring-radius pipe-radius &rest args &key (segments 16))
  "make ring of given ring and pipe radius"
  (let ((v-list) (dtheta (/ 2pi segments)))
    (dotimes (i segments)
      (push (float-vector (+ (* pipe-radius (cos (* i dtheta))) ring-radius)
                          0 (* pipe-radius (sin (* i dtheta))))
            v-list))
    (apply #'make-torus v-list :segments segments args)))

(defun make-fan-cylinder (radius height
                                 &rest args
                                 &key (segments 12)
                                      (angle 2pi)
                                      (mid-angle (/ angle 2.0)))
  "make a cylinder whose base face is a fan. the angle of fan
   is defined by :angle keyword. and, the csg of the returned body is
   (:cylinder radius height segments angle)"
  (let ((base-vertices
         (mapcar #'(lambda (i)
                     (let ((th (- (* i (/ angle segments)) mid-angle)))
                       (float-vector (* radius (cos th))
                                     (* radius (sin th))
                                     0)))
                 (let ((idx -1)) (mapcar #'(lambda (x) (incf idx)) (make-list (+ segments 1)))) ;; This is same as "range" function.
                 )))
    (make-prism
     (if (eps= angle 2pi)
         (cdr (reverse base-vertices))
       (cons (float-vector 0 0 0) (reverse base-vertices)))
     height
     :primitive (list :cylinder radius height segments angle))))

;;
;; accessor to primitive bodies
;;
(defun x-of-cube (cub)
  "return x of cube."
  (let ((csg (send cub :csg)))
    (car (cdr (assoc :cube csg)))))

(defun y-of-cube (cub)
  "return y of cube."
  (let ((csg (send cub :csg)))
    (cadr (cdr (assoc :cube csg)))))

(defun z-of-cube (cub)
  "return z of cube."
  (let ((csg (send cub :csg)))
    (caddr (cdr (assoc :cube csg)))))

(defun height-of-cylinder (cyl)
  "return height of cylinder."
  (let ((csg (send cyl :csg)))
    (caddr (assoc :cylinder csg))))

(defun radius-of-cylinder (cyl)
  "return radius of cylinder."
  (let ((csg (send cyl :csg)))
    (cadr (assoc :cylinder csg))))

(defun radius-of-sphere (sp)
  "return radius of shape."
  (let ((csg (send sp :csg)))
    (cadr (assoc :sphere csg))))

(defun make-faceset-from-vertices (vs)
  "create faceset from vertices."
  (let ((fs))
    (while vs
      (print (list (elt vs 0) (elt vs 1) (elt vs 2)))
      (push (make-face-from-vertices (list (pop vs)(pop vs) (pop vs))) fs)
      )
    (instance faceset :init :faces fs)
    ))

;;
;; matrix-to-euler-angle
;;
(defun matrix-to-euler-angle (m axis-order)
  "return euler angle from matrix."
  (let (a0 a1 a2 (eps 1.0e-6))
    (unless (and (matrixp m) (equal (array-dimensions m) '(3 3)))
      (error "3x3 matrix is required~A" m))
    (case (elt axis-order 0)
      (:x
       (case (elt axis-order 2)
	 (:x
	  (setq a1 (acos (aref m 0 0)))
	  (cond
	   ((eps= (aref m 0 0)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 2 1) (aref m 1 1))))
	   ((eps= (aref m 0 0) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 2 1) (- (aref m 1 1)))))
	   (t
	    (case (elt axis-order 1)
	      (:z ;; x-z-x
	       (setq a0 (user::atan2 (aref m 2 0) (aref m 1 0)))
	       (setq a2 (user::atan2 (- (aref m 0 2)) (aref m 0 1))))
	      (:y ;; x-y-x
	       (setq a0 (user::atan2 (- (aref m 1 0)) (aref m 2 0)))
	       (setq a2 (user::atan2 (aref m 0 1) (aref m 0 2))))
	      (t (error ""))))))
	 (:y ;; x-z-y
	  (unless (eq (elt axis-order 1) :z) (error ""))
	  (setq a1 (- (asin (aref m 0 1))))
	  (cond
	   ((eps= (aref m 0 1)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 2) (aref m 1 0))))
	   ((eps= (aref m 0 1) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 2) (aref m 2 2))))
	   (t
	    (setq a0 (user::atan2 (aref m 2 1) (aref m 1 1)))
	    (setq a2 (user::atan2 (aref m 0 2) (aref m 0 0))))))
	 (:z ;; x-y-z
	  (unless (eq (elt axis-order 1) :y) (error ""))
	  (setq a1 (asin (aref m 0 2)))
	  (cond
	   ((eps= (aref m 0 2)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 0) (aref m 1 1))))
	   ((eps= (aref m 0 2) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 0) (aref m 2 0))))
	   (t
	    (setq a0 (user::atan2 (- (aref m 1 2)) (aref m 2 2)))
	    (setq a2 (user::atan2 (- (aref m 0 1)) (aref m 0 0)))))
	  )))
      (:y
       (case (elt axis-order 2)
	 (:y
	  (setq a1 (acos (aref m 1 1)))
	  (cond
	   ((eps= (aref m 1 1)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 2) (aref m 0 0))))
	   ((eps= (aref m 1 1) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 2 0) (aref m 0 0))))
	   (t
	    (case (elt axis-order 1)
	      (:x ;; y-x-y
	       (setq a0 (user::atan2 (aref m 0 1) (aref m 2 1)))
	       (setq a2 (user::atan2 (- (aref m 1 0)) (aref m 1 2))))
	      (:z ;; y-z-y
	       (setq a0 (user::atan2 (- (aref m 2 1)) (aref m 0 1)))
	       (setq a2 (user::atan2 (aref m 1 2) (aref m 1 0))))
	      (t (error ""))))))
	 (:z ;; y-x-z
	  (unless (eq (elt axis-order 1) :x) (error ""))
	  (setq a1 (- (asin (aref m 1 2))))
	  (cond
	   ((eps= (aref m 1 2)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 1) (aref m 2 1))))
	   ((eps= (aref m 1 2) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 2 0) (aref m 0 0))))
	   (t
	    (setq a0 (user::atan2 (aref m 0 2) (aref m 2 2)))
	    (setq a2 (user::atan2 (aref m 1 0) (aref m 1 1))))))
	 (:x ;; y-z-x
	  (unless (eq (elt axis-order 1) :z) (error ""))
	  (setq a1 (asin (aref m 1 0)))
	  (cond
	   ((eps= (aref m 1 0)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 2) (aref m 2 2))))
	   ((eps= (aref m 1 0) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 2 1) (aref m 2 2))))
	   (t
	    (setq a0 (user::atan2 (- (aref m 2 0)) (aref m 0 0)))
	    (setq a2 (user::atan2 (- (aref m 1 2)) (aref m 1 1)))))
	  )))
      (:z
       (case (elt axis-order 2)
	 (:z
	  (setq a1 (acos (aref m 2 2)))
	  (cond
	   ((eps= (aref m 2 2)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 0) (aref m 0 0))))
	   ((eps= (aref m 2 2) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 0) (aref m 1 1))))
	   (t
	    (case (elt axis-order 1)
	      (:y ;; z-y-z
	       (setq a0 (user::atan2 (aref m 1 2) (aref m 0 2)))
	       (setq a2 (user::atan2 (- (aref m 2 1)) (aref m 2 0))))
	      (:x ;; z-x-z
	       (setq a0 (user::atan2 (- (aref m 0 2)) (aref m 1 2)))
	       (setq a2 (user::atan2 (aref m 2 0) (aref m 2 1))))
	      (t (error ""))))))
	 (:x ;; z-y-x
	  (unless (eq (elt axis-order 1) :y) (error ""))
	  (setq a1 (- (asin (aref m 2 0))))
	  (cond
	   ((eps= (aref m 2 0)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 1 2) (aref m 0 2))))
	   ((eps= (aref m 2 0) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 1) (aref m 0 2))))
	   (t
	    (setq a0 (user::atan2 (aref m 1 0) (aref m 0 0)))
	    (setq a2 (user::atan2 (aref m 2 1) (aref m 2 2))))))
	 (:y ;; z-x-y
	  (unless (eq (elt axis-order 1) :x) (error ""))
	  (setq a1 (asin (aref m 2 1)))
	  (cond
	   ((eps= (aref m 2 1)  1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 2) (aref m 0 0))))
	   ((eps= (aref m 2 1) -1.0 eps)
	    (setq a0 0 a2 (user::atan2 (aref m 0 2) (aref m 0 0))))
	   (t
	    (setq a0 (user::atan2 (- (aref m 0 1)) (aref m 1 1)))
	    (setq a2 (user::atan2 (- (aref m 2 0)) (aref m 2 2)))))
	  ))))
    (list a0 a1 a2)))

(defun quaternion-from-two-vectors (a b)
  "Comupute quaternion which rotate vector a into b."
  (let* ((v0 (normalize-vector a))
         (v1 (normalize-vector b))
         (c (v. v1 v0)))
    (let* ((axis (v* v0 v1))
           (s (sqrt (* 2 (+ 1 c))))
           (invs (/ 1.0 s)))
      (let ((vec (scale invs axis))
            (w (* 0.5 s)))
        (normalize-vector
	 (float-vector w (elt vec 0) (elt vec 1) (elt vec 2)))))))

(provide :irtgeo "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.16  2010-01-13 01:43:47  k-okada
;;; fix typo, check with cvs -r 1.14 1.14 irtgeo.l
;;;
;;; Revision 1.15  2010/01/10 10:57:05  ueda
;;; fix to write mapjoin's package
;;;
;;; Revision 1.14  2009/12/26 15:24:24  eus
;;; bug fix face-to-triangle 'bad face' error if triangle assigned
;;;
;;; Revision 1.13  2009/12/09 06:03:32  k-okada
;;; update face-to-triangle, body-to-triagles
;;;
;;; Revision 1.12  2009/11/27 11:08:59  k-okada
;;; add face-ear-clpping, face-to-triangle supports face with hole
;;;
;;; Revision 1.11  2009/10/27 02:45:31  nozawa
;;; add \*g-vec\* for dynamics calculation (ref. euslib/jsk/preview.l and euslib/rbrain/dynamics.l)
;;;
;;; Revision 1.10  2009/09/24 10:07:34  k-okada
;;; fix differnece-position/rotation to calculate local difference
;;;
;;; Revision 1.9  2009/09/10 16:02:39  nozawa
;;; fix calculation of dif-pos (in :difference-position)
;;;
;;; Revision 1.8  2009/09/10 15:24:04  k-okada
;;; update :difference-translation
;;;
;;; Revision 1.7  2009/08/07 12:13:20  k-okada
;;; support rotation-axis :xx :yy :zz
;;;
;;; Revision 1.6  2009/05/15 07:45:48  eus
;;; remove :box from bodyset
;;;
;;; Revision 1.5  2009/05/15 01:02:27  nozawa
;;; add :box to bodyset
;;;
;;; Revision 1.4  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.3  2009/01/05 03:48:31  k-okada
;;; move defmethod polygon :reset-normal to jskgeo.l
;;;
;;; Revision 1.2  2008/11/11 14:01:54  k-okada
;;; error handling when normalize-vector #f(0 0 0), again
;;;
;;; Revision 1.1  2008/09/18 18:11:00  k-okada
;;; add irteus
;;;
;;;
