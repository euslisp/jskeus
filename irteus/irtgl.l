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

(require :irtglrgb)

(in-package "GL")

(let (gl-lib)
#+:darwin
 (progn
   (cond
    ((probe-file "/opt/local/lib/libGL.dylib")
     (setq gl-lib ( load-foreign "/opt/local/lib/libGL.dylib")))
    (t
     (setq gl-lib ( load-foreign "/opt/X11/lib/libGL.dylib")))))
#+:cygwin
  (progn
    (cond
     ((probe-file "/usr/bin/cygGL-1.dll")
      (setq gl-lib ( load-foreign "/usr/bin/cygGL-1.dll")))
     (t
      (setq gl-lib ( load-foreign "/usr/X11R6/bin/cygGL-1.dll")))))
#+(and :linux (not :darwin))
  (setq gl-lib (sys::sysmod))

  (defforeign glPolygonOffset gl-lib "glPolygonOffset" () :integer)
  (defconstant GL_POLYGON_OFFSET_UNITS		#x2a00)
  (defconstant GL_POLYGON_OFFSET_POINT		#x2a01)
  (defconstant GL_POLYGON_OFFSET_LINE		#x2a02)
  (defconstant GL_POLYGON_OFFSET_FILL		#x8037)
  (defconstant GL_POLYGON_OFFSET_FACTOR		#x8038)
  (defconstant GL_POLYGON_OFFSET_EXT		#x8037)
  (defconstant GL_POLYGON_OFFSET_FACTOR_EXT	#x8038)
  (defconstant GL_POLYGON_OFFSET_BIAS_EXT	#x8039)

  ;; for using array in OpenGL
  (defforeign glEnableClientState gl-lib "glEnableClientState" () :integer)
  (defforeign glDisableClientState gl-lib "glDisableClientState" () :integer)
  (defforeign glVertexPointer gl-lib "glVertexPointer" () :integer)
  (defforeign glColorPointer gl-lib "glColorPointer" () :integer)
  (defforeign glNormalPointer gl-lib "glNormalPointer" () :integer)
  (defforeign glTexCoordPointer gl-lib "glTexCoordPointer" () :integer)
  (defforeign glDrawElements gl-lib "glDrawElements" () :integer)
  (defforeign glArrayElement gl-lib "glArrayElement" () :integer)
  (defforeign glDrawArrays gl-lib "glDrawArrays" () :integer)
  (defconstant GL_VERTEX_ARRAY        #x8074)
  (defconstant GL_NORMAL_ARRAY        #x8075)
  (defconstant GL_TEXTURE_COORD_ARRAY #x8078)
  (defconstant GL_EDGE_FLAG_ARRAY     #x8079)
  (defconstant GL_COLOR_ARRAY         #x8076)
  (defconstant GL_INDEX_ARRAY         #x8077)
  )

(defun set-stereo-gl-attribute ()
  (reset-gl-attribute)
  (let ((iv (make-array (1+ (length gl::*attributelist*)) :element-type :integer)))
    (sys::vector-replace iv *attributelist*)
    (setf (elt iv (1- (length *attributelist*))) glx_stereo)
    (setq *attributelist* iv)))
(defun reset-gl-attribute ()
  (setq *attributelist*
        (integer-vector glx_rgba glx_red_size 1 glx_green_size 1
                        glx_blue_size 1 glx_doublebuffer glx_depth_size 1 0)))

(defun delete-displaylist-id (dllst)
  (cond
   ((numberp dllst)
    (gldeletelists dllst 1))
   (t
    (dolist (alist dllst)
      (let ((v (find (car alist) user::*viewers* :key #'(lambda (x) ((send x :viewsurface) . glcon)))))
        (if v
            (progn
              (send v :viewsurface :makecurrent)
              (cond ((glIsList (cdr alist))
                     (glDeleteLists (cdr alist) 1))
                    (t (error "~A is not display list" alist))))
          (error "could not find viewers for ~A" alist))))) ;; t
   ))

(unless (assoc :color-org (send glviewsurface :methods))
  (rplaca (assoc :color (send glviewsurface :methods)) :color-org))
(defmethod glviewsurface
  (:color 
   (&optional color-vector)
   (if color-vector (send self :color-org color-vector)
     (let ((v (float-vector 0 0 0 0)))
       (glgetfloatv GL_CURRENT_COLOR v)
       (subseq v 0 3))))
  (:line-width 
   (&optional x)
   (if x (glLineWidth (float x))
     (let ((tmp (float-vector 0)))
       (glGetFloatv GL_LINE_WIDTH tmp)
       (elt tmp 0))
     ))
  (:point-size 
   (&optional x)
   (if x (glPointSize (float x))
     (let ((tmp (float-vector 0)))
       (glGetFloatv GL_POINT_SIZE tmp)
       (elt tmp 0))
     ))
  (:3d-point (pos &key (depth-test t) (lighting t)) ;; redefined
	     (if depth-test (glDisable GL_DEPTH_TEST))
	     (if lighting (glDisable GL_LIGHTING))
	     (glBegin GL_POINTS)
	     (glVertex3fv pos)
	     (glEnd)
	     (if depth-test (glEnable GL_DEPTH_TEST))
	     (if lighting (glEnable GL_LIGHTING)))
  (:3d-line (start end &key (depth-test t) (lighting t)) ;; redefined
	    (if depth-test (glDisable GL_DEPTH_TEST))
	    (if lighting (glDisable GL_LIGHTING))
	    (glBegin GL_LINE_STRIP)
	    (glVertex3fv start)
	    (glVertex3fv end)
	    (glEnd)
	    (if depth-test (glEnable GL_DEPTH_TEST))
	    (if lighting (glEnable GL_LIGHTING)))
  (:3d-lines (points &key (depth-test t) (lighting t))
	     (if depth-test (glDisable GL_DEPTH_TEST))
	     (if lighting (glDisable GL_LIGHTING))
	     (glBegin GL_LINE_STRIP)
	     (dolist (p points) (glVertex3fv p))
	     (glEnd)
	     (if depth-test (glEnable GL_DEPTH_TEST))
	     (if lighting (glEnable GL_LIGHTING)))
  ;;
  (:makecurrent () (gl::glxMakeCurrent x::*display* x::drawable glcon))
  (:redraw (&rest args) (send self :flush))
  (:flush 
   ()
   (send self :makecurrent)
   (send self :glflush)
   (send-super :flush)
   )

  ;;
  (:write-to-image-file
   (file &key (x 0) (y 0)
	 (width x::width)
	 (height x::height))
   (let* ((glimg (send self :getglimage
		       :x x :y y :width (1- width) :height height)))
     (image::write-image-file file glimg)
     ))
  (:getglimage
   (&key (x 0) (y 0)
	 (width x::width)
	 (height x::height)
	 ((:imagebuf imgbuf) (make-string (* width height 3)))
         (depthbuf))
   (let ()
     (send self :makecurrent)
     (glReadBuffer GL_BACK)
     (glPixelStorei GL_PACK_ALIGNMENT 1)
     (glReadPixels x y width height GL_RGB GL_UNSIGNED_BYTE imgbuf)
     #-:x86_64
     (if depthbuf (glReadPixels x y width height GL_DEPTH_COMPONENT GL_FLOAT depthbuf))
     #+:x86_64
     (when depthbuf
       (let ((fv (user::dvector2float-bytestring depthbuf)))
	 (glReadPixels x y width height GL_DEPTH_COMPONENT GL_FLOAT fv)
	 (user::float-bytestring2dvector fv depthbuf)))
     ;; transpose
     (let ((b (make-string (* width height 3))))
       (dotimes (x width)
	 (dotimes (y height)
	   (dotimes (z 3)
	     (setf (elt b (+ (* (- height y 1) width 3) (* x 3) z))
		   (elt imgbuf (+ (* y width 3) (* x 3) z))))))
       (instance image::color-image24 :init width height b))
     ))
  )

(defun draw-globjects (vwr draw-things &key (clear t) (flush t))
  (let (pcolor)
    (resetperspective (send vwr :viewing) (send vwr :viewsurface))
    (if clear (send vwr :viewsurface :clear))
    ;;(apply #'geo::draw things)
    (setq pcolor (send vwr :viewsurface :color))
    ;; draw origin
    (glDisable GL_LIGHTING)
    (glBegin GL_LINES)
    (glColor3fv #f(1 0 0)) (glVertex3fv #f(0 0 0)) (glVertex3fv #f(100 0 0))
    (glColor3fv #f(0 1 0)) (glVertex3fv #f(0 0 0)) (glVertex3fv #f(0 100 0))
    (glColor3fv #f(0 0 1)) (glVertex3fv #f(0 0 0)) (glVertex3fv #f(0 0 100))
    (glEnd GL_LINES)
    (glEnable GL_LIGHTING)
    (glDisable GL_BLEND)
    (send vwr :viewsurface :color pcolor)
      
    (dolist (abody draw-things)
      ;; draw body
      (cond
       ((find-method abody :draw)
	(send abody :draw vwr))
       ((derivedp abody faceset)
	(draw-glbody vwr abody))
       ((find-method abody :draw-on)
	(send abody :draw-on :viewer vwr))
       (t (warn "Unknown body to draw ~A~%" abody)))
       )
    (if flush (send vwr :viewsurface :flush))
    ))

;;
;; re-definition
;;  
(defun draw-glbody (vwr abody)
  (let* ((glcon ((send vwr :viewsurface) . glcon))
	 (lis (cdr (assq glcon (get abody :GL-DISPLAYLIST-ID))))
	 (hid (cdr (assq glcon (get abody :gl-hiddenline))))
	 (col (get abody :face-color)))
    (unless col (setq col (float-vector 0.5 0.5 0.5)))
    (unless (vectorp col)
      ;;(warn "draw-body: body ~A face-color ~A~%" abody col)
      (setq col (find-color col))
      (setf (get abody :face-color) col))
    (cond
     (lis
      (let ((mat (send (send abody :worldcoords) :4x4)))
	(glPushMatrix)
	(glMultMatrixf (array-entity (transpose mat *temp-matrix*)))
	(glCallList lis)
	(glPopMatrix)))
     (hid
      (let ((newlis (glGenLists 1)))
	(glNewList newlis gl_compile)
        (glPushAttrib GL_ALL_ATTRIB_BITS)

        (glDepthFunc GL_DEPTH_TEST)
        (glDisable GL_LIGHTING)
        (glColor3fv #f(1 1 1))
	(dolist (aface (send abody :faces))
          (glBegin GL_LINE_STRIP)
          (dolist (p (send aface :vertices))
            (glVertex3fv (send abody :inverse-transform-vector p)))
          (glEnd))
        ;;
        (glEnable GL_POLYGON_OFFSET_FILL)
        (glPolygonMode GL_FRONT_AND_BACK GL_FILL)
        (glPolygonOffset 1.0 1.0)

        (glColor3fv #f(0 0 0))
        (glEnable GL_CULL_FACE)
        (glCullFace GL_FRONT)

        (mapc #'(lambda (aface)
                  (draw-face aface nil nil))
              (send abody :faces))

        (glDisable GL_CULL_FACE)
        (glDisable GL_POLYGON_OFFSET_FILL)
        (glEnable GL_LIGHTING)
        (glPopAttrib)

	(glEndList)
	(setf (get abody :GL-DISPLAYLIST-ID)
	      (cons (cons glcon newlis) (get abody :GL-DISPLAYLIST-ID)))
	(draw-glbody vwr abody)
        ))
     (t
      (let ((newlis (glGenLists 1))
	    (transp (and (= (length col) 4) (< (elt col 3) 1.0)))
            (textures
             (if (get abody :GL-TEXTUREIMAGE)
                 (instantiate integer-vector (length (get abody :GL-TEXTUREIMAGE)))))

	    p2)
	(glNewList newlis gl_compile)
	(when transp
	  (glEnable GL_BLEND)
	  (glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA))
	(glMaterialfv GL_FRONT_AND_BACK GL_AMBIENT_AND_DIFFUSE col)
        (if (> (length textures) 0) (glGenTexturesEXT (length textures) textures))
	(dolist (aface (send abody :faces))
	  (cond
	   ;; normal face
	   ((and (send aface :convexp) (not (send aface :holes)))
	    (let* ((texture-img (get aface :GL-TEXTUREIMAGE))
                   (texture-coords (get aface :GL-TEXTURECOORDS))
                   (pixelformat
                    (cond ((derivedp texture-img img::color-image24) GL_RGB)
                          ((derivedp texture-img img::grayscale-image) GL_LUMINANCE))))
	      (when texture-img
                (unless (get texture-img :GL-BIND-TEXTURE)
                  (let* ((texture-id
                          (elt textures (position texture-img (get abody :GL-TEXTUREIMAGE))))
                         (ow (send texture-img :width))
                         (oh (send texture-img :height))
                         (mw (or (get texture-img :texture-max-width) 256))
                         (mh (or (get texture-img :texture-max-height) 256))
                         (tw (ash 1 (ceiling (log ow 2))))
                         (th (ash 1 (ceiling (log oh 2))))
                         (img texture-img)) ;; img might be scaled image
                    (when (not (and (= ow tw) (= oh th)))
                      ;; rescale to boundary
                      (let (b od (name (send img :name)) scale)
                        (cond
                         ((= pixelformat GL_RGB)
                          (setq od (/ (send img :depth) 8))
                          (when (not (= od 3)) (error "not supported depth")))
                         ((= pixelformat GL_LUMINANCE)
                          (setq od 1))
                         (t
                          (error "unsupported image type ~A in ~A" pixelformat img)))
                        (if (> (setq scale (round (sqrt (/ (* tw th) (* mw mh))))) 1)
                            (setq tw (ash 1 (ceiling (log (/ tw scale) 2)))
                                  th (ash 1 (ceiling (log (/ th scale) 2)))))
                        (setq b (make-string (* tw th od)))
                        (gluScaleImage pixelformat ow oh GL_UNSIGNED_BYTE (send img :entity)
                                       tw th GL_UNSIGNED_BYTE b)
                        (setq img (instance (class img) :init tw th b))
                        (send img :name name)))
                    (glBindTextureEXT GL_TEXTURE_2D texture-id)
                    (glTexImage2D
                     GL_TEXTURE_2D
                     0 GL_RGB
                     (send img :width) (send img :height)
                     0 pixelformat GL_UNSIGNED_BYTE (send img :entity))
                    (setf (get texture-img :GL-BIND-TEXTURE) texture-id)
                    ))
                ;; texturemap stuff...  glviewsurface :create in glview.l
                (glPixelStorei GL_UNPACK_ALIGNMENT 1)
                (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_WRAP_S GL_REPEAT)
                (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_WRAP_T GL_REPEAT)
                (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MIN_FILTER GL_NEAREST)
                (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MAG_FILTER GL_NEAREST)
                (glTexEnvi GL_TEXTURE_ENV GL_TEXTURE_ENV_MODE GL_DECAL)
                (glEnable GL_TEXTURE_2D)
                (glBindTextureEXT GL_TEXTURE_2D (get texture-img :GL-BIND-TEXTURE))
                ) ;; when texture-img
	      (glBegin GL_POLYGON)
	      (glNormal3fv (transform (transpose (send abody :worldrot)) (send aface :normal)))
	      (dolist (e (send aface :edges))
		(if (and texture-coords
			 (setq p2 (gethash e texture-coords)))
		    (glTexCoord2fv p2))
		(glVertex3fv (send abody :inverse-transform-vector (send e :pvertex aface))))
	      (glEnd)
	      (when texture-img
		(glDisable GL_TEXTURE_2D)
		(glBindTextureEXT GL_TEXTURE_2D 0))
	      ))
	   ;; holed face
	   (t
	    (let ((holes (send aface :holes)) l-tessinfo)
	      (setq l-tessinfo
		(mapcar #'(lambda (p)
			    (setq p (send abody :inverse-transform-vector p))
			    (alloctessinfo p 0 nil 0 nil))
			(cdr (send aface :vertices))))
	      (gluTessBeginPolygon *tess-obj* 0)
	      (gluTessBeginContour *tess-obj*)
	      (glNormal3fv (send aface :normal))
	      (mapc #'(lambda (i) (gluTessVertex *tess-obj* i i)) l-tessinfo)
              (gluTessEndContour *tess-obj*)
	      ;; holes
              (gluTessBeginContour *tess-obj*)
	      (when holes
		(dolist (hole holes)
		  (let ((h-l-tessinfo
			 (mapcar #'(lambda (p)
				     (setq p (send abody :inverse-transform-vector p))
				     (alloctessinfo p 0 nil 0 nil))
				 (send hole :vertices))))
		    (gluNextContour *tess-obj* GLU_INTERIOR)
		    (mapc #'(lambda (i) (gluTessVertex *tess-obj* i i)) h-l-tessinfo)
		    (nconc l-tessinfo h-l-tessinfo) ;hook it, to deallocated later.
		    )))
	      (gluTessEndContour *tess-obj*)
	      (gluTessEndPolygon *tess-obj*)
	      (mapc #'unix:free l-tessinfo) ;deallocate
	      )))
	  )
	;;
	(when transp
	  (glDisable GL_BLEND))
	(glEndList)
	(setf (get abody :GL-DISPLAYLIST-ID)
	      (cons (cons glcon newlis) (get abody :GL-DISPLAYLIST-ID)))
	(draw-glbody vwr abody)
	)))
    ))

(defun find-color (color)
  (let (v c)
    (setq v
      (cond
       ((null color) nil)
       ((derivedp color colormaterial) (send color :diffuse))
       ((symbolp color)
	(setq c (find-if #'(lambda (o) (eq (send o :name) color))
			 *face-colors*))
	(when c (send c :diffuse)))
       (t color)))
    (unless v (setq v (float-vector 0.5 0.5 0.5)))
    v))

(defun transparent (abody param)
  (let (fc dif)
    (when (setq fc (get abody :face-color))
      (unless (vectorp fc)
	(warn "transparent: body ~A param ~A face-color ~A~%" abody param fc)
	(setq fc (find-color fc)))
      (if (= (length fc) 3)
	  (setq fc
	    (concatenate float-vector fc #f(0))))
      (setf (elt fc 3) param)
      (setf (get abody :face-color) fc)
      (delete-displaylist-id (get abody :GL-DISPLAYLIST-ID))
      (setf (get abody :GL-DISPLAYLIST-ID) nil)
      fc)))

(defmethod polygon
  (:draw-on
   (&key ((:viewer vwer) *viewer*)
         flush (width 1) (color #f(1 1 1)))
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color))
	 (v (float-vector 0 0 0)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (send vwer :viewsurface :3d-lines (send self :vertices))
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  )

(defmethod line
  (:draw-on
   (&key ((:viewer vwer) *viewer*)
         flush (width 1) (color #f(1 1 1)))
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color))
	 (v (float-vector 0 0 0)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (send vwer :viewsurface :3d-line pvert nvert)
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  )

(defmethod faceset
  (:set-color
   (color &optional (transparent))
   (delete-displaylist-id (get self :GL-DISPLAYLIST-ID))
   (setf (get self :GL-DISPLAYLIST-ID) nil)
   (cond
    (transparent
     (let ((col (gl::find-color color)))
       (setq col (concatenate float-vector col (float-vector transparent)))
       (setf (get self :face-color) col)))
    (t
     (setf (get self :face-color) (gl::find-color color)))
    ))
  (:draw-on
   (&key ((:viewer vwer) *viewer*)
         flush (width 1) (color #f(1 1 1)))
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (dolist (f (send self :faces))
       (send vwer :viewsurface :3d-lines (send f :vertices)))
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  ;;
  (:paste-texture-to-face
   (aface &key file image
          (tex-coords
           (list
            (float-vector 0 0)
            (float-vector 0 1)
            (float-vector 1 1)
            (float-vector 1 0)))
          )
   (let (img ow oh od tw th)
     (cond
      (image
       (setq img image))
      ((member file (send-all (get self :gl-textureimage) :name) :test #'equal)
       (setq img (car (member file (get self :gl-textureimage)
                              :test #'equal
                              :key #'(lambda (x) (send x :name))))))
      ((probe-file file)
       (setq img (user::read-image-file file)))
      ((probe-file (format nil "~A/img/~A" *eusdir* file))
       (setq img (user::read-image-file (format nil "~A/img/~A" *eusdir* file))))
      ((probe-file (format nil "~A/~A" *eusdir* file))
       (setq img (user::read-image-file (format nil "~A/~A" *eusdir* file))))

      (t (warn ";; Could not find file ~A~%" file)
	 (return-from :paste-texture-to-face nil)))
     (setf (get aface :gl-textureimage) img)
     (if (not (memq img (get self :gl-textureimage)))
         (setf (get self :gl-textureimage)
               (append (get self :gl-textureimage) (list img))))
     (setf (get aface :gl-texturecoords) (make-hash-table :test #'equal))
     (delete-displaylist-id (get self :GL-DISPLAYLIST-ID))
     (setf (get self :GL-DISPLAYLIST-ID) nil)
     ;;
     (dolist (e (send aface :edges))
       (setf (gethash e (get aface :gl-texturecoords))
             (pop tex-coords)))
     ))
  )

(defmethod coordinates
  (:vertices () (list (send self :worldpos)))
  (:draw-on
   (&key ((:viewer vwer) user::*viewer*)
	 flush (width (get self :width)) (color (get self :color))
	 (size (get self :size)))
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color))
	 (v (float-vector 0 0 0)) v2)
     (if (null width) (setq width 1))
     (if (null color) (setq color #f(1 1 1)))
     (if (null size) (setq size 50))
     (setq v2 (float-vector (* 0.3 size) 0 (* 0.7 size)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (dotimes (i 3)
       (setf (elt v i) size)
       (send vwer :viewsurface :3d-line
	     (send self :worldpos)
	     (send self :transform-vector v))
       (setf (elt v i) 0))
     (setf (elt v 2) size)
     (send vwer :viewsurface :3d-line
	   (send self :transform-vector v)
	   (send self :transform-vector v2))
     (setf (elt v2 1) (elt v2 0)  (elt v2 0) 0)
     (send vwer :viewsurface :3d-line
	   (send self :transform-vector v)
	   (send self :transform-vector v2))
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  )

(defmethod geo::float-vector
  (:vertices () (list self))
  (:draw-on
   (&key ((:viewer vwer) *viewer*)
         flush (width 1) (color #f(1 1 1)) (size 50))
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color))
	 (v (float-vector 0 0 0)))
     (send vwer :viewsurface :line-width width)
     (send vwer :viewsurface :color color)
     (dotimes (i 3)
       (setf (elt v i) size)
       (send vwer :viewsurface :3d-line self (v+ self v))
       (setf (elt v i) 0))
     (send vwer :viewsurface :line-width pwidth)
     (send vwer :viewsurface :color pcolor)
     (if flush (send vwer :viewsurface :flush))
     ))
  )

(defclass glvertices
  :super cascaded-coords
  :slots (mesh-list  ;; (list (list (:type ) (:material ) (:vertices ) (:normals ) (:indices )) (...) ...)
          filename
          bbox)
  )

(defmethod glvertices
  (:init
   (mlst &rest args &key ((:filename fn)) &allow-other-keys)
   (setq mesh-list mlst)
   (setq filename fn)
   (send-super* :init args)
   self)
  (:filename (&optional nm) (if nm (setq filename nm)) filename)
  (:set-color (color &optional (transparent))
   (delete-displaylist-id (get self :GL-DISPLAYLIST-ID))
   (setf (get self :GL-DISPLAYLIST-ID) nil)
   (setf (get self :transparent) transparent)
   (if color
       (setf (get self :face-color) (gl::find-color color))
     (setf (get self :face-color) nil)))
  ;; (:check-normal ())
  ;; (:update ()) ;; call update when updating coordinates
  (:actual-vertices ()
   (let (ret)
     (dolist (minfo mesh-list)
       (let ((v (cadr (assoc :vertices minfo))))
         (when v
           (dotimes (i (array-dimension v 0))
             (push (user::c-matrix-row v i) ret)))))
     ret))
  (:calc-bounding-box ()
   (setq bbox (make-bounding-box (send self :actual-vertices) 0.0))
   bbox)
  (:vertices ()
   (unless bbox
     (send self :calc-bounding-box))
   (send (send bbox :body) :vertices))
  (:reset-offset-from-parent ()
   (let ((cds (send self :copy-coords)))
     (send self :set-offset cds)
     (send self :reset-coords))
   self)
  (:expand-vertices ()
   (let (new-mesh)
     (dolist (minfo mesh-list)
       (let (;;(nmat (cadr (assoc :normals minfo)))
             ;;(tp (cadr (assoc :type minfo)))
             (idxs (cadr (assoc :indices minfo)))
             (vmat (cadr (assoc :vertices minfo)))
             (p (instantiate float-vector 3)))
         (cond
          ((> (length idxs) (array-dimension vmat 0)) ;; TODO: redundancy check in indices
           (let ((nvmat (make-matrix (length idxs) 3))
                 (nidxs (instantiate integer-vector (length idxs))))
             (setq minfo (delete (assoc :vertices minfo) minfo))
             (setq minfo (delete (assoc :indices minfo) minfo))
             (dotimes (i (length idxs))
               (setf (elt nidxs i) i)
               (user::c-matrix-row vmat (elt idxs i) p)
               (user::c-matrix-row nvmat i p t))
             (setq minfo (nconc minfo (list (list :vertices nvmat) (list :indices nidxs))))
             (push minfo new-mesh)
             ))
          (t
           (push minfo new-mesh)))
         ))
     (setq mesh-list new-mesh)
     ))
  (:calc-normals (&optional (force nil) (expand t))
   (if expand (send self :expand-vertices))
   (dolist (minfo mesh-list)
     (let ((nmat (cadr (assoc :normals minfo)))
           (tp (cadr (assoc :type minfo)))
           (idxs (cadr (assoc :indices minfo)))
           (vmat (cadr (assoc :vertices minfo)))
           (a0 (instantiate float-vector 3))
           (a1 (instantiate float-vector 3))
           (a2 (instantiate float-vector 3))
           (va (instantiate float-vector 3))
           (vb (instantiate float-vector 3))
           (nn (instantiate float-vector 3)))
       (case tp
         (:triangles (setq tp 3))
         (:quads     (setq tp 4))
         (:lines     (setq tp 2))
         (nil (setq tp 3) (warn ";; keyword :type not found (processing as :triangles)~%"))
         (t (warn ";; mesh-type not found ~A~%") (return-from :calc-normals)))
       (unless (and nmat (null force))
         (if nmat (delete (assoc :normals minfo) minfo))
         (let ((len (array-dimension vmat 0)))
           (setq nmat (make-matrix len 3))
           (cond
            (idxs
             (dotimes (i (/ (length idxs) tp))
               (user::c-matrix-row vmat (elt idxs (+ (* tp i) 0)) a0)
               (user::c-matrix-row vmat (elt idxs (+ (* tp i) 1)) a1)
               (user::c-matrix-row vmat (elt idxs (+ (* tp i) 2)) a2)
               (v* (v- a1 a0 va) (v- a2 a0 vb) nn)
               (normalize-vector nn nn)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 0)) nn t)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 1)) nn t)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 2)) nn t)))
            (t
             (dotimes (i (/ len tp))
               (user::c-matrix-row vmat (+ (* tp i) 0) a0)
               (user::c-matrix-row vmat (+ (* tp i) 1) a1)
               (user::c-matrix-row vmat (+ (* tp i) 2) a2)
               (v* (v- a1 a0 va) (v- a2 a0 vb) nn)
               (normalize-vector nn nn)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 0)) nn t)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 1)) nn t)
               (user::c-matrix-row nmat (elt idxs (+ (* tp i) 2)) nn t))))
           (nconc minfo (list (list :normals nmat)))
           ))
       )))
  (:convert-to-faces (&rest args &key (wrt :local) &allow-other-keys) ;; check-normal <-> check-vertices-order, add face-color
   (let (ret)
     (dolist (mesh mesh-list)
       (let ((vlst (cadr (assoc :vertices mesh)))
             (nlst (cadr (assoc :normals mesh)))
             (idces (cadr (assoc :indices mesh)))
             (tp (cadr (assoc :type mesh)))
             mesh-size lst)
         (case tp
           (:triangles (setq mesh-size 3))
           (:quads     (setq mesh-size 4))
           (t (warn ";; not supported mesh type -> ~A" tp)
              (return-from :convert-to-faces)))
         ;; convert from self coords
         (cond
          (idces
           (do ((i 0 (+ i 1))
                (v 0 (+ v mesh-size)))
               ((>= v (length idces)))
             (let (vs ns)
               (dotimes (j mesh-size)
                 (push (user::c-matrix-row vlst (elt idces (+ v j))) vs)
                 (if nlst (push (user::c-matrix-row nlst (elt idces (+ v j))) ns)))
               (setq vs (case wrt
                          (:world (mapcar #'(lambda (x) (send self :transform-vector x)) (nreverse vs)))
                          (:local (nreverse vs))
                          (t )))
               (push
                (cond
                 ;;((and ns check-vertices-order) )
                 ;;(ns (instance face :init :vertices vs :normal ns))
                 (t (instance face :init :vertices vs)))
                lst))))
          (t
           (do ((i 0 (+ i 1))
                (v 0 (+ v mesh-size)))
               ((>= v (array-dimension vlst 0)))
             (let (vs ns)
               (dotimes (j mesh-size)
                 (push (user::c-matrix-row vlst (+ v j)) vs)
                 (if nlst (push (user::c-matrix-row nlst (+ v j)) ns)))
               (setq vs (case wrt
                          (:world (mapcar #'(lambda (x) (send self :transform-vector x)) (nreverse vs)))
                          (:local (nreverse vs))
                          (t )))
               (push
                (if ns
                    (instance face :init :vertices vs :normal ns)
                  (instance face :init :vertices vs))
                lst))))
          ) ;; /cond
         (push (nreverse lst) ret)))
     (nreverse ret)))
  (:faces () (flatten (send self :convert-to-faces)))
  (:convert-to-faceset (&rest args)
   (let ((fs
          (instance faceset :init :faces
                    (flatten (send* self :convert-to-faces args)))))
     (send fs :transform (send self :worldcoords))
     fs))
  (:set-offset (cds) ;; not checked
   (let* ((zero (float-vector 0 0 0))
          (wpos (send cds :worldpos))
          (wrot (send cds :worldrot)))
     (dolist (minfo mesh-list)
       (let ((vmat (cadr (assoc :vertices minfo)))
             (nmat (cadr (assoc :normals minfo))))
         (user::c-coords-transform-vector
          wpos wrot vmat vmat)
         (when nmat
           (user::c-coords-transform-vector
            zero wrot nmat nmat))))
     self))
  (:glvertices (&optional (name) (test #'string=))
   (let (ret)
     (cond
      (name
       (let ((m (find-if #'(lambda (m) (funcall test name (cadr (assoc :name m)))) mesh-list)))
         (if m (setq ret (instance glvertices :init (list m))))))
      (t
       (dolist (minfo mesh-list)
         (push (instance glvertices :init (list minfo)) ret))
       (setq ret (nreverse ret))
       ))
     ret))
  (:append-glvertices (glv-lst) ;; not checked
   (if (atom glv-lst) (setq glv-lst (list glv-lst)))
   (let (ret (sw (send self :worldcoords)))
     (dolist (glv glv-lst)
       (let ((mlst (copy-object (glv . mesh-list)))
             (cds (send sw :transformation (send glv :worldcoords))))
         (let ((lglv (instance glvertices :init mlst)))
           (send lglv :set-offset cds)
           (setq ret (append ret mlst)))))
     (setq mesh-list (append mesh-list ret))
     (send self :calc-bounding-box)
     self))
  (:draw-on
   (&key ((:viewer vwer) *viewer*))
   ;; not implemented yet
   )
  (:draw (vwr &rest args)
   (let* (newlis
          (mat (send (send self :worldcoords) :4x4))
          (stransparent (get self :transparent))
          fcol
#+:jsk
          (glcon (cdr (assq (sys:thread-self) ((send vwr :viewsurface) . glcon))))
#-:jsk
          (glcon ((send vwr :viewsurface) . glcon))
          )
#+:jsk
     (sys::mutex-lock gl::*opengl-lock*)
     (send vwr :viewsurface :makecurrent) ;; ???
     (glPushAttrib GL_ALL_ATTRIB_BITS)
     (glpushmatrix)
     (glmultmatrixf (array-entity (transpose mat *temp-matrix*)))
     (gl::glLightModeli GL_LIGHT_MODEL_TWO_SIDE GL_TRUE) ;; draw back side
     (cond
      ((setq newlis (cdr (assq glcon (get self :GL-DISPLAYLIST-ID))))
       (glCallList newlis))
      (t
       ;; search face color
       (setq fcol (get self :face-color))
       (when (and fcol (not (vectorp fcol)))
         ;;(warn "draw-body: body ~A face-color ~A~%" abody col)
         (setq fcol (find-color fcol))
         (setf (get self :face-color) fcol))

       (setq newlis (glgenlists 1))
       (glnewlist newlis gl_compile)

       (when stransparent
         (glDepthMask GL_FALSE)
         (glEnable GL_BLEND)
         (glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA))

       (dolist (minfo mesh-list)
         (let ((mesh-type (cadr (assoc :type minfo)))
               (material-info (cadr (assoc :material minfo)))
               (vertices (cadr (assoc :vertices minfo)))
               (normals (cadr (assoc :normals minfo)))
               (indices (cadr (assoc :indices minfo)))
               (texcoords (cadr (assoc :texcoords minfo)))
               teximg)
           (cond
            (fcol ;; use :face-color
             (glColor3fv fcol)
             (glMaterialfv GL_BACK  GL_AMBIENT_AND_DIFFUSE
                           (if stransparent
                               (concatenate float-vector fcol (float-vector stransparent))
                             fcol))
             (glMaterialfv GL_FRONT GL_AMBIENT_AND_DIFFUSE (if stransparent
                                                               (float-vector 0.8 0.0 0.54 1)
                                                             (float-vector 0.8 0.0 0.54)))
             ;;(glMaterialfv GL_FRONT_AND_BACK GL_SPECULAR (float-vector 0.2 0.2 0.2 0.1))
             ;;(glMaterialfv GL_FRONT_AND_BACK GL_EMISSION (float-vector 0.1 0.1 0.1 0.1))
             )
            (material-info
             (let (;;(fnm   (cadr (assoc :filename material-info)))
                   (col   (cadr (assoc :color material-info)))
                   (amb   (cadr (assoc :ambient material-info)))
                   (diff  (cadr (assoc :diffuse material-info)))
                   (spec  (cadr (assoc :specular material-info)))
                   (emi   (cadr (assoc :emission material-info)))
                   (shin  (cadr (assoc :shininess material-info)))
                   (trans (cadr (assoc :transparency material-info))))
               (setq teximg (cadr (assoc :teximage material-info)))
               ;; override transparent ...
               ;;(if (and stransparent trans) (setq stransparent trans))
               (when col
                 (glColor3fv col)
                 (glMaterialfv GL_BACK GL_AMBIENT_AND_DIFFUSE
                               (if stransparent
                                   (concatenate float-vector col (float-vector stransparent))
                                 col)))
               (when amb
                 (if stransparent
                     (setf (elt amb 3) (float-vector stransparent)))
                 (glMaterialfv GL_BACK GL_AMBIENT amb))
               (when diff
                 (if stransparent
                     (setf (elt diff 3) (float-vector stransparent)))
                 (glMaterialfv GL_BACK GL_DIFFUSE diff))
               (when spec
                 (if stransparent
                     (setf (elt spec 3) (float-vector stransparent)))
                 (glMaterialfv GL_BACK GL_SPECULAR spec))
               (when emi
                 (if stransparent
                     (setf (elt emi 3) (float-vector stransparent)))
                 (glMaterialfv GL_BACK GL_EMISSION emi))
               (cond
                (shin
                 (glMaterialf GL_BACK GL_SHININESS shin))
                (t
                 (glMaterialf GL_BACK GL_SHININESS 0.0)
                 (glMaterialfv GL_BACK GL_SPECULAR (float-vector 0 0 0 0))
                 ))
               (when (and teximg texcoords)
                 (let ((im_width  (send teximg :width))
                       (im_height (send teximg :height))
                       (im_buf    (send teximg :entity)))
                   (glPixelStorei GL_UNPACK_ALIGNMENT 1)
                   ;;
                   (cond
                    ((derivedp teximg image::grayscale-image)
                     (glTexImage2D GL_TEXTURE_2D 0 GL_RGB im_width im_height
                                   0 GL_LUMINANCE GL_UNSIGNED_BYTE im_buf))
                    ((derivedp teximg image::color-image24)
                     (glTexImage2D GL_TEXTURE_2D 0 GL_RGB im_width im_height
                                   0 GL_RGB GL_UNSIGNED_BYTE im_buf))
                    ((derivedp teximg image::color-image32)
                     (glTexImage2D GL_TEXTURE_2D 0 GL_RGBA im_width im_height
                                   0 GL_RGBA GL_UNSIGNED_BYTE im_buf)))
                   ;;(gluBuild2DMipmaps GL_TEXTURE_2D GL_RGB im_width im_height GL_RGB GL_UNSIGNED_BYTE imbuf) ;; case 2
                   ;;
                   (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_WRAP_S GL_REPEAT)
                   (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_WRAP_T GL_REPEAT)
                   ;;
                   (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MIN_FILTER GL_NEAREST)
                   (glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MAG_FILTER GL_NEAREST)
                   ;;(glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MAG_FILTER GL_LINEAR) ;; case 2
                   ;;(glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MIN_FILTER GL_LINEAR_MIPMAP_LINEAR) ;; case 2

                   ;;(glTexEnvi GL_TEXTURE_ENV GL_TEXTURE_ENV_MODE GL_DECAL) ;; flat color
                   (glTexEnvi GL_TEXTURE_ENV GL_TEXTURE_ENV_MODE GL_MODULATE)
                   (glEnable GL_TEXTURE_2D)
                   ))
               ))
            (t ;; default color
             ;; Font face was set CW at glview.l (glFrontFace GL_CW)
             (glMaterialfv GL_BACK  GL_AMBIENT (float-vector 0.2 0.2 0.2 0.1))
             (glMaterialfv GL_BACK  GL_DIFFUSE (float-vector 1 1 1 1))
             (glMaterialfv GL_BACK  GL_SPECULAR (float-vector 0.2 0.2 0.2 0.1))
             (glMaterialfv GL_BACK  GL_EMISSION (float-vector 0.1 0.1 0.1 0.1))
             ;;
             (glMaterialfv GL_FRONT GL_AMBIENT (float-vector 0.8 0.0 0.54 1))
             (glMaterialfv GL_FRONT GL_DIFFUSE (float-vector 0.8 0.0 0.54 1))
             (glMaterialfv GL_FRONT GL_SPECULAR (float-vector 0.2 0.2 0.2 1))
             (glMaterialfv GL_FRONT GL_EMISSION (float-vector 0.1 0.1 0.1 1))
             )
            ) ;; /cond
           (unless (and teximg texcoords) (setq texcoords nil))
           ;;
           (glEnableClientState GL_VERTEX_ARRAY)
#+:x86_64
           (glVertexPointer 3 GL_DOUBLE 0 (array-entity vertices))
#-:x86_64
           (glVertexPointer 3 GL_FLOAT 0 (array-entity vertices))
           ;;
           (when normals
             (glEnableClientState GL_NORMAL_ARRAY)
#+:x86_64
             (glNormalPointer GL_DOUBLE 0 (array-entity normals))
#-:x86_64
             (glNormalPointer GL_FLOAT 0 (array-entity normals))
             )
           ;;
           (when texcoords
             (glEnableClientState GL_TEXTURE_COORD_ARRAY)
#+:x86_64
             (glTexCoordPointer 2 GL_DOUBLE 0 texcoords)
#-:x86_64
             (glTexCoordPointer 2 GL_FLOAT 0 texcoords)
             )

           (let (tp)
             (case mesh-type
               (:triangles (setq tp GL_TRIANGLES))
               (:quads     (setq tp GL_QUADS))
               (:lines     (setq tp GL_LINES))
               (nil (setq tp GL_TRIANGLES) (warn ";; keyword :type not found (processing as :triangles)~%"))
               (t (warn ";; mesh-type not found ~A~%") mesh-type))
             (when tp
               (cond
                (indices
#+:x86_64
                 (glDrawElements tp (length indices) GL_UNSIGNED_INT
                                 (user::lvector2integer-bytestring indices))
#-:x86_64
                 (glDrawElements tp (length indices) GL_UNSIGNED_INT indices)
                 )
                (t
                 (glDrawArrays tp 0 (array-dimension vertices 0)))
                )))

           (when texcoords
             (glDisable GL_TEXTURE_2D)
             (glDisableClientState GL_TEXTURE_COORD_ARRAY))
           (when normals (glDisableClientState GL_NORMAL_ARRAY))
           (glDisableClientState GL_VERTEX_ARRAY)
           ))

       (when stransparent
         (glDepthMask GL_TRUE)
         (glDisable GL_BLEND))

       (glendlist)
       (setf (get self :GL-DISPLAYLIST-ID)
             (cons (cons glcon newlis)
                   (get self :GL-DISPLAYLIST-ID)))
       (setq newlis nil)
       ))
     (gl::glLightModeli GL_LIGHT_MODEL_TWO_SIDE GL_TRUE) ;; return to default
     (glpopmatrix)
     (glpopattrib)
#+:jsk
     (sys::mutex-unlock *opengl-lock*)
     (unless newlis (send self :draw vwr))
     ))
  (:collision-check-objects (&rest args)) ;; compatibility to jsk library
  (:box ()
   (unless bbox
     (send self :calc-bounding-box))
   bbox)
  )

#| ;; sample of glvertices
(setq ver-lst (list
(float-vector 0 0 0) (float-vector 1 0 0) (float-vector 1 1 0) (float-vector 0 1 0)
(float-vector 1 0 0) (float-vector 1 0 -1) (float-vector 1 1 -1) (float-vector 1 1 0)
(float-vector 0 0 -1) (float-vector 0 1 -1) (float-vector 1 1 -1) (float-vector 1 0 -1)
(float-vector 0 0 -1) (float-vector 0 0 0) (float-vector 0 1 0) (float-vector 0 1 -1)
(float-vector 0 1 0) (float-vector 1 1 0) (float-vector 1 1 -1) (float-vector 0 1 -1)
(float-vector 0 0 0) (float-vector 0 0 -1) (float-vector 1 0 -1) (float-vector 1 0 0)
))

(setq nom-lst (list
(float-vector 0 0 1) (float-vector 0 0 1) (float-vector 0 0 1) (float-vector 0 0 1)
(float-vector 1 0 0) (float-vector 1 0 0) (float-vector 1 0 0) (float-vector 1 0 0)
(float-vector 0 0 -1) (float-vector 0 0 -1) (float-vector 0 0 -1) (float-vector 0 0 -1)
(float-vector -1 0 0) (float-vector -1 0 0) (float-vector -1 0 0) (float-vector -1 0 0)
(float-vector 0 1 0) (float-vector 0 1 0) (float-vector 0 1 0) (float-vector 0 1 0)
(float-vector 0 -1 0) (float-vector 0 -1 0) (float-vector 0 -1 0) (float-vector 0 -1 0)
))

(mapcar #'(lambda (v) (scale 1000.0 v v)) ver-lst)

;;(setq a (instance gl::glvertices :init))
(setq p (instance pointcloud :init :points ver-lst :normals nom-lst))


(setq l
(list (list :material '((:ambient #f(0.0 0.3 0.8)) (:diffuse #f(0.0 0.415686 0.921569)) (:specular #f(0.2 0.2 0.2)) (:emission #f(0.1 0.1 0.1)) (:shininess 0.22) (:transparency 0)))
      (list :material '((:color #f(1 1 1))))
      (list :vertices (send p :points))
      (list :normals (send p :normals))
      (list :type :quads)))

(setq a (instance gl::glvertices :init (list l)))
(objects (list a))
|#

(provide :irtgl "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.7  2009-08-03 06:26:32  eus
;;; add ia32 mode for x86_64 linux
;;;
;;; Revision 1.6  2009/03/12 13:43:02  k-okada
;;; fix for new cygwin/X
;;;
;;; Revision 1.5  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.4  2009/01/06 13:18:23  k-okada
;;; check event-type and window-id for skipping x event, use :expose for each item
;;;
;;; Revision 1.3  2008/11/06 17:19:47  k-okada
;;; fix to work with jskrbeusgl
;;;
;;; Revision 1.2  2008/09/22 06:02:10  k-okada
;;; fix to work with jskeusgl
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
