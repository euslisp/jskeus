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

(defun transpose-image-rows (img &optional ret)
  (let* ((h (send img :height))
         (step (* (send img :width) (send img :components)))
         (src (send img :entity)))
    (cond
      (ret
       (unless (and (eq (send img :height) (send ret :height))
                    (eq (send img :width) (send ret :width))
                    (eq (send img :components) (send ret :components)))
         (error "invalid return image format"))
       (ctranspose-image-rows h step src (send ret :entity))
       ret)
      (t
       (ctranspose-image-rows h step src)
       img))))

(unless (assoc :color-org (send glviewsurface :methods))
  (rplaca (assoc :color (send glviewsurface :methods)) :color-org))
(defmethod glviewsurface
  (:color 
   (&optional color-vector)
   "Returns color, if color-vector is given it set color"
   (if color-vector (send self :color-org color-vector)
     (let ((v (float-vector 0 0 0 0)))
       (glgetfloatv GL_CURRENT_COLOR v)
       (subseq v 0 3))))
  (:line-width 
   (&optional x)
   "Returns line width, if x is given, it set line width"
   (if x (glLineWidth (float x))
     (let ((tmp (float-vector 0)))
       (glGetFloatv GL_LINE_WIDTH tmp)
       (elt tmp 0))
     ))
  (:point-size 
   (&optional x)
   "Returns point size, if x is given, it set point size"
   (if x (glPointSize (float x))
     (let ((tmp (float-vector 0)))
       (glGetFloatv GL_POINT_SIZE tmp)
       (elt tmp 0))
     ))
  (:3d-point (pos &key (depth-test t) (lighting t)) ;; redefined
             "Draw 3D point"
	     (if depth-test (glDisable GL_DEPTH_TEST))
	     (if lighting (glDisable GL_LIGHTING))
	     (glBegin GL_POINTS)
	     (glVertex3fv pos)
	     (glEnd)
	     (if depth-test (glEnable GL_DEPTH_TEST))
	     (if lighting (glEnable GL_LIGHTING)))
  (:3d-line (start end &key (depth-test t) (lighting t)) ;; redefined
            "Draw 3D line from start to end"
	    (if depth-test (glDisable GL_DEPTH_TEST))
	    (if lighting (glDisable GL_LIGHTING))
	    (glBegin GL_LINE_STRIP)
	    (glVertex3fv start)
	    (glVertex3fv end)
	    (glEnd)
	    (if depth-test (glEnable GL_DEPTH_TEST))
	    (if lighting (glEnable GL_LIGHTING)))
  (:3d-lines (points &key (depth-test t) (lighting t))
             "Draw 3D lines that connecting points"
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
   "send glflush"
   (send self :makecurrent)
   (send self :glflush)
   (send-super :flush)
   )

  ;;
  (:write-to-image-file
   (file &key (x 0) (y 0)
	 (width x::width)
	 (height x::height))
   "Write current view to file name"
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
   "Get current view to a image object. It returns color-image24 object."
   (let ((img (instance image::color-image24 :init width height imgbuf)))
     (send self :makecurrent)
     (glReadBuffer GL_FRONT)
     (glPixelStorei GL_PACK_ALIGNMENT 1)
     (glReadPixels x y width height GL_RGB GL_UNSIGNED_BYTE imgbuf)
     #-(or :x86_64 :aarch64)
     (if depthbuf (glReadPixels x y width height GL_DEPTH_COMPONENT GL_FLOAT depthbuf))
     #+(or :x86_64 :aarch64)
     (when depthbuf
       (let ((fv (user::dvector2float-bytestring depthbuf)))
	 (glReadPixels x y width height GL_DEPTH_COMPONENT GL_FLOAT fv)
	 (user::float-bytestring2dvector fv depthbuf)))
     ;; transpose
     (transpose-image-rows img)
     img))
  )

(defun draw-globjects (vwr draw-things &key (clear t) (flush t) (draw-origin 150) (draw-floor nil) (floor-color #f(1 1 1)))
  (let (pcolor)
    (resetperspective (send vwr :viewing) (send vwr :viewsurface))
    (if clear (send vwr :viewsurface :clear))
    ;;(apply #'geo::draw things)
    (setq pcolor (send vwr :viewsurface :color))
    ;; draw origin
    (when draw-origin
      (let ((l (if (numberp draw-origin) draw-origin 150)))
        (glDisable GL_LIGHTING)
        (glBegin GL_LINES)
        (glColor3fv #f(1 0 0)) (glVertex3fv #f(0 0 0)) (glVertex3fv (float-vector l 0 0))
        (glColor3fv #f(0 1 0)) (glVertex3fv #f(0 0 0)) (glVertex3fv (float-vector 0 l 0))
        (glColor3fv #f(0 0 1)) (glVertex3fv #f(0 0 0)) (glVertex3fv (float-vector 0 0 l))
        (glEnd GL_LINES)
      (glEnable GL_LIGHTING)))
    ;; draw floor
    (when draw-floor
      (let* ((l (if (numberp draw-floor) draw-floor 1000))
             (s 5000) (-s (- s)))
        (glDisable GL_LIGHTING)
        (glBegin GL_LINES)
        (glColor3fv floor-color)
        (do ((y -s (+ y l)))
            ((> y s))
          (do ((x -s (+ x l)))
              ((> x s))
            (glVertex3fv (float-vector x -s 0))
            (glVertex3fv (float-vector x  s 0))
            (glVertex3fv (float-vector -s y 0))
            (glVertex3fv (float-vector  s y 0))))
        (glEnd GL_LINES)
      (glEnable GL_LIGHTING)))
    ;;
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

        (dolist (aface (send abody :faces))
          (glBegin GL_POLYGON)
          (glNormal3fv (transform (transpose (send abody :worldrot)) (send aface :normal)))
          (dolist (e (send aface :edges))
            (glVertex3fv (send abody :inverse-transform-vector (send e :pvertex aface))))
          (glEnd))

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
  "returns color vector of given color name, the name is defiend in https://github.com/euslisp/jskeus/blob/master/irteus/irtglrgb.l"
  (let (v c)
    (setq v
      (cond
       ((null color) nil)
       ((derivedp color colormaterial) (send color :diffuse))
       ((vectorp color) color)
       ((listp color) (float-vector (/ (elt color 0) 255.0) (/ (elt color 1) 255.0) (/ (elt color 2) 255.0)))
       ((symbolp color)
	(setq c (find-if #'(lambda (o) (eq (send o :name) color))
			 *face-colors*))
	(if c (send c :diffuse) (warn "Color name not found ")))
       (t color)))
    (unless v (setq v (float-vector 0.5 0.5 0.5)))
    v))

(defun transparent (abody param)
  "Set abody to transparent with param"
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
   (if flush (send vwer :viewsurface :makecurrent))
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
   (if flush (send vwer :viewsurface :makecurrent))
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
   "Set color of given color name, color sample and color name are referenced from http://en.wikipedia.org/wiki/X11_color_names"
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
   (if flush (send vwer :viewsurface :makecurrent))
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
   (if flush (send vwer :viewsurface :makecurrent))
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
   (if flush (send vwer :viewsurface :makecurrent))
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
   "set color as float vector of 3 elements, and transparent as float, all values are betwenn 0 to 1"
   (delete-displaylist-id (get self :GL-DISPLAYLIST-ID))
   (setf (get self :GL-DISPLAYLIST-ID) nil)
   (setf (get self :transparent) transparent)
   (if color
       (setf (get self :face-color) (gl::find-color color))
     (setf (get self :face-color) nil)))
  ;; (:check-normal ())
  ;; (:update ()) ;; call update when updating coordinates
  (:get-meshinfo (key &optional (pos -1))
   (when (< pos 0)
     (return-from :get-meshinfo (mapcar #'(lambda (info) (cadr (assoc key info))) mesh-list)))
   (cadr (assoc key (elt mesh-list pos))))
  (:set-meshinfo (key info &optional (pos -1))
   (when (< pos 0)
     (dolist (meshinfo mesh-list)
       (delete (assoc key meshinfo) meshinfo)
       (nconc meshinfo (list (list key info))))
     (return-from :set-meshinfo))
   (let ((meshinfo (elt mesh-list pos)))
     (delete (assoc key meshinfo) meshinfo)
     (nconc meshinfo (list (list key info)))
     (send self :get-meshinfo key pos)))
  (:get-material (&optional (pos -1))
   (send self :get-meshinfo :material pos))
  (:set-material
   (mat &optional (pos -1))
   (send self :set-meshinfo :material mat pos))
  (:actual-vertices ()
   "return list of vertices(float-vector), it returns all vertices of this object"
   (let (ret)
     (dolist (minfo mesh-list)
       (let ((v (cadr (assoc :vertices minfo))))
         (when v
           (dotimes (i (array-dimension v 0))
             (push (user::c-matrix-row v i) ret)))))
     ret))
  (:calc-bounding-box ()
   "calculate and set bounding box of this object"
   (setq bbox (make-bounding-box (send self :actual-vertices) 0.0))
   bbox)
  (:vertices ()
   "return list of vertices(float-vector), it returns vertices of bounding box of this object"
   (unless bbox
     (send self :calc-bounding-box))
   (send (send bbox :body) :vertices))
  (:reset-offset-from-parent ()
   "move vertices in this object using self transformation, this method change values of vertices. coordinates's method such as :transform just change view of this object"
   (let ((cds (send self :copy-coords)))
     (send self :set-offset cds)
     (send self :reset-coords))
   self)
  (:expand-vertices ()
   "expand vertices number as same number of indices, it enable to set individual normal to every vertices"
   (let (new-mesh)
     (dolist (minfo mesh-list)
       (push (send self :expand-vertices-info minfo) new-mesh))
     (setq mesh-list new-mesh)
     ))
  (:expand-vertices-info (minfo)
   (let ((idxs (cadr (assoc :indices minfo)))
         (vmat (cadr (assoc :vertices minfo)))
         (p (instantiate float-vector 3)))
     (if (> (length idxs) (array-dimension vmat 0)) ;; TODO: redundancy check in indices
       (let ((nvmat (make-matrix (length idxs) 3))
             (nidxs (instantiate integer-vector (length idxs))))
         (setq minfo (delete (assoc :vertices minfo) minfo))
         (setq minfo (delete (assoc :indices minfo) minfo))
         (setq minfo (delete (assoc :normals minfo) minfo))
         (dotimes (i (length idxs))
           (setf (elt nidxs i) i)
           (user::c-matrix-row vmat (elt idxs i) p)
           (user::c-matrix-row nvmat i p t))
         (setq minfo (nconc minfo (list (list :vertices nvmat) (list :indices nidxs))))
         minfo)
       minfo)))
  (:use-flat-shader ()
   "use flat shader mode, use opengl function of glShadeModel(GL_FLAT)"
   (dolist (minfo mesh-list)
     (nconc minfo (list (list :flat t)))))
  (:use-smooth-shader ()
   "use smooth shader mode, use opengl function of glShadeModel(GL_SMOOTH) {default}"
   (dolist (minfo mesh-list)
     (let ((a (assoc :flat minfo)))
       (if a (delete a minfo)))))
  (:calc-normals (&optional (force nil) (expand t) (flat t))
   "normal calculation
if force option is true, clear current normalset.
if exapnd option is ture, do :expand-vertices.
if flat option is true, use-flat-shader"
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
         (when expand
           (send self :expand-vertices-info minfo)
           (setq idxs (cadr (assoc :indices minfo)))
           (setq vmat (cadr (assoc :vertices minfo))))
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
           (when flat
             (nconc minfo (list (list :flat t))))
           ))
       )))
  (:mirror-axis
   (&key (create t) (invert-faces t) (axis :y))
   "creating mirror vertices respect to :axis"
   (case axis
     (:x (setq axis 0))
     (:y (setq axis 1))
     (:z (setq axis 2)))
   (let ((ret (copy-object mesh-list))
         (p (instantiate float-vector 3)))
     (dolist (mesh ret)
       (let* ((vts (cadr (assoc :vertices mesh)))
              (len (array-dimension vts 0)))
         (when len
           (dotimes (j len)
             (user::c-matrix-row vts j p)
             (setf (elt p axis) (- (elt p axis)))
             (user::c-matrix-row vts j p t))
           (when invert-faces
             (let* ((idx (cadr (assoc :indices mesh)))
                    (idx-len (/ (length idx) 3)) ;; mesh should be triangle
                    i0 i1 i2)
               (dotimes (i idx-len)
                 (setq i0 (elt idx (* i 3))
                       i1 (elt idx (+ (* i 3) 1))
                       i2 (elt idx (+ (* i 3) 2)))
                 (setf (elt idx (* i 3))       i2)
                 (setf (elt idx (+ (* i 3) 1)) i1)
                 (setf (elt idx (+ (* i 3) 2)) i0))
               )))
         ))
     (if create
         (instance glvertices :init ret)
       (progn
         (setq mesh-list ret)
         self))
     ))
  (:convert-to-faces (&rest args &key (wrt :local) &allow-other-keys) ;; check-normal <-> check-vertices-order, add face-color
   "create list of faces using vertices of this object"
   (let (ret)
     (dolist (mesh mesh-list)
       (let* ((vlst (cadr (assoc :vertices mesh)))
              (nlst (cadr (assoc :normals mesh)))
              (idces (cadr (assoc :indices mesh)))
              (idlen (length idces))
              (tp (cadr (assoc :type mesh)))
              mesh-size lst)
         (case tp
           (:triangles (setq mesh-size 3))
           (:quads     (setq mesh-size 4))
           (t (warn ";; not supported mesh type -> ~A" tp)
              (return-from :convert-to-faces)))
         (setq idlen (* (/ idlen mesh-size) mesh-size))
         ;; convert from self coords
         (cond
          (idces
           (do ((i 0 (+ i 1))
                (v 0 (+ v mesh-size)))
               ((>= v idlen))
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
                (cond
                 ;;((and ns check-vertices-order) )
                 ;;(ns (instance face :init :vertices vs :normal ns))
                 (t (instance face :init :vertices vs)))
                lst))))
          ) ;; /cond
         (push (nreverse lst) ret)))
     (nreverse ret)))
  (:faces () (flatten (send self :convert-to-faces :wrt :world)))
  (:convert-to-faceset (&rest args)
   "create faceset using vertices of this object"
   (let ((fs
          (instance faceset :init :faces
                    (flatten (send* self :convert-to-faces args)))))
     (send fs :transform (send self :worldcoords))
     fs))
  (:set-offset (cds &key (create))
   "move vertices in this object using given coordinates, this method change values of vertices. coordinates's method such as :transform just change view of this object"
   (let* ((zero (float-vector 0 0 0))
          (wpos (send cds :worldpos))
          (wrot (send cds :worldrot))
          ret)
     (if create
         (setq ret
               (instance glvertices :init (copy-object mesh-list)))
       (setq ret self))
     (dolist (minfo (ret . mesh-list))
       (let ((vmat (cadr (assoc :vertices minfo)))
             (nmat (cadr (assoc :normals minfo))))
         (user::c-coords-transform-vector
          wpos wrot vmat vmat)
         (when nmat
           (user::c-coords-transform-vector
            zero wrot nmat nmat))))
     ret))
  (:convert-to-world (&key (create))
   "move vertices in this object using self's coordinates. vertices data should be moved as the same as displayed"
   (let ((ret (send self :set-offset (send self :worldcoords) :create create)))
     (send ret :reset-coords)
     (send self :worldcoords)
     ret))
  (:glvertices (&optional (name) (test #'string=))
   "create individual glvertices objects from mesh-list. if name is given, search mesh has the same name"
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
  (:append-glvertices (glv-lst)
   "append list of glvertices to this object"
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
               (flat-shade (cadr (assoc :flat minfo)))
               teximg)
           (when flat-shade
             (glEnable GL_FLAT)
             (glShadeModel GL_FLAT))
           (cond
            (fcol ;; use :face-color
             (glColor3fv fcol)
             (cond
              (stransparent
               (glMaterialfv GL_BACK
                             GL_AMBIENT_AND_DIFFUSE
                             (concatenate float-vector fcol (float-vector stransparent)))
               (glMaterialfv GL_FRONT
                             GL_AMBIENT_AND_DIFFUSE (float-vector 0 0 0 0)))
              (t
               (glMaterialfv GL_BACK  GL_AMBIENT_AND_DIFFUSE fcol)
               (glMaterialfv GL_FRONT GL_AMBIENT_AND_DIFFUSE (float-vector 0.8 0.0 0.54))
               ))
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
                     (setf (elt amb 3) stransparent))
                 (glMaterialfv GL_BACK GL_AMBIENT amb))
               (when diff
                 (if stransparent
                     (setf (elt diff 3) stransparent))
                 (glMaterialfv GL_BACK GL_DIFFUSE diff))
               (when spec
                 (if stransparent
                     (setf (elt spec 3) stransparent))
                 (glMaterialfv GL_BACK GL_SPECULAR spec))
               (when emi
                 (if stransparent
                     (setf (elt emi 3) stransparent))
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
               (when stransparent
                 (glMaterialfv GL_FRONT
                               GL_AMBIENT_AND_DIFFUSE (float-vector 0 0 0 0)))
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
#+(or :x86_64 :aarch64)
           (glVertexPointer 3 GL_DOUBLE 0 (array-entity vertices))
#-(or :x86_64 :aarch64)
           (glVertexPointer 3 GL_FLOAT 0 (array-entity vertices))
           ;;
           (when normals
             (glEnableClientState GL_NORMAL_ARRAY)
#+(or :x86_64 :aarch64)
             (glNormalPointer GL_DOUBLE 0 (array-entity normals))
#-(or :x86_64 :aarch64)
             (glNormalPointer GL_FLOAT 0 (array-entity normals))
             )
           ;;
           (when texcoords
             (glEnableClientState GL_TEXTURE_COORD_ARRAY)
#+(or :x86_64 :aarch64)
             (glTexCoordPointer 2 GL_DOUBLE 0 texcoords)
#-(or :x86_64 :aarch64)
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
#+(or :x86_64 :aarch64)
                 (glDrawElements tp (length indices) GL_UNSIGNED_INT
                                 (user::lvector2integer-bytestring indices))
#-(or :x86_64 :aarch64)
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
           (when flat-shade
             (glEnable GL_SMOOTH)
             (glShadeModel GL_SMOOTH))
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
  (:make-pqpmodel (&key (fat 0))
   (let ((m (geo::pqpmakemodel))
         v1 v2 v3 (id 0))
     (dolist (mesh mesh-list) ;; check triangle model
       (let ((tp (cadr (assoc :type mesh))))
         (unless (eq tp :triangles)
           (warn ";; not supported mesh type -> ~A" tp)
           (geo::pqpdeletemodel m)
           (return-from :make-pqpmodel nil))))
     (setq v1 (float-vector 0 0 0))
     (setq v2 (float-vector 0 0 0))
     (setq v3 (float-vector 0 0 0))
     (geo::pqpbeginmodel m)
     (dolist (mesh mesh-list)
       (let* ((vlst (cadr (assoc :vertices mesh)))
              (idces (cadr (assoc :indices mesh)))
              (idlen (* (/ (length idces) 3) 3)))
         (cond
           (idces
            (do ((v 0 (+ v 3)))
                ((>= v idlen))
              (user::c-matrix-row vlst (elt idces    v   ) v1)
              (user::c-matrix-row vlst (elt idces (+ v 1)) v2)
              (user::c-matrix-row vlst (elt idces (+ v 2)) v3)
              (when (not (= fat 0))
                (v+ v1 (scale fat (normalize-vector v1)) v1)
                (v+ v2 (scale fat (normalize-vector v2)) v2)
                (v+ v3 (scale fat (normalize-vector v3)) v3))
              (geo::pqpaddtri m v1 v2 v3 id)
              (incf id)
              ))
           (t
            (do ((v 0 (+ v 3)))
                ((>= v (array-dimension vlst 0)))
              (user::c-matrix-row vlst    v    v1)
              (user::c-matrix-row vlst (+ v 1) v2)
              (user::c-matrix-row vlst (+ v 2) v3)
              (when (not (= fat 0))
                (v+ v1 (scale fat (normalize-vector v1)) v1)
                (v+ v2 (scale fat (normalize-vector v2)) v2)
                (v+ v3 (scale fat (normalize-vector v3)) v3))
              (geo::pqpaddtri m v1 v2 v3 id)
              (incf id)
              )))
         ))
     (geo::pqpendmodel m)
     m
     ))
  )

;;;
(defclass glbody
  :super body
  :slots (aglvertices)
  )
(defmethod glbody
  (:glvertices (&rest args) (user::forward-message-to aglvertices args))
  (:draw (vwr)
   (when aglvertices
     (send aglvertices :draw vwr)))
  (:set-color (&rest args)
   (send-super* :set-color args)
   (when aglvertices (send* aglvertices :set-color args)))
  )

;;; glvertices utility
(defun make-glvertices-from-faceset (fs &key (material))
  "returns glvertices instance
fs is geomatry::faceset"
  (let (mat)
    (cond
     (material (setq mat material))
     ((get fs :face-color)
      (let ((col (get fs :face-color)))
        (unless (vectorp col)
          (setq col (gl::find-color col)))
        (setq mat
              (list (list :ambient (float-vector (elt col 0) (elt col 1) (elt col 2)))
                    (list :diffuse (float-vector (elt col 0) (elt col 1) (elt col 2)))))))
     (t ;; use default material
      (setq mat
            (list (list :ambient (float-vector 0.65 0.65 0.65))
                  (list :diffuse (float-vector 0.65 0.65 0.65))))))
    (make-glvertices-from-faces (send fs :faces) :material mat)))

(defun make-glvertices-from-faces (flst &key (material))
  "returns glvertices instance
flst is list of geomatry::face"
  (setq flst (flatten (mapcar #'(lambda (f) (geometry::face-to-triangle-aux f)) flst)))
  (let ((mat (make-matrix (* 3 (length flst)) 3))
        (nom (make-matrix (* 3 (length flst)) 3))
        (idx (instantiate integer-vector (* 3 (length flst))))
        (cntr 0))
    (dolist (f flst)
      (let ((nm (normalize-vector (send f :normal)))
            (vsl (send f :vertices)))
        ;;
        (user::c-matrix-row mat cntr (car vsl) t)
        (user::c-matrix-row nom cntr nm t)
        (incf cntr)
        ;;
        (user::c-matrix-row mat cntr (cadr vsl) t)
        (user::c-matrix-row nom cntr nm t)
        (incf cntr)
        ;;
        (user::c-matrix-row mat cntr (caddr vsl) t)
        (user::c-matrix-row nom cntr nm t)
        (incf cntr)
        ))
    (dotimes (i (length idx)) (setf (elt idx i) i))
    (let ((msh
           (list (list :type :triangles)
                 (list :vertices mat)
                 (list :normals nom)
                 (list :indices idx))))
      (when material
        (push (list :material material) msh))
      (instance gl::glvertices :init (list msh)))
    ))

(defun _dump-wrl-shape (strm mesh &key ((:scale scl) 1.0) (use_ambient nil)
                             (use_normal nil) (use_texture nil)
                             &allow-other-keys)
  (let* ((tp   (cadr (assoc :type      mesh))) ;; type should be :triangle
         (indx (cadr (assoc :indices   mesh))) ;; should have index
         (vmat (cadr (assoc :vertices  mesh)))
         (matl (cadr (assoc :material  mesh)))
         (tcds (cadr (assoc :texcoords mesh)))
         (nmat (cadr (assoc :normals mesh)))
         (len (/ (length (array-entity vmat)) 3)))
    (format strm "    Shape {~%")
    (let ((col   (cadr (assoc :color matl)))
          (amb   (cadr (assoc :ambient matl)))
          (diff  (cadr (assoc :diffuse matl)))
          (spec  (cadr (assoc :specular matl)))
          (emi   (cadr (assoc :emission matl)))
          (shin  (cadr (assoc :shininess matl)))
          (trans (cadr (assoc :transparency matl)))
          (matfile (cadr (assoc :filename matl))))
      (format strm "      appearance Appearance {
        material Material {~%")
      (if diff
          (format strm "          diffuseColor     ~A ~A ~A~%" (elt diff 0) (elt diff 1) (elt diff 2)))
      (if (and use_ambient amb)
          (format strm "          diffuseColor     ~A ~A ~A~%" (elt amb 0) (elt amb 1) (elt amb 2)))
      (if spec
          (format strm "          specularColor    ~A ~A ~A~%" (elt spec 0) (elt spec 1) (elt spec 2)))
      (if emi
          (format strm "          emissiveColor    ~A ~A ~A~%" (elt emi 0) (elt emi 1) (elt emi 2)))
      (if shin
          (format strm "          shininess        ~A~%" shin))
      (if trans
          (format strm "          transparency     ~A~%" trans))
      (format strm "          ambientIntensity 0~%") ;; ??
      (format strm "        }~%")
      (format strm "      }~%" )
      )
    (format strm "      geometry IndexedFaceSet {
        ccw    TRUE
        convex TRUE
        solid  FALSE
        creaseAngle 0
        coord  Coordinate {
          point [~%")
    (let ((p (float-vector 0 0 0)))
      (dotimes (i len)
        (user::c-matrix-row vmat i p)
        (format strm "~8,8f ~8,8f ~8,8f,~%" (* scl (elt p 0)) (* scl (elt p 1)) (* scl (elt p 2)))
        ))
    (format strm "                ]
        }~%")
    (when (and use_normal nmat)
    ;;;; normal
    ;;(format strm "        normal Normal { vector [~%")
    ;; nx ny nz,
    ;;(format strm "        ] } #normal~%")
      )
    (when (and use_texture tcds)
    ;;;; texcoords
    ;;(format strm "        texCoord TextureCoordinate { point [~%")
    ;; t s,
    ;;(format strm "        ] } #texCoord~%")
      )
    (format strm "        coordIndex [~%")
    (dotimes (i (/ (length indx) 3))
      (format strm "~D, ~D, ~D, -1,~%"
              (elt indx    (* i 3))
              (elt indx (+ (* i 3) 1))
              (elt indx (+ (* i 3) 2))))
    (format strm "        ]
      } #IndexedFaceSet
    } #Shape~%")
    ))

(defun write-wrl-from-glvertices (fname glv &rest args)
  "write .wrl file from instance of glvertices"
  (let ((mesh-list (glv . gl::mesh-list)))
    (with-open-file
     (strm fname :direction :output)
     (format strm "Transform {~%")
     (format strm "  children [~%")
     (dolist (mesh mesh-list)
       (apply #'_dump-wrl-shape strm mesh args))
     (format strm "  ] #children~%")
     (format strm "} #Transform~%")
     )))

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
