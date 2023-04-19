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

(in-package "X")

(defun make-lr-ud-coords (lra uda)
  (let ((c (geo::make-coords)))
    (send c :rotate (deg2rad lra) :z)
    (send c :rotate (- (deg2rad uda)) :y)
    c))

(defmethod geo::viewer
  (:draw-circle (c &key (radius 50) (flush nil) (arrow nil) (arc 2pi) (arrow-scale #f(1 1)))
    (let* ((s 16) (sr (/ arc s)) p0 p1)
      (dotimes (i s)
	(setq p1 
	  (send c :transform-vector
		(float-vector (* radius (sin (* i sr)))
			      (* radius (cos (* i sr))) 0)))
	(if p0 (send self :draw-line p0 p1))
	(setq p0 p1))
      (setq p1 (send c :transform-vector
		     (float-vector (* radius (sin (* s sr)))
				   (* radius (cos (* s sr))) 0)))
      (if arrow
	  (send self :draw-arrow p0 p1 t nil :arrow-scale arrow-scale)
	(send self :draw-line p0 p1))
      (if flush (send self :flush))
      ))
  (:draw-objects
    (&rest args)
    (send* (get self :pickviewer) :draw-objects args)))
(defvar gl::*light3*)
(defclass irtviewer
  :super x::panel
  :slots (viewer
	  objects draw-things
	  previous-cursor-pos
	  left-right-angle
	  up-down-angle
	  viewpoint
	  viewtarget
	  drawmode
          draw-origin
          draw-floor
          floor-color
          image-sequence
          logging-flag))
(defmethod irtviewer
  (:create
   (&rest args
	  &key (title "IRT viewer")
	  (view-name (gensym "title"))
	  (hither 200.0) (yon 50000.0)
	  (width 500) (height 500)
          ((:draw-origin do) 150) ((:draw-floor df) nil) ((:floor-color fc) #f(1 1 1))
          (call-super t)
	  &allow-other-keys)
   (let ()
     (setq left-right-angle 60
	   up-down-angle 20
	   viewpoint (float-vector 700 400 250)
	   viewtarget (float-vector 0 0 0)
           draw-origin do draw-floor df floor-color fc
           logging-flag nil)
     (if call-super
         (send-super* :create :width width :height height :title title
                      :event-mask '(:configure) args))
     (setq gl::*perspective-far* yon)
     (setq gl::*perspective-near* hither)
     (setq viewer (apply #'user::view :parent self
			 :x 0 :y 0
			 :width width :height height 
			 :title title
			 :name view-name
			 args))
     (setf (get viewer :pickviewer) self)
     (setq user::*viewer* viewer)

     (let ((default-ambient (float-vector 0.1 0.1 0.1 1.0))
           (default-diffuse (float-vector 1.0 1.0 1.0 1.0))
           (default-specular (float-vector 0.1 0.1 0.1 1.0))
           (scale-light0 1.00)
           (scale-light1 0.70)
           (scale-light2 0.40)
           (scale-light3 0.25))
       (setq gl::*light0* ;; from back
             (instance gl::lightsource :create 0
                       :ambient (scale scale-light0 default-ambient)
                       :diffuse (scale scale-light0 default-diffuse)
                       :specular (scale scale-light0 default-specular)
                       :position (float-vector 4000.0 3000.0 0.0 0.0)))
       (setq gl::*light1* ;; from front-left
             (instance gl::lightsource :create 1
                       :ambient (scale scale-light1 default-ambient)
                       :diffuse (scale scale-light1 default-diffuse)
                       :specular (scale scale-light1 default-specular)
                       :position (float-vector -4000.0 -2000.0 -2000.0 0.0)))
       (setq gl::*light2* ;; from front-left
             (instance gl::lightsource :create 2
                       :ambient (scale scale-light2 default-ambient)
                       :diffuse (scale scale-light2 default-diffuse)
                       :specular (scale scale-light2 default-specular)
                       :position (float-vector -2000.0 -2000.0 2500.0 0.0)))
       (setq gl::*light3*
             (instance gl::lightsource :create 3
                       :ambient (scale scale-light3 default-ambient)
                       :diffuse (scale scale-light3 default-diffuse)
                       :specular (scale scale-light3 default-specular)
                       :position (float-vector 0.0 0.0 0.0 1.0)))
       )
     (send gl::*light0* :opengl)
     (send gl::*light1* :opengl)
     (send gl::*light2* :opengl)
     (send gl::*light3* :opengl)
     ;;
     (send viewer :viewsurface :set-event-proc
	   :buttonrelease-left :draw-event self)
     (send viewer :viewsurface :set-event-proc
	   :buttonpress :set-cursor-pos-event self)
     (send viewer :viewsurface :set-event-proc
	   :motionnotify-left :move-coords-event self)
     (send viewer :viewsurface :set-event-proc
	   :motionnotify-middle :move-coords-event self)

     (send self :look1)
     (send self :draw-objects)
     self))
  (:viewer (&rest args) (user::forward-message-to viewer args))
  (:redraw () (send self :draw-objects))
  ;;
  (:expose (event) )
  (:resize (newwidth newheight)
   (setq x::width newwidth x::height newheight)
   (send-super  :resize newwidth newheight)
   (send viewer :viewsurface :resize newwidth newheight)
   (send viewer :viewsurface :makecurrent)
   (send viewer :viewsurface :3d-mode)
   (let ((wh-max (max newwidth newheight)))
     (send (send viewer :viewport)
           :resize :width wh-max :height (- wh-max)
           :xcenter (/ newwidth 2) :ycenter (/ newheight 2)))
   (send self :redraw))
  (:configureNotify
   (event)
   (if *debug* (warn "ConfigureNotify came to ~S~%" self))
   (let ((newwidth (send-message self x::xwindow :width))
	 (newheight (send-message self x::xwindow :height)))
     (when (or (/= newwidth x::width) (/= newheight x::height))
       (send self :resize newwidth newheight))))
  ;;
  (:viewtarget (&optional p) (if p (setq viewtarget p)) viewtarget)
  (:viewpoint  (&optional p) (if p (setq viewpoint p))  viewpoint)
  (:look1
   (&optional (vt viewtarget)
       (lra left-right-angle) (uda up-down-angle))
   (let* ((c (make-lr-ud-coords lra uda))
	  (vp
	   (v+ vt (send c :transform-vector
			(float-vector (norm (v- viewpoint vt)) 0 0))))
	  (view-up (send c :rotate-vector (float-vector 0 0 1))))
     (send self :viewpoint vp)
     (if (send self :viewer :viewing)
	 (send self :viewer :viewing :look viewpoint vt view-up))
     ))
  (:look-all
   (&optional bbox)
   (cond ((derivedp bbox geo::bounding-box))
         ((null bbox)
	  (send-all x::draw-things :worldcoords)
          (setq bbox (geo::make-bounding-box (flatten (send-all x::draw-things :vertices)))))
         (bbox
          (setq bbox (geo::make-bounding-box (flatten (send-all (x::draw-things bbox) :vertices))))))
   (when bbox
     (let (vt vp sc)
       (setq vt (send bbox :center))
       (setq sc (/ (apply #'max (coerce (send bbox :diagonal) cons))
		   (* 0.5 (tan (send self :viewer :viewing :view-angle)))))
       (when (or (> (/ sc gl::*perspective-far*) 2)
		 (< (/ sc gl::*perspective-far*) 0.001))
	 (warning-message 2 "Detect very large/small objects(~A), change perspective~%" (/ sc gl::*perspective-far*) )
	 (setq gl::*perspective-near* (* (/ sc gl::*perspective-far*) gl::*perspective-near*)
	       gl::*perspective-far*  (* (/ sc gl::*perspective-far*) gl::*perspective-far*)))
       (setq vp
	 (scale
          (min
           (* gl::*perspective-far* 0.8)
           (max (* gl::*perspective-near* 1.5) sc))
          (normalize-vector (v- viewpoint viewtarget))))
       (send self :viewtarget vt)
       (send self :viewpoint (v+ vt vp))
       ))
   (send self :look1)
   (send self :draw-objects))
  (:move-viewing-around-viewtarget
   (event x y dx dy vwr)
   (cond
       ((event-middle event)
	(let* ((vtp (v- viewpoint viewtarget))
               (v (send vwr :viewing :transform-vector
                        (scale (* 0.001 (norm vtp))
                               (float-vector (* -1 dx) dy 0))))
	       (tp (send self :viewtarget))
	       (dv (v- v viewpoint)))
	  (send self :viewpoint v)
	  (send self :viewtarget (v+ tp dv))))
       ((or (event-right event)
	    (and (< 60 x (- (send vwr :viewsurface :width) 60))
		 (< 60 y (- (send vwr :viewsurface :height) 60))
                 (event-left event)))
	(setq left-right-angle (- left-right-angle dx))
	(setq up-down-angle (+ up-down-angle dy)))
       ((or (> x (- (send vwr :viewsurface :width) 60)) (< y 60))
        (let* ((vtp (v- viewpoint viewtarget)))
          (send self :viewpoint
                (v+ viewpoint
                    (scale (* 0.01 (if (< y 60) dx dy) (1+ (norm vtp)))
                           (normalize-vector vtp))))))
       ((or (> y (- (send vwr :viewsurface :height) 60)) (< x 60))
	(let* ((vtp (v- viewpoint viewtarget))
               (v (send vwr :viewing :transform-vector
                        (scale (* 0.001 (norm vtp))
                               (if (< x 60)
                                   (float-vector 0 dy 0)
                                 (float-vector (* -1 dx) 0 0)))))
	       (tp (send self :viewtarget))
	       (dv (v- v viewpoint)))
	  (send self :viewpoint v)
	  (send self :viewtarget (v+ tp dv))
	  ))))
  (:set-cursor-pos-event
   (event)
   (setq previous-cursor-pos (event-pos event)))
  (:move-coords-event
   (event)
   (if (null previous-cursor-pos)
       (setq previous-cursor-pos (event-pos event)))
   (let* ((current-pos (event-pos event))
	  (x (elt current-pos 0))
	  (y (elt current-pos 1))
	  (dx (- x (elt previous-cursor-pos 0)))
	  (dy (- y (elt previous-cursor-pos 1)))
	  )
     (send self :move-viewing-around-viewtarget event x y dx dy viewer)
     (send self :look1)
     (send self :draw-objects)
     (setq previous-cursor-pos current-pos)))
  (:draw-event
   (event)
   (send self :draw-objects))
  (:draw-objects
   (&rest args)
   (send viewer :viewsurface :makecurrent)
   (if logging-flag (send self :push-image))
   (apply #'gl::draw-globjects viewer draw-things :draw-origin draw-origin :draw-floor draw-floor :floor-color floor-color args))
  (:objects
   (&rest args)
   (when
       args
     (cond
	((null (car args))
	 (setq objects nil))
	((consp (car args))
	 (setq objects (car args)))
	((atom (car args))
	 (setq objects (append args objects)))
	 ))
   (send-all objects :worldcoords)
   (setq draw-things (draw-things objects))
   objects)
  (:select-drawmode
   (mode)
   (when (not (eq mode drawmode))
     (let ((glcon ((send self :viewer :viewsurface) . gl::glcon)))
       (dolist (obj draw-things)
         (gl::delete-displaylist-id (get obj :GL-DISPLAYLIST-ID))
         (setf (get obj :GL-DISPLAYLIST-ID) nil)
         (case mode
           (user::hid
            (setf (get obj :gl-hiddenline) (list (cons glcon t))))
           (t
            (setf (get obj :gl-hiddenline) nil))))))
   (setq drawmode mode)
   drawmode)
  (:flush () (if viewer (send viewer :viewsurface :glflush)))
  (:change-background (col)
   (let ((vs (send self :viewer :viewsurface)))
     (send vs :makecurrent)
     (setq (vs . x::bg-color) col)
     (gl::glClearColorfv col)))
  (:draw-origin
   (&optional (tmp-draw-origin :null))
   "get/set draw-origin"
   (if (not (eq tmp-draw-origin :null)) (setq draw-origin tmp-draw-origin) draw-origin))
  (:draw-floor
   (&optional (tmp-draw-floor :null))
   "get/set draw-floor"
   (if (not (eq tmp-draw-floor :null)) (setq draw-floor tmp-draw-floor) draw-floor))
  (:floor-color
   (&optional (tmp-floor-color :null))
   "get/set floor-color"
   (if (not (eq tmp-floor-color :null)) (setq floor-color tmp-floor-color) floor-color))
  )

;; save functions
(defun make-mpeg-from-images
    (mpgfile images
             &key (delete t)
             (delay 1))
  (let ((counter 0) (basename (pathname-name mpgfile))
        (random-str (random 65536))
	ppm-filename cmd)
    ;; check animgif commands (convert, ffmpeg)
    (when (/= (unix::system "which convert > /dev/null") 0)
      (warning-message 1 "(make-animgif-from-images) requires 'convert' command.~%")
      (return-from make-mpeg-from-images nil))
    (when (/= (unix::system "which ffmpeg > /dev/null") 0)
      (warning-message 1 "(make-mpeg-from-images) requires 'ffmpeg' command.~%")
      (return-from make-mpeg-from-images nil))
    ;;
    (if (string= ".mpg" (subseq basename (- (length basename) 4)))
	(setq basename (subseq basename 0 (- (length basename) 4))))

    (dolist (pm images)
      (setq ppm-filename (format nil "/tmp/~A-~A-~0,7d.ppm" basename random-str counter))
      (cond ((oddp (send pm :width))
             (user::write-image-file (format nil "/tmp/~A-~A.ppm" basename random-str) pm)
             (unix::system (format nil "convert ~A -resize ~Ax~A\! ~A"
                                   (format nil "/tmp/~A-~A.ppm" basename random-str)
                                   (* (round (/ (send pm :width) 2)) 2)
                                   (* (round (/ (send pm :height) 2)) 2)
                                   ppm-filename)))
            (t
             (user::write-image-file ppm-filename pm)))
      (format t ";; writing ~A of ~A images...~C" ppm-filename (length images) #x0d)
      (incf counter))
    (unix:system (format nil "rm -f ~A.mpg" basename))
    (setq cmd
          (format
           nil
           "ffmpeg -framerate ~A -i /tmp/~A-~A-%07d.ppm -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p ~A.mpg" (/ 25 delay) basename random-str basename))
    (if *debug* (warn cmd))
    (unix:system cmd)

    (format t ";; generate ~A.mpg from ~A images~%" basename (length images))
    (if delete (unix:system  (format nil "rm /tmp/~A-~A*.ppm" basename random-str)))
    ))


(defun make-animgif-from-images
  (giffile images
	   &key (delete t)
	   transparent
	   (loop t)
	   (delay 10)
           (background "000000"))
  (let* ((counter 0) (basename (pathname-name giffile))
	 gif-filename ppm-filename (gif-filenames " ")
	 cmd)
    ;; check animgif commands (convert, gifsicle)
    (when (/= (unix::system "which convert > /dev/null") 0)
      (warning-message 1 "(make-animgif-from-images) requires 'convert' command.~%")
      (return-from make-animgif-from-images nil))
    (when (/= (unix::system "which gifsicle > /dev/null") 0)
      (warning-message 1 "(make-animgif-from-images) requires 'gifsicle' command.~%")
      (return-from make-animgif-from-images nil))
    ;;
    (if (string= ".gif" (subseq basename (- (length basename) 4)))
	(setq basename (subseq basename 0 (- (length basename) 4))))
    (setq gif-filename (format nil "~A.gif" basename))
    (dolist (pm images)
      (setq ppm-filename (format nil "/tmp/~A~A.ppm" basename counter))
      (setq gif-filename (format nil "./tmp~A.gif" counter))
      (setq gif-filenames (concatenate string gif-filenames " " gif-filename))
      (user::write-image-file ppm-filename pm)
      (format t ";; writing ~A of ~A images...~C" ppm-filename (length images) #x0d)
      (finish-output t)
      (unix:system (format nil "convert ~A ~A"  ppm-filename gif-filename))
      (if delete (unix:system (format nil "rm ~A " ppm-filename)))
      (incf counter))
    (setq cmd
	  (format
	   nil
	   "gifsicle -O2 -D2 -w --delay=~A ~A ~A --colors 256 ~A > ~A.gif"
	   delay
	   (if transparent (format nil "-t#~A" background) "")
	   (if loop "--loop" "")
	   gif-filenames basename))
    (if *debug* (warn cmd))
    (unix:system cmd)

    (format t ";; generate ~A.gif from ~A images~%" basename (length images))
    (if delete (unix:system  (format nil "rm ~A " gif-filenames)))
    ))

(defmethod irtviewer
  (:logging
   (&optional (flag :on))
   "start/stop logging
    :clear    Clear log
    :start    Start logging
    :restart  Stop and restart logging
    :stop     Stop logging
   "
   (case
       flag
     ((:clear)
      (send self :clear-image-sequence))
     ((:start :on) ;; :on 'start is for backward compatibility
      (setq logging-flag t)
      (send self :push-image))
     ((:restart)
      (send self :logging :clear)
      (send self :logging :start))
     ((:stop :end :off)
      (send self :push-image)
      (setq logging-flag nil)))
   )
  (:clear-log () (send self :clear-image-sequence))
  ;; store images to image-sequence
  (:push-image
   ()
   (push (send viewer :viewsurface :getglimage
	       :width (- (send viewer :viewsurface :width) 1)
	       :height (send viewer :viewsurface :height))
	 image-sequence))
  (:clear-image-sequence ()(setq image-sequence nil))
  (:image-sequence () (reverse image-sequence))
  ;; save functions
  (:save-mpeg
   (&key (fname "anim.mpg")
	 (delay 5) (delete t))
   "':save-mpeg' saves logged images as mpeg.
     To start image log, run ':logging :start' and to stop log, run ':logging :stop'.
     Note that ':logging :stop' did not clear the logged sequence, so you need to run
     ':logging :clear'.
     :save-anmigif' did not stop nor clear the image sequence, so you have to run them manuualy"
   (let ((bg-color ((send self :viewer :viewsurface) . x::bg-color)))
     (unless (send self :image-sequence)
       (send self :push-image)) ;; make sure at least we have one images
     (make-mpeg-from-images
      fname (send self :image-sequence)
      :delay delay
      :delete delete)
     ))
  (:save-animgif
   (&key (fname "anim.gif")
	 (delay 5) (transparent t) (loop t)
	 (delete t))
   "':save-anmigif' saves logged images as animation gif.
     To start image log, run ':logging :start' and to stop log, run ':logging :stop'.
     Note that ':logging :stop' did not clear the logged sequence, so you need to run
     ':logging :clear'.
     :save-anmigif' did not stop nor clear the image sequence, so you have to run them manuualy"
   (let ((bg-color ((send self :viewer :viewsurface) . x::bg-color)))
     (unless (send self :image-sequence)
       (send self :push-image)) ;; make sure at least we have one images
     (make-animgif-from-images
      fname (send self :image-sequence)
      :delay delay
      :loop loop
      :delete delete
      :transparent transparent
      :background (cond ((float-vector-p bg-color)
                         (format nil "~0,2x~0,2x~0,2x"
                                 (round (* 255 (min 1 (elt bg-color 0))))
                                 (round (* 255 (min 1 (elt bg-color 1))))
                                 (round (* 255 (min 1 (elt bg-color 2))))))
                        (t "000000")))
     ))
  (:save-animgif
   (&key (fname "anim.gif")
	 (delay 5) (transparent t) (loop t)
	 (delete t))
   "':save-anmigif' saves logged images as animation gif.
     To start image log, run ':logging :start' and to stop log, run ':logging :stop'.
     Note that ':logging :stop' did not clear the logged sequence, so you need to run
     ':logging :clear'.
     :save-anmigif' did not stop nor clear the image sequence, so you have to run them manuualy"
   (let ((bg-color ((send self :viewer :viewsurface) . x::bg-color)))
     (unless (send self :image-sequence)
       (send self :push-image)) ;; make sure at least we have one images
     (make-animgif-from-images
      fname (send self :image-sequence)
      :delay delay
      :loop loop
      :delete delete
      :transparent transparent
      :background (cond ((float-vector-p bg-color)
                         (format nil "~0,2x~0,2x~0,2x"
                                 (round (* 255 (min 1 (elt bg-color 0))))
                                 (round (* 255 (min 1 (elt bg-color 1))))
                                 (round (* 255 (min 1 (elt bg-color 2))))))
                        (t "000000")))
     ))
  (:save-image
   (filename)
   "save curent view to image, supported formats are jpg/png/pnm"
   (user::write-image-file filename
                           (send viewer :viewsurface :getglimage
                                 :width (- (send viewer :viewsurface :width) 1)
                                 :height (send viewer :viewsurface :height))))
  )

;; 2022.04.30 move draw-things to irtgeo.l


(in-package "GEOMETRY")

;;
;; re-definition
;;
(defmethod viewing
  (:look
   (from &optional (to (float-vector 0 0 0)) (view-up (float-vector 0 0 1)))
   (let* ((view-direction (normalize-vector (v- to from)))
	  (v1 (v* view-direction view-up))
	  view-right)
     (if (< (norm v1) *epsilon*)
	 (setq view-right (normalize-vector
			   (v* view-direction (float-vector -1 0 0))))
       (setq view-right (normalize-vector v1)))
     (setq view-up (normalize-vector (v* view-right view-direction)))
     (setq view-direction (scale -1.0 view-direction))
     (setf (array-entity rot)
	   (concatenate float-vector view-right view-up view-direction))
     (transpose rot rot)
     (send self :locate from :world)
     (send self :worldcoords))
   )
  )

(in-package "USER")

;; 2022.04.30 move objects, make-irtviwer, irtviewer-dummy, make-irtviewer-dummy to irtgeo.l
;;
;;
(defun make-irtviewer (&rest args)
  "Create irtviewer
  :view-name title
  :hither    near cropping plane
  :yon       far cropping plane
  :width     width of the window
  :height    height of the window
  :draw-origin size of origin arrow, use nil to disable it
  :draw-floor  use t to view floor
  :floor-color floor color. default is #f(1 1 1), i.e. white."
  (cond
   ((or (null x::*display*) (= x::*display* 0))
    (warning-message 1 ";; no display is found. exit from (make-irtviewer)~%")
    (make-irtviewer-dummy))
   ((memq :no-window args)
    (make-irtviewer-no-window))
   (t
    (setq *irtviewer* (send-lexpr (instantiate x::irtviewer) :create args))))
   *irtviewer*)

(defclass irtviewer-no-window
  :super propertied-object
  :slots (irtviewer))
(defmethod irtviewer-no-window
  (:init (&rest args)
   (setq irtviewer (instantiate x::irtviewer))
   self)
  (:create (&rest args)
   (send* irtviewer :create :call-super nil :alternate t args))
  (:resize (newwidth newheight)
   ;; (setq x::width newwidth x::height newheight)
   ;; (send-super  :resize newwidth newheight) ; re-define resize, to skip resize xwindow
   (send self :viewer :viewsurface :resize newwidth newheight)
   (send self :viewer :viewsurface :makecurrent)
   (send self :viewer :viewsurface :3d-mode)
   (let ((wh-max (max newwidth newheight)))
     (send (send self :viewer :viewport)
           :resize :width wh-max :height (- wh-max)
           :xcenter (/ newwidth 2) :ycenter (/ newheight 2)))
   (send self :redraw))
  (:nomethod (&rest args)
   (send* irtviewer args))
  ) ;;

(defun geo::default-pixmapsurface (&rest args)
  (send-lexpr (instantiate gl::glpixmapsurface) :create args))
(defun make-irtviewer-no-window (&rest args)
  (setf (symbol-function 'geo::default-viewsurface-orig)
        (symbol-function 'geo::default-viewsurface))
  (setf (symbol-function 'geo::default-viewsurface)
        (symbol-function 'geo::default-pixmapsurface))
  (setq *irtviewer* (instance irtviewer-no-window :init))
  (send-lexpr *irtviewer* :create args)
  (setf (symbol-function 'geo::default-viewsurface)
        (symbol-function 'geo::default-viewsurface-orig))
  *irtviewer*)


(defmacro with-save-mpeg (fname &rest forms)
  `(let ()
     (send *irtviewer* :logging :clear)
     (send *irtviewer* :logging :start)
     ,@forms
     (send *irtviewer* :save-mpeg :fname ,fname)
     (send *irtviewer* :logging :stop)
     ))
(defmacro with-save-animgif (fname &rest forms)
  `(let ()
     (send *irtviewer* :logging :clear)
     (send *irtviewer* :logging :start)
     ,@forms
     (send *irtviewer* :save-animgif :fname ,fname)
     (send *irtviewer* :logging :stop)
     ))

(in-package "GL")

(provide :irtviewer "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.8  2010-02-04 01:23:30  k-okada
;;; :make-current before gl::draw-globjects in irtviewer
;;;
;;; Revision 1.7  2009/11/08 10:36:17  k-okada
;;; set :name using gensym, for (find-viewer) in send irtviewer :create
;;;
;;; Revision 1.6  2009/11/08 09:43:12  k-okada
;;; make-irtviewer :title works
;;;
;;; Revision 1.5  2009/07/31 11:10:31  ueda
;;; add :draw-objects to geo::viewer for (pickview :separate t)
;;;
;;; Revision 1.4  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.3  2009/01/06 13:18:23  k-okada
;;; check event-type and window-id for skipping x event, use :expose for each item
;;;
;;; Revision 1.2  2008/09/22 06:02:10  k-okada
;;; fix to work with jskeusgl
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
