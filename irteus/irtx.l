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

(in-package "X")

(setq *skip-motion-event* (list :motionNotify :configureNotify :expose))

(defun window-main-one (&optional fd)
  (when (or (null *display*) (= *display* 0))
    (warning-message 3 ";; (window-main-one ~A) was called, but no display is found.~%" fd)
    (return-from window-main-one nil))
  (catch :window-main-loop
      (while (> (EventsQueued *display* 1) 0)
	   (NextEvent *display* event)
	   (if *xevent-debug* (print-event event)) 
	   (when (and *skip-motion-event*
		      (memq (event-type event) *skip-motion-event*))
		(while
		    (and (> (Pending *display*) 0)
			 (progn (PeekEvent *display* event2)
				(and
				 (memq (event-type event2) *skip-motion-event*)
				 (= (xevent-type event) (xevent-type event2))
				 (= (xevent-window event) (xevent-window event2)))))
		    ;ignore the previous event
		    (nextEvent *display* event)
		    (incf *skipped-event-count*)) )
	   (process-event event))
	(xflush))
   )

(when (and lisp::*use-top-selector* *display* (not (eq *display* 0)))
  (send *top-selector* :remove-port (display-fd))
  (send *top-selector* :add-port (display-fd) #'window-main-one))

(let (x-lib pname)
#+:darwin
#+:darwin
 (if (probe-file "/opt/local/lib/libX11.dylib")
     (setq pname "/opt/local/lib/libX11.dylib")
   (setq pname "/opt/X11/lib/libX11.dylib"))
#+:cygwin
  (if (probe-file "/usr/bin/cygX11-6.dll")
      (setq pname "/usr/bin/cygX11-6.dll")
    (setq pname "/usr/X11R6/bin/cygX11-6.dll"))

  ;; for receiving Window Manager Messages
  (if (probe-file pname)
      (setq x-lib (load-foreign pname))
    (setq x-lib (sys::sysmod)))
  (defforeign SetWMProtocols x-lib "XSetWMProtocols" () :integer))

(unless (assoc :create-org (send Xwindow :methods))
  (rplaca (assoc :create (send Xwindow :methods)) :create-org))
(unless (assoc :event-notify-org (send Xwindow :methods))
  (rplaca (assoc :event-notify (send Xwindow :methods))
 :event-notify-org))

;; wheel mouse
(defun event-far (e)  (logtest (XEvent-state e) #x800))
(defun event-near (e)  (logtest (XEvent-state e) #x1000))

(defmethod Xwindow
 (:create
  (&rest args)
  (let (wmname xatom)
    (send* self :create-org args)
    (setq wmname "WM_DELETE_WINDOW")
    (setq xatom (integer-vector (InternAtom *display* (+ lisp::sizeof-header-offset
                                                         (sys:address wmname)) 1)))
    (SetWMProtocols *display* drawable (+ lisp::sizeof-header-offset
                                          (sys:address xatom)) 1)
    self))
  (:event-notify
   (type event)
   (if (member type '(:ClientMessage))
       (send self :event-notify-dispatch type event)
     (send self :event-notify-org type event)))
  (:quit (&rest a)
    (send self :destroy)
    (sync *display* 1)
    :destroyed)
  (:ClientMessage  (event) (send self :quit))
  ;;
  (:set-event-proc
   (type method receiver)
   (setf (get self :event-procs)
	 (cons (list type method receiver) (get self :event-procs)))
   )
  ;; override this method
  (:buttonRelease
   (event)
   (let ((event-procs (get self :event-procs))
	 (proc))
     (cond
      ((event-left event)
       (setq proc (assoc :buttonrelease-left event-procs)))
       ((event-right event)
	(setq proc (assoc :buttonrelease-right event-procs)))
       ((event-middle event)
	(setq proc (assoc :buttonrelease-middle event-procs)))
       ((event-far event)
	(setq proc (assoc :buttonrelease-far event-procs)))
       ((event-near event)
	(setq proc (assoc :buttonrelease-near event-procs)))
       )
     (when proc (send (third proc) (second proc) event))
     ))
  (:motionnotify
   (event)
   (let ((event-procs (get self :event-procs))
	 (proc))
     (cond
      ((event-left event)
       (setq proc (assoc :motionnotify-left event-procs)))
      ((event-right event)
       (setq proc (assoc :motionnotify-right event-procs)))
      ((event-middle event)
       (setq proc (assoc :motionnotify-middle event-procs)))
      )
     (when proc (send (third proc) (second proc) event))
     ))
  (:ButtonPress
   (event)
   (let ((proc (assoc :buttonpress (get self :event-procs))))
     (when proc (send (third proc) (second proc) event))
     ))
  )
  
(defmethod panel
  (:quit (&rest args) (send-super* :quit args)))

(defmethod Xscroll-bar
  (:redraw () (send self :draw-pattern)))

(defclass tabbed-panel
  :super panel
  :slots (tabbed-buttons tabbed-panels selected-tabbed-panel))
(defmethod tabbed-panel
  (:create
   (&rest args)
   (send-super* :create args)
   self)
  (:add-tabbed-panel
   (name)
   (let (p)
     (setq p (instance panel-tab-button-item :create
	       name self :change-tabbed-panel
	       :map nil
	       :parent self))
     (send self :locate-item p)
     (send p :state :up)
     (send-all tabbed-buttons :state :down)
     (push p tabbed-buttons)
     (push p items)
     (setq p (instance panel :create
		       :x 1
		       :y 24
		       :width (- (send self :width) 4)
		       :height (- (send self :height) 27)
		       :name name
		       :parent self))
     (push p tabbed-panels)
     (push p items)
     p))
  (:change-tabbed-panel
   (obj)
   (let (p)
     (setq selected-tabbed-panel
       (send self :tabbed-panel (if (stringp obj) obj (send obj :name))))
     (when selected-tabbed-panel
       (send-all tabbed-panels :unmap)
       (send selected-tabbed-panel :map)
       )
     (send-all tabbed-buttons :draw-label :down)
     (setq p (send self :tabbed-button (if (stringp obj) obj (send obj :name))))
     (if p (send p :draw-label :up))
     p))
  (:tabbed-button
   (name &rest args)
   (let (p)
     (setq p (car (member name tabbed-buttons
			  :key #'(lambda (o) (send o :name)) :test #'equal)))
     (if (and p args) 
	 (send* p args)
       p)
     ))
  (:tabbed-panel
   (name &rest args)
   (let (p)
     (setq p (car (member name tabbed-panels 
			  :key #'(lambda (o) (send o :name)) :test #'equal)))
     (if (and p args) 
	 (send* p args)
       p)
     ))
  (:resize
   (w h)
   (send-super :resize w h)
   (dolist (p tabbed-panels)
     (send p :resize (- w 4) (- h 34)))
   (send-all items :expose nil))
  )

(defclass panel-tab-button-item
  :super button-item
  :slots ())
(defmethod panel-tab-button-item
 (:draw-label (&optional (state :up) (offset 0))
      (let ((org-pixel (send gcon :foreground)))
	(setf 3D-state state)
	(if (eq state :up)
	    (send gcon :foreground *fg-pixel*)
	  (send gcon :foreground dark-edge-color))
	(send self :draw-polygon
	      (list (float-vector 0 height) (float-vector 0 0) 
		    (float-vector (1- width) 0) (float-vector (1- width) height)
		    ))
	(send self :string 
	      (+ ;4
	       (/ width 2)
	       (- (/ (aref labeldots 2) 2))
	       offset)			;x
	      (+ ;; (aref labeldots 0) 2 
	       (/ height 2) (aref labeldots 0)
	       (- (/ (+ (aref labeldots 0) (aref labeldots 1)) 2))
	       offset)	;y
	      label)
	(send gcon :foreground org-pixel)
	(send self :flush)
	))
 )

(provide :irtx "$Id$")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;; Revision 1.5  2009-08-03 06:26:32  eus
;;; add ia32 mode for x86_64 linux
;;;
;;; Revision 1.4  2009/03/12 13:43:02  k-okada
;;; fix for new cygwin/X
;;;
;;; Revision 1.3  2009/02/17 02:04:48  k-okada
;;; fix typo on copyright
;;;
;;; Revision 1.2  2009/01/06 13:18:23  k-okada
;;; check event-type and window-id for skipping x event, use :expose for each item
;;;
;;; Revision 1.1  2008/09/18 18:11:01  k-okada
;;; add irteus
;;;
;;;
