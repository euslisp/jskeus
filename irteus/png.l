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

(in-package "IMAGE")

(export '(read-png-file write-png-file))

(defun read-png-file (fname)
  (if (probe-file fname)
      (let (ret width height byte-depth image img)
        (setq ret (png-read-image fname))
        (setq width (elt ret 0) height (elt ret 1)
              byte-depth (elt ret 2) image (elt ret 3))
        (case byte-depth
          (1
           (setq img (instance grayscale-image :init width height image)))
          (3
           (setq img (instance color-image24 :init width height image)))
          (t
           (error ";; read-png-file: unsupported image byte-depth ~A" byte-depth)))
        (send img :name fname)
        img)
    nil))

(defun write-png-file (fname img)
  (let (byte-depth)
    (cond
     ((derivedp img grayscale-image) (setq byte-depth 1))
     ((derivedp img color-image24) (setq byte-depth (send img :byte-depth)))
     (t (error ";; write-png-file: unsupported image type ~A" img)))
  (png-write-image fname (send img :width) (send img :height) byte-depth
                   (send img :entity))
  ))

(provide :png "@(#)$Id$")
