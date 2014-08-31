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

(export '(read-image-file write-image-file))

(defun read-image-file (fname)
  "read image of given fname. It returns instance of {\\bf grayscale-image} or {\\bf color-image24}."
  (cond
   ((probe-file fname)
    (cond ((or (string= (pathname-type fname) "jpg")
               (string= (pathname-type fname) "jpeg"))
           (read-jpeg-file fname))
          ((string= (pathname-type fname) "png")
           (read-png-file fname))
          ((member (pathname-type fname) (list "pbm" "pgm" "ppm") :test #'string=)
           (read-pnm-file fname))
          (t
           (warn ";; Unknown file format ~A~%" fname))))
   (t (warn ";; Could not find file ~A~%" fname)
      (return-from read-image-file nil))))

(defun write-image-file (fname img)
  "write img to given fname"
  (cond
   ((or (string= (pathname-type fname) "jpg")
        (string= (pathname-type fname) "jpeg"))
    (write-jpeg-file fname img))
   ((string= (pathname-type fname) "png")
    (write-png-file fname img))
   (t
    (write-pnm-file fname img))))

(provide :irtimage "@(#)$Id$")
