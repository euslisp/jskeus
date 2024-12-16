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

(defun write-image-file (fname img &optional background)
  "write img to given fname"
  (cond
   ((or (string= (pathname-type fname) "jpg")
        (string= (pathname-type fname) "jpeg"))
    (write-jpeg-file fname img))
   ((string= (pathname-type fname) "png")
    (write-png-file fname img background))
   (t
    (write-pnm-file fname img))))

(provide :irtimage "@(#)$Id$")
