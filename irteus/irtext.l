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

(unless (find-package "COLLADA") (make-package "COLLADA")) ;; define collad package to import collada function
(defun load-irteus ()
  (in-package "USER")
  (load-library
   (format nil "~A~A/lib/libirteus"
	   *eusdir* (unix:getenv "ARCHDIR"))
   '("irtmath" "irtutil" "irtc" "irtgeoc" "irtgraph" "gnuplotlib" "___time" "___pgsql")))
(defun load-irteusg ()
  (in-package "GEOMETRY")
  (load-library
   (format nil "~A~A/lib/libirteusg"
	   *eusdir* (unix:getenv "ARCHDIR"))
   '("irtgeo" "euspqp" "pqp" "irtscene" "irtmodel" "irtdyna" "irtrobot" "irtsensor" "irtbvh" "irtcollada" "irtstl" "irtwrl" "irtdraw" "irtpointcloud" "eusbullet" "bullet" "irtcollision"))
  (in-package "USER")
  (import '(collada::convert-irtmodel-to-collada collada::eus2collada)))
(defun load-irteusx ()
  (in-package "X")
  (load-library
   (format nil "~A~A/lib/libirteusx"
	   *eusdir* (unix:getenv "ARCHDIR"))
   '("irtx"))
  (in-package "IMAGE")
  (load-library
   (format nil "~A~A/lib/libirteusimg"
	   *eusdir* (unix:getenv "ARCHDIR"))
   '("eusjpeg" "euspng" "png" "irtimage"))
  (in-package "USER")
  )
(defun load-irteusgl ()
  (in-package "GL")
  (load-library
   (format nil "~A~A/lib/libirteusgl"
	   *eusdir* (unix:getenv "ARCHDIR"))
   '("irtglrgb" "irtgl" "irtglc" "irtviewer"))
  )

(unless (member (pathname-name *program-name*)
		(list "eus0" "eus1" "eus2" "eus" "eusx" "eusg" "eusgl" "euscomp")
		:test #'string=)
  (when (substringp "eus" (pathname-name *program-name*))
    (load-irteus))
  (when (or (substringp "eusg" (pathname-name *program-name*))
	    (substringp "eusx" (pathname-name *program-name*)))
    (load-irteusg))
  (when (or (substringp "eusx" (pathname-name *program-name*))
	    (substringp "eusgl" (pathname-name *program-name*)))
    (load-irteusx))
  (when (substringp "eusgl" (pathname-name *program-name*))
    (load-irteusgl))
  (unless (member :irt *features*) (push :irt *features*))
  )

;; add extra *load-path* for jskeus

