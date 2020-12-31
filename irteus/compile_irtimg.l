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


(unless (boundp '*objdir*)
  (setq *objdir*
        (format nil "~A/~A/obj/" *eusdir* (unix:getenv "ARCHDIR"))
  ))

(in-package "IMAGE")
(if (eq (unix:access (format nil "~A/lisp/image/jpeg/" *eusdir*) unix::O_RDWR) t)
    (comp:compile-file-if-src-newer (format nil "~A/lisp/image/jpeg/eusjpeg.l" *eusdir*) user::*objdir*)
  (progn
    (unix::mkdir (format nil "/tmp/~A" (unix::getpid)))
    (unix::system (format nil "cp ~A/lisp/image/jpeg/eusjpeg.l /tmp/~A/eusjpeg.l" user::*eusdir* (unix::getpid)))
    (unix::link (format nil "~A/lisp/image/jpeg/eusjpeg.l" *eusdir*) (format nil "/tmp/~A/eusjpeg.l" (unix::getpid)))
    (comp:compile-file-if-src-newer (format nil "/tmp/~A/eusjpeg.l" (unix::getpid)) user::*objdir*)
    (unix::unlink (format nil "/tmp/~A" (unix::getpid)))
    ))
(comp:compile-file-if-src-newer (format nil "png.l" *eusdir*) user::*objdir*)
(comp:compile-file-if-src-newer (format nil "irtimage.l" *eusdir*) user::*objdir*)

(exit 0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id$
;;;
;;; $Log$
;;;
