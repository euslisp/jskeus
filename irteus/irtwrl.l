;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: $
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

(in-package "GEOMETRY")

(export '(eus2wrl))

(defun eus2wrl (fname obj &rest args)
  (let (set-name)
    (unless (send obj :name)
      (setq set-name t)
      (send obj :name "object"))
    (with-open-file (f fname :direction :output)
                    (apply #'geo::dump-object-to-wrl obj f args))
    (if set-name
        (send obj :name nil))))
;;
;; wrl
;;
(defun dump-object-to-wrl (obj strm &key ((:scale sc) 0.001))
  (let* ((pos (scale sc (send obj :pos)))
	 (rot (rotation-angle (send obj :rot)))
	 (children (send obj :descendants))
	 (color (get obj :face-color))
	 name material diffuse fs vs vlist v rot-v rot-a)
    (if (get obj :abbrev-name) (setq name (get obj :abbrev-name))
      (if (send obj :name)(setq name (send obj :name))
	(return-from dump-object-to-wrl nil)))
    (if (or (null rot) (memq *nan* (coerce (cadr rot) cons)))
	(setq rot-v (float-vector 0 0 1) rot-a 0)
      (setq rot-v (cadr rot) rot-a (car rot)))
    (format strm "DEF ~A Transform {~%" name)
    (format strm "  translation ~A ~A ~A~%"
	    (elt pos 0) (elt pos 1) (elt pos 2))
    (format strm "  rotation ~A ~A ~A ~A~%"
	    (elt rot-v 0) (elt rot-v 1) (elt rot-v 2) rot-a)
    (format strm "  children [~%")
    (when (or (derivedp obj body) (derivedp obj user::bodyset-link))
     (when (derivedp obj body)
       (setq fs (send (body-to-faces obj) :faces)
             vs (body-model-vertices obj)
             vlist vs))
     (when (derivedp obj user::bodyset-link)
       (setq fs (send (body-to-faces obj) :faces)
             vs (flatten (mapcar #'(lambda (o) (geo::faceset-model-vertices o)) (send obj :bodies)))
             vlist vs))
     (format strm "    Shape {~%")
     (format strm "      appearance Appearance {~%")
     (when (and color (find-package "GL"))
       (setq material (gl::find-color color)
	     diffuse (send material :diffuse))
       (format strm "        material Material {~%")
       (format strm "          diffuseColor ~A ~A ~A~%"
	       (elt diffuse 0) (elt diffuse 1) (elt diffuse 2))
       (format strm "        }~%")
       )
     (format strm "      }~%")
     (format strm "      geometry DEF ~A-FACES IndexedFaceSet {~%" name)
     (format strm "        ccw TRUE~%")
     (format strm "        solid TRUE~%")
     (format strm "        coord DEF ~A-COORD Coordinate {~%" name)
     (format strm "          point[~%")
     (while vlist
       (setq v (scale sc (pop vlist)))
       (if vlist
	   (format strm "            ~A ~A ~A,~%"
		   (elt v 0) (elt v 1) (elt v 2))
	 (format strm "            ~A ~A ~A~%"
		 (elt v 0) (elt v 1) (elt v 2))
	 )
       )
     (format strm "          ]~%")
     (format strm "        }~%")
     (format strm "        coordIndex [~%")
     (while fs
       (format strm "        ")
       (dolist
	   (p (cdr (send (pop fs) :vertices)))
	 (format strm "~A, "
		 (position (send obj :inverse-transform-vector p) vs
			   :test #'eps-v=))
	 )
       (format strm "-1,~%")
       )
     (format strm "        ]~%")
     (format strm "      }~%")
     (format strm "    }~%")
     )
    (while children
      (if (and (dump-object-to-wrl (pop children) strm :scale sc)
	       (< 1 (length children)))
	  (format strm "    ,~%"))
      )
    (format strm "  ]~%") ;; children[
    (format strm "}~%") ;; DEF
    t))


(provide :irtcad "$Id: ")
