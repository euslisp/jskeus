;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

(load "sample-arm-model.l")


(defun sample-collision-distance-body
    (&key
     (obj1 (make-cube 200 300 500))
     (obj2 (make-cube 400 600 1000))
     (obj1-coords-func
      #'(lambda (cnt) (make-coords :pos (float-vector 1000 0 0))))
     (obj2-coords-func
      #'(lambda (cnt)
          (make-coords
           :pos (float-vector (* 1500 (sin (/ cnt 100.0))) (* 500 (sin (+ (/ cnt 200.0) (deg2rad 45)))) 0)
           :rpy (list (* pi (sin (/ cnt 200.0))) (+ (* pi (sin (/ cnt 400.0))) pi/2) 0)
           )))
     )
  (let* ((cnt 0)
         (ret)
         )
    (when obj1-coords-func
      (send obj1 :newcoords (funcall obj1-coords-func (float cnt))))
    (when obj2-coords-func
      (send obj2 :newcoords (funcall obj2-coords-func (float cnt))))
    (send obj1 :set-color (float-vector 1 0 0) 0.5)
    (send obj2 :set-color (float-vector 0 1 0) 0.4)
    (objects (list obj1 obj2))

    (do-until-key
     ;; move object
     (incf cnt)
     (when obj1-coords-func
       (send obj1 :newcoords (funcall obj1-coords-func (float cnt))))
     (when obj2-coords-func
       (send obj2 :newcoords (funcall obj2-coords-func (float cnt))))
     (send *irtviewer* :draw-objects :flush nil)
     ;; get distance beween target-link and obj
     (setq ret (collision-distance obj1 obj2))
     ;; draw
     (send (elt ret 1) :draw-on :flush nil :width 16 :size 50 :color (float-vector 1 0.4 0.4))
     (send (elt ret 2) :draw-on :flush nil :width 16 :size 50 :color (float-vector 0.4 1 0.4))
     (send (make-line (elt ret 1) (elt ret 2)) :draw-on :flush nil
           :width 8 :color (if (> (elt ret 0) 0) (float-vector 0 1 1) (float-vector 1 1 0)))
     (send *irtviewer* :viewer :flush)
     (unix::usleep (* 20 1000))
     (x::window-main-one)
     )
    ))
(warn "(sample-collision-distance-body)~%")

(defun sample-collision-distance-sphere
    ()
  (sample-collision-distance-body
   :obj2 (make-sphere 600))
  )

(defun sample-collision-distance-cube
    ()
  (sample-collision-distance-body)
  )

(defun sample-collision-distance-cylinder
    ()
  (sample-collision-distance-body
   :obj2 (make-cylinder 400 1200)
   )
  )

(defun sample-collision-distance-conv
    ()
  (sample-collision-distance-body
   :obj2 (make-cone (float-vector 0 0 1500) (list (float-vector -800 -500 0) (float-vector 800 -500 0) (float-vector 0 500 0)))
   )
  )


(defun sample-collision-distance-link
    (&key
     (obj (make-cube 200 500 500))
     (obj-coords-func
      #'(lambda (cnt) (make-coords :pos (float-vector 500 0 250))))
     )
  (let* ((cnt 0)
         (ret)
         (robot (instance sarmclass :init))
         (target-link (elt (send robot :links) 4))
         (base-link (elt (send robot :links) 0))
         )
    (when obj-coords-func
      (send obj :newcoords (funcall obj-coords-func (float cnt))))
    (send obj :set-color (float-vector 1 0 0) 0.5)
    (objects (list robot obj))

    (do-until-key
     ;; move object and robot
     (incf cnt)
     (dolist (j (send robot :joint-list))
       (send j :joint-angle
             (+ (* 0.49 (- (send j :max-angle) (send j :min-angle)) (sin (/ cnt 100.0)))
                (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
       )
     (when obj-coords-func
       (send obj :newcoords (funcall obj-coords-func (float cnt))))
     (send obj :newcoords (funcall obj-coords-func (float cnt)))
     (send *irtviewer* :draw-objects :flush nil)
     ;; get distance beween target-link and obj
     (setq ret (collision-distance target-link obj))
     (send (elt ret 1) :draw-on :flush nil :width 16 :size 50 :color (float-vector 1 0.4 0.4))
     (send (elt ret 2) :draw-on :flush nil :width 16 :size 50 :color (float-vector 0.4 1 0.4))
     (send (make-line (elt ret 1) (elt ret 2)) :draw-on :flush nil
           :width 8 :color (if (> (elt ret 0) 0) (float-vector 0 1 1) (float-vector 1 1 0)))
     ;; get distance beween target-link and base-link
     (setq ret (collision-distance target-link base-link))
     ;; draw
     (send (elt ret 1) :draw-on :flush nil :width 16 :size 50 :color (float-vector 1 0.4 0.4))
     (send (elt ret 2) :draw-on :flush nil :width 16 :size 50 :color (float-vector 0.4 1 0.4))
     (send (make-line (elt ret 1) (elt ret 2)) :draw-on :flush nil
           :width 8 :color (if (> (elt ret 0) 0) (float-vector 0 1 1) (float-vector 1 1 0)))
     (send *irtviewer* :viewer :flush)
     (unix::usleep (* 20 1000))
     (x::window-main-one)
     )
    ))
(warn "(sample-collision-distance-link)~%")
