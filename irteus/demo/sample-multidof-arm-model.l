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

;;;
;;; add sample arm robot which we can configure links and joints parameters
;;;
(defclass sample-multidof-arm-robot
  :super robot-model
  :slots ()
  )

(defmethod sample-multidof-arm-robot
  (:init
   (&key (axis-order (list :z :y :x))
         (joint-class rotational-joint)
         (arm-length 50.0)
         (joint-class-list (make-list (length axis-order) :initial-element joint-class))
         (arm-length-list (make-list (1+ (length axis-order)) :initial-element arm-length)))
   (send-super :init)
   (setq links
         (mapcar
          #'(lambda (al)
              (let ((b (make-cube 10 10 al)))
                (send b :set-color #F(0 1 0))
                (send b :translate (float-vector 0 0 (* 0.5 al)))
                (let ((bb
                       (instance bodyset-link :init
                                 (make-cascoords)
                                 :weight 1000
                                 :centroid (float-vector 0 0 (* 0.5 al))
                                 :inertia-tensor (scale-matrix 1000 (unit-matrix 3))
                                 :bodies (list b))))
                  bb)))
          arm-length-list))
   (let ((tpos 0)
         rarm-end-coords)
     (dotimes (i (length links))
       (send (elt links i) :locate (float-vector 0 0 tpos) :world)
       (setq tpos (+ tpos (elt arm-length-list i)))))
   (send self :assoc (car links))
   (setq joint-list
         (mapcar
          #'(lambda
              (pl cl ax jc)
              (send pl :assoc cl)
              (instance jc :init
                        :parent-link pl
                        :child-link cl
                        :max-joint-torque 10
                        :axis ax))
          (butlast links) (cdr links) axis-order joint-class-list))
   (mapcar #'(lambda (x) (send x :name (format nil "link~A" (position x links)))) links)
   (mapcar #'(lambda (x) (send x :name (format nil "joint~A" (position x joint-list)))) joint-list)
   (dolist (j (send self :joint-list))
     (eval `(defmethod ,(send (class self) :name)
              (,(read-from-string (format nil ":~A" (send j :name)))
               () (elt joint-list ,(position j joint-list))))))
   (let ((rarm-end-coords (make-cascoords :pos (float-vector 0 0 (reduce #'+ arm-length-list)))))
     (send (car (last links)) :assoc rarm-end-coords)
     (send self :add-limb :rarm :links (cdr links) :end-coords rarm-end-coords :root-link (cadr links)))
   (send-super :init-ending)
   self)
  )
