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
;;; Like particle.l's *space-box* -- a transparent cube that particles
;;; bounce around inside forever -- but with prim-drop.l's five
;;; different primitive shapes (sphere/cube/box/cylinder/cone) instead
;;; of all-identical particles, so you can watch how each shape's own
;;; geometry (round vs cornered, tapered vs not) changes how it bounces
;;; and tumbles off a floor, ceiling and four walls, and off each other.
;;;
;;; Reuses drop-body, bounce-against-floor, apply-rolling-friction and
;;; bounce-between from prim-drop.l as-is; the floor (bottom face) still
;;; gets full rolling-friction treatment (see prim-drop.l's header), but
;;; the other five faces only get a one-shot translational bounce plus
;;; the same off-centre torque impulse a floor bounce gets -- no
;;; continuous rolling friction against a wall or the ceiling, which
;;; this demo does not need enough to be worth the extra complexity.
;;;
(load "prim-drop.l")

(defun bounce-against-wall (obj axis dir wall-coord normal &key (restitution 0.35))
  "Generalization of bounce-against-floor to any axis-aligned wall: AXIS
   is 0/1/2 (x/y/z), DIR is :min or :max (which side of that axis the
   wall sits on), WALL-COORD its position, NORMAL the unit vector
   pointing back into the box.  Tunnel-proof and torque-generating in
   exactly the same way as bounce-against-floor.  Restitution matches
   the floor's own default (0.35, previously this was a bouncier 0.4)
   on the assumption the whole box is one material; apply-surface-
   friction (called for all six faces, not just the floor) is what
   mainly keeps a body from bouncing around indefinitely now, though."
  (let* ((ep (send obj :extreme-point axis dir))
         (ev (car ep)) (contact-pt (cadr ep))
         (hit (if (eq dir :min) (<= ev wall-coord) (>= ev wall-coord))))
    (when hit
      (let ((v-before (copy-seq (send obj :velocity))))
        (send obj :bounce-off :normal normal :restitution restitution
              :push-out (abs (- wall-coord ev)))
        (when (send obj :rotatable)
          ;; see bounce-against-floor in prim-drop.l: the velocity change
          ;; scaled back up by mass recovers a momentum-like impulse
          (let* ((dv (v- (send obj :velocity) v-before))
                 (r (v- contact-pt (send obj :center))))
            (send obj :apply-angular-impulse (scale (send obj :mass) (v* r dv)))))))))

(defun drop-primitives-in-box ()
  "Sphere/cube/box/cylinder/cone are scattered inside a transparent cube
   with random initial velocities, and bounce off its floor, ceiling,
   and four walls, and off each other, under gravity, until they settle
   -- then they are scattered again.  Any key stops the demo."
  (send *irtviewer* :title "prim-box")
  (let* ((bodies (make-drop-bodies))
         (n (length bodies))
         (half 250.0)
         (box (make-cube (* 2 half) (* 2 half) (* 2 half)))
         (margin 3.0)
         (max-ccd-iters 12)  ;; see prim-drop.l's pair-toi-bound/min-pair-toi/
                             ;; ccd-advance-frame: a cap on how many times one
                             ;; frame's position advance can be subdivided
                             ;; before the final catch-up step just consumes
                             ;; whatever is left in one go -- kept low since
                             ;; that catch-up guarantees full-frame completion
                             ;; regardless
         (settle-count 0))
    (send box :set-color (float-vector 0.6 0.85 1.0))
    (gl::transparent box 0.15)
    (objects (append bodies (list box)))
    (labels
        ((scatter ()
           (dotimes (i n)
             (let ((b (elt bodies i)))
               (send b :newcoords (make-coords))
               (send b :locate (float-vector (- (random (* 1.2 half)) (* 0.6 half))
                                              (- (random (* 1.2 half)) (* 0.6 half))
                                              (+ (random half) 20.0))
                     :world)
               ;; a slight tilt for every shape -- a real object rarely
               ;; leaves your hand perfectly level -- plus, for whichever
               ;; shapes can actually roll, a full random tumble too
               (send b :rotate (deg2rad (+ 3 (random 8.0))) (random-horizontal-axis) :world)
               (when (send b :rotatable)
                 (send b :rotate (deg2rad (random 360.0)) (random-horizontal-axis) :world))
               (send b :velocity (float-vector (- (random 6.0) 3.0)
                                                (- (random 6.0) 3.0)
                                                (- (random 3.0) 1.0)))
               (send b :angular-velocity
                     (if (send b :rotatable)
                         (float-vector (- (random 0.2) 0.1) (- (random 0.2) 0.1) (- (random 0.2) 0.1))
                       (float-vector 0 0 0)))))
           (setq settle-count 0)
           (warn "~%;; scattered ~a shapes inside the box~%" n))
         (all-settled-p ()
           (every #'(lambda (b) (< (abs (elt (send b :velocity) 2)) 0.3)) bodies)))
      (scatter)
      (do-until-key
       (dolist (b bodies) (send b :integrate-velocity :dt 1.0))  ;; gravity, once per frame
       (ccd-advance-frame
        bodies margin max-ccd-iters
        #'(lambda ()
            (dolist (b bodies)
              (bounce-against-floor b (- half))
              (bounce-against-wall b 2 :max half (float-vector 0 0 -1))
              (bounce-against-wall b 0 :min (- half) (float-vector 1 0 0))
              (bounce-against-wall b 0 :max half (float-vector -1 0 0))
              (bounce-against-wall b 1 :min (- half) (float-vector 0 1 0))
              (bounce-against-wall b 1 :max half (float-vector 0 -1 0))
              ;; every one of the box's six faces gets the same continuous
              ;; friction/damping treatment the floor alone used to (see
              ;; apply-surface-friction in prim-drop.l) -- a body mostly
              ;; bouncing between walls, never touching the floor, used to
              ;; have nothing but each bounce's restitution loss to ever
              ;; slow it down, which in a box this size could take a long time
              (apply-surface-friction b 2 :min (- half))
              (apply-surface-friction b 2 :max half)
              (apply-surface-friction b 0 :min (- half))
              (apply-surface-friction b 0 :max half)
              (apply-surface-friction b 1 :min (- half))
              (apply-surface-friction b 1 :max half))
            (dotimes (i n)
              (do ((j (1+ i) (1+ j))) ((>= j n))
                (bounce-between (elt bodies i) (elt bodies j) margin)))))
       (if (all-settled-p)
           (progn
             (incf settle-count)
             (when (>= settle-count 150) (scatter)))
         (setq settle-count 0))
       (send *irtviewer* :draw-objects)
       (unix::usleep (* 15 1000))
       (x::window-main-one))
      )))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(drop-primitives-in-box) ;; sphere/cube/box/cylinder/cone bounce around inside a box~%")
