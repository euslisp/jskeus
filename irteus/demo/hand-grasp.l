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
;;; Lightweight rigid-body catch/release demo, now with a whole sequence
;;; of shapes instead of just a ball.
;;;
;;; Reuses prim-drop.l's drop-body class and its five primitive shapes
;;; (sphere/cube/box/cylinder/cone) instead of a bespoke ball class, so
;;; the same falling object (gravity, explicit Euler integration, no
;;; FEM/soft-body) is used here too -- but deliberately WITHOUT
;;; prim-drop.l's torque/rolling-friction physics (see
;;; simple-floor-bounce below): a shape that ever grazes a non-thumb
;;; part of the hand and picks up a little sideways speed must settle
;;; near where it lands, not roll off out of the hand's reach, which is
;;; exactly what happened the first time this was tried with the real
;;; rolling physics turned on.  Only one of the five is ever
;;; falling/held at a time; the other four sit parked off to the side
;;; until it is their turn.
;;;
;;; While the current object falls, the thumb alone chases a point at
;;; its actual current lowest vertex (drop-body's :lowest-point --
;;; generic across every shape, unlike a fixed "under the ball centre"
;;; offset would be); the object is not "caught" by any administrative
;;; height/position check -- it keeps falling and bouncing off the hand
;;; (collision-distance's nearest-point pair gives the contact normal)
;;; until the thumb itself actually touches it.  Only that real
;;; collision stops the fall and starts the index/middle fingers closing
;;; in, each driven by the same :inverse-kinematics-loop hand-grasp-ik.l
;;; uses -- aimed at the object's true centre (drop-body's :center,
;;; which corrects for a cylinder/cone's origin sitting at one end
;;; rather than its middle) -- and frozen the instant it too touches the
;;; object.  Once all three fingers have locked on, the object is pinned
;;; kinematically; opening the fingers releases it, it falls and bounces
;;; again, settles on the floor, and then the next shape in the
;;; sequence gets its turn.
;;;
(load "sample-hand-model.l")
(load "prim-drop.l")

(defun link-descendants (l &optional (r (list l)) rr)
  (let ((cl (send l :child-links)))
    (unless cl
      (push r rr)
      (return-from link-descendants rr))
    (dolist (ll cl)
      (setq r (append r (list ll)))
      (setq rr (link-descendants ll r rr)))
    rr))

(defun hand-catch-ball ()
  "sample-hand catches whichever of the five prim-drop.l shapes is
   currently falling toward it; the thumb tracks the shape's own
   underside and its actual collision with it is what stops the fall,
   then the other fingers close in and lock on contact.  Opening the
   fingers drops it, it falls (and bounces) and settles, and then the
   next shape in the sequence is dropped.  Any key stops the demo."
  (send *irtviewer* :title "hand-catch-ball")
  (let* ((h0 (instance sample-hand :init :name "catch-hand"))
         (flr (make-cube 1000 1000 10))       ;; generous so post-bounce drift still lands on it
         (hand-height 180.0)                  ;; lift the hand well above the floor
         (floor-top-z 0.0)                    ;; flr is located so its top face is at world z=0
         (objs (make-drop-bodies))            ;; sphere/cube/box/cylinder/cone, from prim-drop.l
         (n (length objs))
         (obj-index 0)
         (parked-spot (float-vector 2000 2000 0))
         (av-open   #f(0 0 0 0 -90 0 0))
         (drop-pos (float-vector 60 0 (+ hand-height 250)))
         (contact-margin 5.0)
         (close-steps 60) (hold-steps 120) (open-steps 40)
         ;; index ranges into (send h0 :joint-list) = (j00 j01 j10 j11 j20 j21 j22)
         (finger-joint-ranges (list (cons 0 1) (cons 2 3) (cons 4 6)))
         (finger-links
          (mapcar #'(lambda (r)
                      (flatten (link-descendants
                                (send (elt (send h0 :joint-list) (car r)) :child-link))))
                  finger-joint-ranges))
         (hand-links (send h0 :links))
         ;; fingertip coords / link-lists, ordered to match finger-joint-ranges
         ;; (koyubi middle-end-coords, hitosashi index-end-coords, oyayubi thumb-end-coords)
         (move-targets (mapcar #'(lambda (x) (send h0 x))
                                '(:middle-end-coords :index-end-coords :thumb-end-coords)))
         (link-lists (mapcar #'(lambda (mt) (send h0 :link-list (send mt :parent))) move-targets))
         locked phase step-in-phase av-at-grasp)
    (send h0 :locate (float-vector 0 0 hand-height) :world)
    (send flr :locate (float-vector 60 0 -5) :world)
    (send flr :set-color (float-vector 0.6 0.6 0.6))
    (objects (append (list h0 flr) objs))
    (labels
        ((current-obj () (elt objs obj-index))
         (links-min-distance (obj links)
           (let (best d)
             (dolist (l links)
               (setq d (car (collision-distance obj l)))
               (when (or (null best) (< d best)) (setq best d)))
             best))
         (nearest-contact (obj links)
           "Return (list min-distance point-on-obj point-on-link) over LINKS."
           (let (best-d best-p1 best-p2)
             (dolist (l links)
               (let* ((r (collision-distance obj l)) (d (car r)))
                 (when (or (null best-d) (< d best-d))
                   (setq best-d d best-p1 (elt r 1) best-p2 (elt r 2)))))
             (list best-d best-p1 best-p2)))
         (bounce-against-hand ()
           (let* ((obj (current-obj))
                  (info (nearest-contact obj hand-links))
                  (d (car info)))
             (when (<= d contact-margin)
               (let* ((diff (v- (elt info 1) (elt info 2)))
                      (n (if (> (norm diff) 1e-6) (normalize-vector diff) (float-vector 0 0 1))))
                 (send obj :bounce-off :normal n :restitution 0.25
                       :push-out (max 0.0 (- contact-margin d))))
               t)))
         (simple-floor-bounce (obj)
           "Tunnel-proof floor contact (drop-body's :lowest-point, same
            as prim-drop.l's bounce-against-floor), but deliberately
            WITHOUT its torque impulse or rolling friction: if the
            object ever grazes a non-thumb part of the hand and picks
            up a little sideways speed before it reaches the floor, we
            want it to just settle near where it lands, not develop
            real rolling motion and roll away out of the hand's reach."
           (let* ((lp (send obj :lowest-point)) (lz (car lp)))
             (when (<= lz floor-top-z)
               (send obj :bounce-off :normal (float-vector 0 0 1) :restitution 0.3
                     :push-out (- floor-top-z lz)))))
         (ik-step (idxs target)
           "Drive the fingertip(s) in IDXS toward TARGET one IK step."
           (when idxs
             (let* ((mts (mapcar #'(lambda (g) (elt move-targets g)) idxs))
                    (lls (mapcar #'(lambda (g) (elt link-lists g)) idxs))
                    (tcs (make-list (length mts) :initial-element target))
                    (dif-pos (mapcar #'(lambda (x) (send x :difference-position target :translation-axis t)) mts))
                    (dif-rot (mapcar #'(lambda (x) (send x :difference-rotation target :rotation-axis nil)) mts)))
               (send h0 :inverse-kinematics-loop dif-pos dif-rot
                     :move-target mts :link-list lls :target-coords tcs
                     :collision-avoidance-link-pair nil
                     :rotation-axis (make-list (length mts) :initial-element nil)
                     :translation-axis (make-list (length mts) :initial-element t)
                     :thre (make-list (length mts) :initial-element 1)
                     :rthre (make-list (length mts) :initial-element 0.1)))))
         (reset-scene ()
           (send h0 :angle-vector av-open)
           (dotimes (k n)
             (unless (= k obj-index)
               (send (elt objs k) :newcoords (make-coords))
               (send (elt objs k) :velocity (float-vector 0 0 0))
               (send (elt objs k) :angular-velocity (float-vector 0 0 0))
               (send (elt objs k) :locate (v+ parked-spot (float-vector 0 (* 250 k) 0)) :world)))
           (let ((obj (current-obj)))
             (send obj :newcoords (make-coords))
             (send obj :locate drop-pos :world)
             (send obj :velocity (float-vector 0 0 0))
             (send obj :angular-velocity (float-vector 0 0 0))
             (send obj :rotate (deg2rad (+ 3 (random 8.0))) (random-horizontal-axis) :world))
           (setq locked (list nil nil nil))
           (setq av-at-grasp nil)
           (setq phase :falling step-in-phase 0)
           (warn "~%;; dropping ~a (~a/~a)~%" (car (elt *drop-shape-specs* obj-index)) (1+ obj-index) n)))
      (reset-scene)
      (do-until-key
       (let ((obj (current-obj)) (caught (eq phase :holding)))
         (unless caught
           ;; several smaller physics steps per displayed frame, not one
           ;; full-sized one -- see prim-drop.l's bounce-between
           ;; docstring: a fast-falling object can otherwise cross a
           ;; thin finger's whole width between one frame and the next
           ;; and collision-distance's margin check never catches it
           (let ((dt (/ 1.0 4)))
             (dotimes (sub 4)
               (send obj :step :dt dt)
               (bounce-against-hand)
               (simple-floor-bounce obj))))
         (case phase
           (:falling
            ;; thumb (group 2) chases the still-falling object's own
            ;; current underside; it is not "caught" by anything until
            ;; the thumb itself actually touches it -- that collision is
            ;; what stops it.
            ;; aim the thumb directly under the object's true centre in
            ;; x/y, at the height of its current lowest point in z --
            ;; using the lowest VERTEX's x/y too (as a first cut) was
            ;; less accurate: a faceted mesh's lowest vertex is rarely
            ;; exactly under the centre, and that small persistent
            ;; horizontal offset was enough to make the thumb miss the
            ;; still-falling object noticeably more often
            (let ((c (send obj :center)) (lp (send obj :lowest-point)))
              (ik-step (list 2) (make-coords :pos (float-vector (elt c 0) (elt c 1) (car lp)))))
            (if (<= (links-min-distance obj (elt finger-links 2)) contact-margin)
                (progn
                  (setf (elt locked 2) t)
                  (setq phase :closing step-in-phase 0)
                  (warn ";; thumb under the object -- closing other fingers~%"))
              ;; the thumb only reacts to where the object currently is,
              ;; not where it will be, so a fast-accelerating fall can
              ;; graze past it -- and once clear of the hand, a bounce
              ;; off some other part can send it sliding away in a
              ;; straight line forever (no floor friction here) with
              ;; nothing to bring it back.  Rather than let a missed
              ;; catch hang the demo indefinitely, notice it has drifted
              ;; well clear of the hand and simply try that same shape
              ;; again.
              (let* ((p (send obj :worldpos))
                     (dx (- (elt p 0) 60)) (dy (elt p 1)))
                (when (> (+ (* dx dx) (* dy dy)) (* 350.0 350.0))
                  (warn ";; missed -- dropping ~a again~%" (car (elt *drop-shape-specs* obj-index)))
                  (reset-scene)))))
           (:closing
            (incf step-in-phase)
            (let* ((target (make-coords :pos (send obj :center)))
                   (idxs (remove-if #'(lambda (g) (elt locked g)) (list 0 1 2))))
              (ik-step idxs target)
              (dotimes (g 3)
                (unless (elt locked g)
                  (when (<= (links-min-distance obj (elt finger-links g)) contact-margin)
                    (setf (elt locked g) t)
                    (warn ";; finger ~a reached the object~%" g))))
              (when (or (and (elt locked 0) (elt locked 1) (elt locked 2))
                        (>= step-in-phase close-steps))
                (setq av-at-grasp (send h0 :angle-vector))
                (setq phase :holding step-in-phase 0)
                (warn ";; grasp complete -- holding~%"))))
           (:holding
            (incf step-in-phase)
            (when (>= step-in-phase hold-steps)
              (setq phase :opening step-in-phase 0)
              (warn ";; opening fingers~%")))
           (:opening
            (incf step-in-phase)
            (send h0 :angle-vector
                  (v+ av-at-grasp (scale (min 1.0 (/ (float step-in-phase) open-steps)) (v- av-open av-at-grasp))))
            ;; release only once the fingers are (almost) fully open, so
            ;; the object does not keep getting jostled by fingers still
            ;; nearby
            (when (>= step-in-phase open-steps)
              (setq phase :falling-2)
              (warn ";; released -- falling again~%")))
           (:falling-2
            ;; only vertical speed matters -- simple-floor-bounce has no
            ;; friction at all, so any horizontal drift from an earlier
            ;; hand bounce never decays and must not block noticing it
            ;; has landed
            (when (and (< (abs (elt (send obj :velocity) 2)) 0.3)
                       (<= (car (send obj :lowest-point)) (+ floor-top-z 0.5)))
              (send obj :velocity (float-vector 0 0 0))
              (send obj :angular-velocity (float-vector 0 0 0))
              (setq phase :resting step-in-phase 0)))
           (:resting
            (incf step-in-phase)
            (when (>= step-in-phase 100)
              (setq obj-index (mod (1+ obj-index) n))
              (reset-scene)))))
       (send *irtviewer* :draw-objects)
       (unix::usleep (* 15 1000))
       (x::window-main-one))
      )))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(hand-catch-ball) ;; hand catches each of sphere/cube/box/cylinder/cone in turn~%")
