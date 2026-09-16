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
;;; Lightweight rigid-body multi-shape drop demo, with rolling.
;;;
;;; A sphere, cube, box, cylinder and cone (all built from the same
;;; primitive constructors as sample-collision.l: make-sphere,
;;; make-cube, make-cylinder, make-cone) alternate between two stages:
;;;
;;;  :group -- all five are released together above the floor, each
;;;            with a small initial velocity aimed toward the centre so
;;;            they are bound to meet each other, not just the floor.
;;;  :solo  -- the same five bodies are then dropped one at a time from
;;;            straight above the centre, at the same height, so each
;;;            shape's own falling/landing/rolling behaviour can be
;;;            watched in isolation.
;;;
;;; Every body is a drop-body: gravity, explicit Euler integration, no
;;; FEM/soft-body, but it also carries a MASS and a diagonal MOMENT OF
;;; INERTIA TENSOR, both derived from an actual per-shape DENSITY (a
;;; genuine material property -- see *drop-shape-specs*) and the
;;; shape's true volume and standard closed-form solid-body inertia
;;; formula (sphere/box/cylinder/cone each have a different one; a box's
;;; three principal moments are not even equal to each other, so this is
;;; a real 3-component tensor in the body's own local/principal frame,
;;; not the single isotropic scalar an earlier version of this file
;;; used).  A torque impulse is transformed into that local frame,
;;; divided component-wise by the local principal moments, then
;;; transformed back to world frame to update angular velocity (see
;;; :apply-angular-impulse) -- the standard technique for simulating a
;;; rigid body whose inertia tensor is diagonal only in its own frame.
;;; Floor contact is found from the shape's own *current* lowest vertex
;;; (recomputed every step, since it rotates), which is what keeps floor
;;; contact both tunnel-proof and correct once a body has tipped over.
;;; A bounce applies a torque impulse from the offset between that
;;; contact point and the body's centre, and while resting a simple
;;; rolling-friction impulse damps the contact point's own horizontal
;;; velocity each frame -- converting spin into translation (or the
;;; reverse) until the body is genuinely rolling rather than just
;;; spinning in place or sliding.  Body-to-body contact stays purely
;;; translational (collision-distance's nearest-point pair gives the
;;; bounce normal, as before); adding rotational coupling there too was
;;; judged not worth the extra complexity for what this demo is for --
;;; but it IS mass-aware now: a real reduced-mass impulse (see
;;; bounce-between) means a heavy object barely moves when a light one
;;; hits it, instead of the two trading velocity as if equal-mass.
;;;
(defun weighted-extreme-point (vs axis dir)
  "Weighted-average extreme point of VS along AXIS (0/1/2), DIR :min or
   :max: each vertex is weighted 1/(1+d), d = its distance above (or
   below, for :max) the true extreme value.  Returns (list extreme
   weighted-point).  See drop-body's :lowest-point for why this
   continuous weighting replaced a hard within-1mm threshold."
  (let* ((sign (if (eq dir :min) 1.0 -1.0))
         (vals (mapcar #'(lambda (v) (* sign (elt v axis))) vs))
         (best (reduce #'min vals))
         (weights (mapcar #'(lambda (val) (/ 1.0 (+ 1.0 (- val best)))) vals))
         (wsum (reduce #'+ weights))
         (wpt (float-vector 0 0 0)))
    (mapc #'(lambda (v w) (setq wpt (v+ wpt (scale w v)))) vs weights)
    (list (* sign best) (scale (/ 1.0 wsum) wpt))))

;; --- solid-body mass/inertia formulas, closed-form per shape type -----
;; Each returns (list mass local-inertia) where local-inertia is a
;; float-vector of the three principal moments (Ixx Iyy Izz) about the
;; centre of mass, in the shape's OWN local/principal axes -- which for
;; make-cube/make-cylinder/make-cone/make-sphere already coincide with
;; the local x/y/z axes the shapes are built along (checked empirically
;; for the cylinder/cone: their length runs along local z).

(defun sphere-mass-inertia (radius density)
  (let* ((volume (* (/ 4.0 3.0) pi radius radius radius))
         (mass (* density volume))
         (i (* 0.4 mass radius radius)))  ;; solid sphere: I = (2/5) m r^2, same about any axis
    (list mass (float-vector i i i))))

(defun box-mass-inertia (w d h density)
  "W/D/H along local x/y/z (matches make-cube's own argument order)."
  (let* ((volume (* w d h))
         (mass (* density volume))
         (k (/ mass 12.0)))
    (list mass (float-vector (* k (+ (* d d) (* h h)))
                              (* k (+ (* w w) (* h h)))
                              (* k (+ (* w w) (* d d)))))))

(defun cylinder-mass-inertia (r h density)
  "Symmetry axis along local z (make-cylinder's own axis)."
  (let* ((volume (* pi r r h))
         (mass (* density volume))
         (i-axis (* 0.5 mass r r))                              ;; about its own axis
         (i-perp (* (/ mass 12.0) (+ (* 3.0 r r) (* h h)))))     ;; about a diameter, through the centroid
    (list mass (float-vector i-perp i-perp i-axis))))

(defun cone-mass-inertia (r h density)
  "Symmetry axis along local z, base at z=0 (make-cone-shape's own axis)."
  (let* ((volume (* (/ 1.0 3.0) pi r r h))
         (mass (* density volume))
         (i-axis (* 0.3 mass r r))                                       ;; about its own axis
         (i-perp (+ (* 0.15 mass r r) (* 0.0375 mass h h))))              ;; through the centroid
    (list mass (float-vector i-perp i-perp i-axis))))

(defclass drop-body
  :super body
  :slots (velocity acceleration angular-velocity mass local-inertia rotatable center-offset
          bounding-radius))

(defmethod drop-body
  (:init (&key shape (gravity (float-vector 0 0 -0.05)) (can-rotate t)
               (body-mass 1.0) (body-local-inertia (float-vector 0.4 0.4 0.4)))
   (replace-object self shape)
   (setq velocity (float-vector 0 0 0))
   (setq acceleration gravity)
   (setq angular-velocity (float-vector 0 0 0))
   ;; A few-cornered flat-faced shape (cube/box) resting on the floor has
   ;; several vertices tied for lowest, and which ones end up within the
   ;; :lowest-point averaging window can flip between frames as it rocks,
   ;; making the torque arm jump discontinuously and pump in energy.  A
   ;; round base (sphere/cylinder/cone) does not have this problem: any
   ;; symmetric subset of its rim still averages close to the true centre.
   ;; So torque/rolling-friction is only enabled where it stays stable;
   ;; a non-rotatable body still translates and bounces normally.
   ;;
   ;; NB: the keyword parameter is deliberately NOT named "rotatable" --
   ;; that collided with the slot of the same name (EusLisp's implicit
   ;; slot binding shadowed the argument), so (setq rotatable rotatable)
   ;; was silently a self-assignment of the slot's own uninitialized nil,
   ;; and every body ended up non-rotatable regardless of what was
   ;; passed in.  Caught by manually tracing why a cylinder given an
   ;; explicit initial spin never actually rolled.
   (setq rotatable can-rotate)
   ;; mass and (local-frame, diagonal) moment-of-inertia tensor -- see
   ;; sphere/box/cylinder/cone-mass-inertia and *drop-shape-specs*; the
   ;; defaults above (unit mass, isotropic) only matter if a caller ever
   ;; constructs a drop-body without passing real ones.  Same naming
   ;; trap as :can-rotate above: the keyword args are NOT named "mass"/
   ;; "local-inertia" (the slot names), or they would shadow just the
   ;; same way and silently leave every body at the default again.
   (setq mass body-mass)
   (setq local-inertia body-local-inertia)
   ;; make-cylinder/make-cone put the shape's local origin at one END
   ;; (the base), not at its centre of mass, unlike make-cube/make-sphere
   ;; (origin = centre already).  :rotate spins a body about its own
   ;; local origin, so left uncorrected a cylinder would swing its whole
   ;; body around that end instead of spinning about its own axis.
   ;; center-offset is the (local-frame, i.e. captured before any
   ;; :locate/:rotate) vector from the origin to the vertex centroid, an
   ;; adequate stand-in for the centre of mass here; :center below uses
   ;; it to find the true centre in world frame at any later orientation.
   (setq center-offset (scale (/ 1.0 (length (send self :vertices)))
                               (reduce #'v+ (send self :vertices))))
   ;; farthest vertex from the centre -- an upper bound on how far any
   ;; point on this body can be from its own axis of rotation, used only
   ;; by the CCD time-of-impact bound (pair-toi-bound) to turn an
   ;; angular velocity into a bound on how fast a SURFACE point can be
   ;; moving from rotation alone, on top of the body's linear velocity.
   (setq bounding-radius
         (reduce #'max (mapcar #'(lambda (v) (norm (v- v (send self :worldpos))))
                                (send self :vertices))))
   self)
  (:velocity (&optional v) (when v (setq velocity v)) velocity)
  (:angular-velocity (&optional w) (when w (setq angular-velocity w)) angular-velocity)
  (:bounding-radius () bounding-radius)
  (:rotatable () rotatable)
  (:mass () mass)
  (:center () (v+ (send self :worldpos) (send self :rotate-vector center-offset)))
  (:apply-impulse
   (j)
   "J is a linear MOMENTUM impulse (force integrated over time), not
    already a velocity change -- dividing by mass here is what makes a
    heavier body change velocity less for the same impulse."
   (setq velocity (v+ velocity (scale (/ 1.0 mass) j))))
  (:apply-angular-impulse
   (torque-impulse)
   "TORQUE-IMPULSE (r x linear-momentum-impulse, WORLD frame) divided by
    the inertia tensor to get a change in angular velocity.  The tensor
    (local-inertia) is only diagonal in the body's OWN axes, so the
    impulse is rotated into local frame, divided component-wise by the
    three principal moments there, then rotated back to world frame --
    the standard trick for a rigid body whose inertia tensor is diagonal
    only in its own (principal-axis) frame, not the world's."
   (let* ((local-ti (send self :inverse-rotate-vector torque-impulse))
          (local-dw (float-vector (/ (elt local-ti 0) (elt local-inertia 0))
                                   (/ (elt local-ti 1) (elt local-inertia 1))
                                   (/ (elt local-ti 2) (elt local-inertia 2))))
          (dw (send self :rotate-vector local-dw)))
     (setq angular-velocity (v+ angular-velocity dw)))
   ;; a runaway impulse (e.g. from a jittering contact point) must not be
   ;; able to blow the simulation up -- clamp to a sane maximum spin rate
   (let ((wn (norm angular-velocity)))
     (when (> wn 3.0) (setq angular-velocity (scale (/ 3.0 wn) angular-velocity)))))
  (:lowest-point
   ()
   "Current (world-frame) lowest point: (list z point).  The point is a
    weighted average of ALL vertices, each weighted by 1/(1+d) where d
    is how far above the true minimum that vertex sits -- so a flat
    face (cube/box) or an edge (cylinder/cone lying on its side), which
    has several vertices tied for lowest, contributes a stable contact
    centroid near all of them, but as the body tips the weighting shifts
    *continuously* toward whichever corner is now lowest.  An earlier
    version instead hard-thresholded to vertices within 1mm of the
    minimum, which for a tipping box could suddenly jump between
    2 vertices and 4 as the threshold happened to catch a different
    subset frame to frame -- a discontinuous contact point that injected
    spurious torque and could blow up the simulation; this weighting
    has no such jump."
   (weighted-extreme-point (send self :vertices) 2 :min))
  (:extreme-point
   (axis dir)
   "Generalization of :lowest-point to any AXIS (0/1/2 for x/y/z) and
    DIR (:min or :max): current (world-frame) extreme point along that
    axis, as (list extreme-coordinate point), with the same smooth
    inverse-distance weighting -- used for bouncing off any
    axis-aligned wall, not just the floor."
   (weighted-extreme-point (send self :vertices) axis dir))
  (:integrate-velocity
   (&key (dt 1.0))
   "Gravity only -- no air resistance is modelled anywhere else in this
    demo -- so this must be the ONLY thing that changes velocity between
    contacts; kept separate from :advance so a caller can integrate
    velocity once per displayed frame and then subdivide only the
    POSITION advance (see :advance and pair-toi-bound/min-pair-toi)."
   (setq velocity (v+ velocity (scale dt acceleration))))
  (:advance
   (&key (dt 1.0))
   "Move position (and, per angular-velocity, orientation) forward by
    DT, at whatever velocity/angular-velocity currently is -- does NOT
    touch velocity itself (see :integrate-velocity).  A caller can call
    this several times with smaller DTs adding up to one frame instead
    of once with DT=1 -- see bounce-between's docstring and
    pair-toi-bound: a fast body can otherwise cross another body's
    whole width between one frame and the next, so collision-distance's
    margin check (unlike the floor's tunnel-proof :lowest-point test)
    never sees them close enough and the two simply pass through each
    other."
   (send self :locate (v+ (send self :worldpos) (scale dt velocity)) :world)
   (let ((wn (norm angular-velocity)))
     (when (> wn 1e-6)
       ;; :rotate spins the body about its own local ORIGIN, which is not
       ;; the centre of mass for a cylinder/cone (see center-offset
       ;; above).  Rotating naively would let the true centre swing away
       ;; from where it physically is; instead rotate in place and then
       ;; re-locate the origin so the centre ends up exactly where it
       ;; was before this step (a standard pivot-about-an-offset-point
       ;; correction) -- linear motion is applied separately, above.
       (let ((c-before (send self :center)))
         (send self :rotate (* wn dt) (scale (/ 1.0 wn) angular-velocity) :world)
         (send self :locate (v+ (send self :worldpos) (v- c-before (send self :center))) :world)))))
  (:step
   (&key (dt 1.0))
   "Convenience: :integrate-velocity then :advance, both by DT -- for a
    caller that does not need CCD/time-of-impact-adaptive sub-stepping
    (hand-grasp.l, which only ever has one falling object active, so
    there is no second body for it to tunnel through)."
   (send self :integrate-velocity :dt dt)
   (send self :advance :dt dt))
  (:bounce-off
   (&key (normal (float-vector 0 0 1)) (restitution 0.4) (push-out 0.0))
   (when (> push-out 0.0)
     (send self :locate (v+ (send self :worldpos) (scale push-out normal)) :world))
   (let ((vn (v. velocity normal)))
     (when (< vn 0.0)
       (setq velocity (v- velocity (scale (* (+ 1.0 restitution) vn) normal)))))
   velocity))

(defun contact-normal (r)
  "R is a collision-distance result (list distance p1 p2); nearest-point
   direction p1->p2, or straight up if the two points coincide."
  (let ((diff (v- (elt r 1) (elt r 2))))
    (if (> (norm diff) 1e-6) (normalize-vector diff) (float-vector 0 0 1))))

;; --- continuous collision detection (conservative advancement) --------
;; The floor and walls are already tunnel-proof by construction (they
;; are tested by a body's own absolute extreme point against a fixed
;; plane -- see :lowest-point/:extreme-point -- which cannot be skipped
;; over no matter how far a body moves in one step).  Body-to-body
;; contact (bounce-between) has no such absolute test available -- it
;; only knows collision-distance at whatever instant it is asked -- so
;; instead of guessing a fixed number of smaller steps (which only
;; lowers the ODDS of tunnelling for a given speed, not rule it out for
;; an arbitrarily fast one), pair-toi-bound/min-pair-toi compute an
;; actual, provably-safe upper bound on how long two bodies can be
;; advanced at their current velocities before they might come within
;; MARGIN of each other: the standard "conservative advancement" method
;; used for continuous collision detection in real physics engines.
;; The caller (drop-primitives/drop-primitives-in-box) advances every
;; body by that bound (or the rest of the frame, if smaller), re-checks,
;; and repeats -- a large gap between two bodies is crossed in one big
;; jump, only an actually-closing pair forces the step size down, and it
;; is mathematically impossible for that pair to pass through each
;; other unnoticed regardless of how fast they are moving.
(defun pair-toi-bound (a b margin)
  "Conservative lower bound on how much time must pass before A and B
   could possibly come within MARGIN of each other, given their CURRENT
   velocities/angular velocities: 0.0 if already within MARGIN AND still
   closing (so bounce-between must run right now, before either is
   allowed to move any further), NIL if they are not closing at all --
   whether or not they are already within MARGIN -- since then no
   further bound is needed from this pair: either they are far apart
   and not approaching, or they are a settled resting/stacked contact
   that bounce-between has already resolved (a pair permanently at
   d<=margin, e.g. two stacked bodies, must NOT keep forcing a
   zero-length step forever once it is no longer closing -- that would
   freeze every body's position for the whole frame, since the caller
   takes the MINIMUM bound over all pairs, while gravity still
   integrates into velocity every frame regardless; only an actually
   closing pair forces the step size down).
   The bound uses the actual contact normal to find the closing
   component of relative linear velocity (so a separating pair is not
   needlessly treated as an emergency), plus a worst-case bound on how
   fast rotation alone could move either body's surface -- angular
   speed times that body's own bounding radius -- since a spinning body
   can bring a point around to meet the other pair even while its
   centre is not closing at all.
   NB: that rotational term must NOT be part of what decides whether an
   ALREADY-touching pair (d<=margin) needs a mandatory zero-step -- a
   resting contact routinely still has one or both bodies spinning
   (rolling friction damps it only slowly, and a cylinder/cone starts
   with real angular velocity), which would otherwise keep v-max
   permanently above the 1e-6 threshold and force a zero-step on every
   single iteration forever, freezing that frame's position advance
   completely (this was observed: positions frozen solid for 50+
   consecutive frames while gravity kept integrating into velocity
   regardless).  Once two bodies are already within MARGIN, only actual
   LINEAR closing speed along the contact normal (not rotation) should
   force an immediate resolve; the rotational bound still matters for
   the other branch below, where it can be what closes an existing gap
   between two bodies not yet touching."
  (let* ((r (collision-distance a b)) (d (car r)))
    (let* ((n (contact-normal r))
           (closing-speed (max 0.0 (- (v. (v- (send a :velocity) (send b :velocity)) n))))
           (w-bound (+ (* (norm (send a :angular-velocity)) (send a :bounding-radius))
                       (* (norm (send b :angular-velocity)) (send b :bounding-radius))))
           (v-max (+ closing-speed w-bound)))
      (cond
       ((<= d margin) (if (> closing-speed 1e-6) 0.0 nil))
       ((< v-max 1e-6) nil)
       (t (/ (- d margin) v-max))))))

(defun min-pair-toi (bodies margin)
  "Smallest pair-toi-bound over every pair of BODIES, or NIL if none of
   them are closing with any other at all."
  (let (best)
    (dotimes (i (length bodies))
      (do ((j (1+ i) (1+ j))) ((>= j (length bodies)))
        (let ((tt (pair-toi-bound (elt bodies i) (elt bodies j) margin)))
          (when (and tt (or (null best) (< tt best))) (setq best tt)))))
    best))

(defun ccd-advance-frame (bodies margin max-iters resolve-fn)
  "Advance BODIES through one whole frame (dt totalling 1.0, at whatever
   velocity/angular-velocity :integrate-velocity already set) without
   tunnelling: repeatedly find the smallest pair-toi-bound over all of
   BODIES, :advance every body by that (or by whatever time is left in
   the frame, if smaller), then call RESOLVE-FN -- a thunk with no
   arguments that applies whatever floor/wall bounces, friction and
   bounce-between are relevant to the caller -- to resolve any contact
   this step reached, and repeat until the full frame is consumed or
   MAX-ITERS is hit.
   Several bodies meeting each other at nearly the same instant (e.g.
   prim-drop.l's group-drop, five shapes converging on the centre
   together, or several bodies wedged against two walls at once in
   prim-box.l) can make pairwise resolution cycle for a few iterations
   -- resolving pair (A,B) can leave A newly closing with C, resolving
   that can put A back on a collision course with B, and so on --
   before every pair is genuinely separating at once.  Each such
   iteration correctly reports a zero time bound (something IS still
   closing), so without a fallback the loop would keep re-resolving
   without ever advancing position at all; since gravity still
   integrates into velocity every frame regardless of whether position
   moved, this was observed to become a PERMANENT freeze from the
   frame it first happened in onward, not a transient stall (verified
   with a headless diagnostic: position identical for 100+ consecutive
   frames, velocity magnitude still growing).  So: if several
   iterations in a row make no time progress at all (tracked by
   ZERO-RUN below), this is no longer a single clean impact with a
   precise time-of-impact worth computing -- give up on TOI precision
   for the rest of THIS frame and consume all of the remaining time in
   one shot instead.  The cost is a slightly less exact contact instant
   for this one messy multi-way collision, not a return to the
   tunnelling this whole mechanism exists to avoid: the floor and
   walls are separately tunnel-proof by construction (a body's own
   absolute extreme point against a fixed plane, not a distance-based
   margin check), and any body-body overlap this fallback could
   momentarily leave is bounded by MARGIN itself, exactly as it always
   was before CCD replaced fixed substepping.
   NB: \"no progress\" must be judged against a threshold that is a
   meaningful FRACTION of the whole frame (dt totals 1.0), not against
   an near-zero epsilon like 1e-9 -- profiling with the latter showed
   pair-toi-bound routinely returning tiny but not-quite-zero bounds
   during a crowded moment (float slack from bounce-between's own
   push-out leaving d a hair's breadth above MARGIN, against a large
   V-MAX from fast-moving bodies), so ZERO-RUN kept getting reset to 0
   just above that epsilon and this escape valve never actually
   triggered.
   A SEPARATE problem profiling then found even after fixing that: a
   frame with several bodies in prolonged shallow/glancing contact
   (e.g. group-drop's five shapes converging together, still sliding
   past each other rather than cleanly separating) can need FAR more
   than MAX-ITERS genuine, non-stalled TOI events to fully play out --
   one measured frame took 32 real (non-zero, ~0.02 each) advances and
   still only consumed 65% of the frame before hitting the iteration
   cap, silently DROPPING the remaining 35% of that frame's simulated
   time rather than ever applying it.  That is worse than merely slow:
   a body that keeps losing a chunk of simulated time exactly when the
   scene is busiest visibly falls behind and looks wrong compared to
   how the same collision would actually play out, on top of costing
   600+ms of real time for what should be a 15ms frame.  So: whatever
   is left over once the loop below exits (whether from genuine
   completion or from hitting MAX-ITERS) is always applied in one
   final catch-up advance+resolve after it -- every frame's full 1.0
   of simulated time is guaranteed to be consumed no matter what, at
   the cost of one less-precise final contact instant only in this
   rare dense-cluster case (MARGIN bounds how much that imprecision
   can ever mean for body-body overlap, same as above)."
  (let ((remaining 1.0) (iters 0) (zero-run 0) (stall-dt 1e-3))
    (while (and (> remaining 1e-4) (< iters max-iters))
      (incf iters)
      (let ((safe-t (min remaining (or (min-pair-toi bodies margin) remaining))))
        (if (< safe-t stall-dt)
            (progn
              (incf zero-run)
              (when (> zero-run 3) (setq safe-t remaining)))
          (setq zero-run 0))
        (dolist (b bodies) (send b :advance :dt safe-t))
        (setq remaining (- remaining safe-t))
        (funcall resolve-fn)))
    (when (> remaining 1e-4)
      (dolist (b bodies) (send b :advance :dt remaining))
      (funcall resolve-fn))))

(defun bounce-against-floor (obj floor-top-z &key (restitution 0.35))
  "Tunnel-proof floor contact using the body's own current lowest vertex
   (see drop-body's :lowest-point), so a fast fall cannot skip past a
   thin collision-distance margin in a single time-step, and torque is
   applied from the actual (possibly off-centre, once tipped) contact
   point rather than assuming contact is always straight under the
   centre of mass."
  (let* ((lp (send obj :lowest-point)) (lz (car lp)) (contact-pt (cadr lp)))
    (when (<= lz floor-top-z)
      (let ((v-before (copy-seq (send obj :velocity))))
        (send obj :bounce-off :normal (float-vector 0 0 1) :restitution restitution
              :push-out (- floor-top-z lz))
        (when (send obj :rotatable)
          ;; :bounce-off's reflection is a plain kinematic formula (correct
          ;; regardless of mass, since it is bouncing off an immovable
          ;; floor); the velocity CHANGE it produced, scaled back up by
          ;; this body's own mass, recovers the momentum-like impulse
          ;; :apply-angular-impulse expects.
          (let* ((dv (v- (send obj :velocity) v-before))
                 (r (v- contact-pt (send obj :center))))
            (send obj :apply-angular-impulse (scale (send obj :mass) (v* r dv)))))))))

(defun apply-surface-friction (obj axis dir wall-coord &key (margin 2.0) (mu 0.1) (damping 0.99))
  "Generalization of the floor-only friction this replaced (see
   apply-rolling-friction below) to ANY axis-aligned surface -- AXIS
   0/1/2, DIR :min/:max, WALL-COORD its position, same meaning as
   bounce-against-wall/:extreme-point.  While OBJ's extreme point along
   AXIS/DIR is within MARGIN of WALL-COORD (not just at the instant of a
   bounce), damp the TANGENTIAL (in-plane, i.e. velocity with the
   surface-normal component removed) velocity of that contact point a
   little each frame -- what turns a spinning-in-place body into one
   that actually rolls away, or a sliding one into a rolling one: a
   crude per-frame relaxation toward the no-slip condition, not an
   exact impulse solve, but enough to see the qualitative rolling
   behaviour of each shape.  Once truly rolling without slipping this
   stops doing anything (the contact point's own velocity is already
   ~0), so a well-balanced roller would otherwise roll (or, against a
   wall, slide) forever -- real surfaces are never quite frictionless,
   and a demo that waits for everything to visibly settle before moving
   on must not wait forever for one still merrily bouncing around the
   arena, so a gentle constant DAMPING on top bleeds off that last bit
   of energy over a few hundred frames without being noticeable
   frame-to-frame.  Only the floor had this in an earlier version of
   this file; a body bouncing between walls or the ceiling had no
   friction of any kind and only restitution loss to ever slow it down,
   which could take a very long time in a large box."
  (let* ((ep (send obj :extreme-point axis dir)) (ev (car ep)) (contact-pt (cadr ep))
         (near (if (eq dir :min) (<= ev (+ wall-coord margin)) (>= ev (- wall-coord margin))))
         (n (float-vector 0 0 0)))
    (setf (elt n axis) (if (eq dir :min) 1.0 -1.0))
    (when (and (send obj :rotatable) near)
      (let* ((r (v- contact-pt (send obj :center)))
             (vc (v+ (send obj :velocity) (v* (send obj :angular-velocity) r)))
             (vc-t (v- vc (scale (v. vc n) n))))
        (when (> (norm vc-t) 1e-6)
          ;; J here is meant as a velocity-change-sized quantity (the old,
          ;; implicitly-unit-mass style of this formula); scaling by mass
          ;; turns it into a real momentum impulse for :apply-impulse and
          ;; :apply-angular-impulse, both of which now divide by mass (or
          ;; the inertia tensor, which already has mass baked in) internally.
          (let* ((j (scale (- mu) vc-t))
                 (momentum-j (scale (send obj :mass) j)))
            (send obj :apply-impulse momentum-j)
            (send obj :apply-angular-impulse (v* r momentum-j)))))
      (send obj :velocity (scale damping (send obj :velocity)))
      (send obj :angular-velocity (scale damping (send obj :angular-velocity))))))

(defun apply-rolling-friction (obj floor-top-z &key (margin 2.0) (mu 0.1) (damping 0.99))
  "Floor-specific case of apply-surface-friction (axis 2, :min), kept
   under its original name for prim-drop.l's and hand-grasp.l's own
   floor-only calls."
  (apply-surface-friction obj 2 :min floor-top-z :margin margin :mu mu :damping damping))

(defun bounce-between (a b margin &key (restitution 0.5))
  "Purely translational bounce between two free drop-bodies A and B (no
   rotational coupling between two moving bodies -- see the file
   header), but properly mass-aware: the standard reduced-mass impulse
   J = -(1+e) vn / (1/ma + 1/mb) along the contact normal, so a heavy
   body barely moves when a light one hits it instead of the two
   trading velocity as if they had equal mass.
   Unlike the floor (bounce-against-floor, tunnel-proof via a body's own
   :lowest-point), this still only fires when collision-distance already
   reports the two bodies within MARGIN -- a thin band a fast-enough
   relative approach can cross in a single :advance without ever landing
   inside it, letting the two pass clean through each other.  The demo
   functions do not call :advance with one big DT per displayed frame;
   see pair-toi-bound/min-pair-toi above and drop-primitives/
   drop-primitives-in-box: they compute how long it can possibly take
   before any pair gets this close and never advance further than that
   in one go, so this function itself does not need to guard against
   tunnelling -- by the time it is called, the pair is already known to
   be within reach."
  (let* ((r (collision-distance a b)) (d (car r)))
    (when (<= d margin)
      (let* ((n (contact-normal r))
             (push (max 0.0 (- margin d)))
             (vn (v. (v- (send a :velocity) (send b :velocity)) n)))
        (when (> push 0.0)
          (send a :locate (v+ (send a :worldpos) (scale (* 0.5 push) n)) :world)
          (send b :locate (v+ (send b :worldpos) (scale (* -0.5 push) n)) :world))
        (when (< vn 0.0)
          (let* ((inv-mass-sum (+ (/ 1.0 (send a :mass)) (/ 1.0 (send b :mass))))
                 (j-scalar (/ (* (- (+ 1.0 restitution)) vn) inv-mass-sum))
                 (j (scale j-scalar n)))
            (send a :apply-impulse j)
            (send b :apply-impulse (scale -1.0 j))))))))

(defun make-cone-shape (r h &key (segments 16))
  "Approximate cone: apex at (0 0 H), circular base of radius R at z=0."
  (let (verts)
    (dotimes (i segments)
      (let ((a (* 2 pi (/ (float i) segments))))
        (push (float-vector (* r (cos a)) (* r (sin a)) 0) verts)))
    (make-cone (float-vector 0 0 h) verts)))

(defun random-horizontal-axis ()
  (normalize-vector (float-vector (- (random 2.0) 1.0) (- (random 2.0) 1.0) 0)))

;; Per-shape spec used both for the group drop and the solo showcase.
;;   name        -- printed when the solo showcase moves to this shape
;;   make        -- thunk building a fresh shape (local frame, untilted)
;;   color       -- float-vector
;;   lie-on-side -- t for cylinder/cone: rotated 90 deg so it lands on
;;                  its curved/slanted side instead of a flat end, which
;;                  is what lets it roll at all
;;   rotatable   -- t for every shape now that :lowest-point/:extreme-point
;;                  use a smooth weighted contact point instead of a hard
;;                  within-1mm vertex threshold (see drop-body's
;;                  :lowest-point docstring) -- a few-cornered flat
;;                  contact used to jitter and blow up under that hard
;;                  threshold, which is why cube/box were kept
;;                  non-rotatable at first
;;   mass-inertia -- (list mass local-inertia), from the shape's own
;;                  dimensions and an assigned DENSITY via
;;                  sphere/box/cylinder/cone-mass-inertia -- a real
;;                  material property, not just a size-derived guess;
;;                  the specific density numbers below are only chosen
;;                  to be illustratively different materials (rubber,
;;                  wood, aluminium, steel, foam), not tied to any real
;;                  unit system (this demo's gravity is not one either).
;;
;;   NB: the dimensions are written as literals directly in each
;;   make-fn lambda, duplicated in the mass-inertia call right next to
;;   it, rather than shared via an enclosing let* -- a lambda stored in
;;   *drop-shape-specs* and only called much later (well after this
;;   defparameter's own let* has returned) segfaulted in make-sphere
;;   when it tried to close over a let*-bound variable from that outer
;;   scope: EusLisp closures here do not safely outlive the dynamic
;;   extent of the let* that bound the variable.  Literal constants in
;;   the lambda body do not have this problem.
(defparameter *drop-shape-specs*
    (list (append (list :sphere #'(lambda () (make-sphere 40))
                        (float-vector 0.85 0.2 0.2) nil t)
                  (sphere-mass-inertia 40 1.0))          ;; rubber-ish
          (append (list :cube #'(lambda () (make-cube 70 70 70))
                        (float-vector 0.2 0.75 0.2) nil t)
                  (box-mass-inertia 70 70 70 0.6))        ;; wood-ish
          (append (list :box #'(lambda () (make-cube 60 100 40))
                        (float-vector 0.2 0.4 0.9) nil t)
                  (box-mass-inertia 60 100 40 2.7))        ;; aluminium-ish
          (append (list :cylinder #'(lambda () (make-cylinder 35 90))
                        (float-vector 0.9 0.8 0.1) t t)
                  (cylinder-mass-inertia 35 90 5.0))       ;; steel-ish
          (append (list :cone #'(lambda () (make-cone-shape 40 90))
                        (float-vector 0.8 0.2 0.8) t t)
                  (cone-mass-inertia 40 90 0.5))))          ;; foam-ish

(defun make-drop-bodies ()
  (mapcar #'(lambda (spec)
              (let ((b (instance drop-body :init :shape (funcall (elt spec 1))
                                  :can-rotate (elt spec 4)
                                  :body-mass (elt spec 5)
                                  :body-local-inertia (elt spec 6))))
                (send b :set-color (elt spec 2))
                b))
          *drop-shape-specs*))

(defun drop-primitives ()
  "Sphere/cube/box/cylinder/cone fall together onto a floor and bounce
   off it and off each other; once everything settles, the same five are
   dropped again but all from dead centre, released one after another in
   quick succession (not waiting for each to land) so later ones come
   down on top of the earlier ones and you can watch them collide and
   get knocked around, then the group drop happens again.  Any key
   stops the demo."
  (send *irtviewer* :title "prim-drop")
  (let* ((bodies (make-drop-bodies))
         (n (length bodies))
         (flr (make-cube 1200 1200 20))
         (floor-top-z 0.0)                    ;; flr is located so its top face is at world z=0
         (margin 3.0)
         (drop-radius 180.0)
         (drop-height 350.0)
         (inward-speed 1.2)
         (release-interval 90)   ;; frames between releases -- shorter than the
                                 ;; ~120-frame fall time, so the next body is
                                 ;; usually still on its way down or just
                                 ;; landing when it meets the previous one
         (parked-spot (float-vector 2000 2000 0))  ;; well off to the side, out of the way
         (max-ccd-iters 12)  ;; see pair-toi-bound/min-pair-toi/ccd-advance-frame:
                             ;; a cap on how many times one frame's position
                             ;; advance can be subdivided before ccd-advance-
                             ;; frame's own final catch-up step just consumes
                             ;; whatever is left in one go -- since that catch-
                             ;; up now guarantees the frame always fully
                             ;; completes regardless, this can stay low (cheap:
                             ;; bounds the worst case cost of a dense, slowly-
                             ;; resolving cluster) without any risk of a
                             ;; pathological case spinning the loop forever
         (settle-count 0)
         stage stream-index release-timer)
    (send flr :locate (float-vector 0 0 -10) :world)
    (send flr :set-color (float-vector 0.6 0.6 0.6))
    (objects (append bodies (list flr)))
    (labels
        ((reset-body (b)
           (send b :newcoords (make-coords))
           (send b :velocity (float-vector 0 0 0))
           (send b :angular-velocity (float-vector 0 0 0)))
         (drop-shape (b i pos)
           "Reset body I (shape spec index I) and drop it from POS,
            starting from a slightly tilted posture (not dead level --
            a real object rarely leaves your hand perfectly plumb), plus
            whatever extra tilt/spin its shape needs to plausibly tip or
            roll; every shape can now rotate, so a cube/box landing on a
            corner or edge tips itself back onto a face instead of
            staying frozen at this initial tilt."
           (reset-body b)
           (send b :locate pos :world)
           (send b :rotate (deg2rad (+ 3 (random 8.0))) (random-horizontal-axis) :world)
           (let ((lie (elt (elt *drop-shape-specs* i) 3)))
             (when lie
               ;; :rotate 90deg about world x puts the cylinder/cone's
               ;; own length axis along world y (checked empirically);
               ;; a guaranteed spin about that same axis is what reliably
               ;; starts it rolling once it lands -- the tilt above alone
               ;; would only sometimes create enough off-centre contact
               ;; to torque it into rolling
               (send b :rotate (deg2rad 90) :x :world)
               (send b :angular-velocity (float-vector 0 0.15 0)))))
         (start-group ()
           (dotimes (i n)
             (let* ((a (* 2 pi (/ (float i) n)))
                    (pos (float-vector (* drop-radius (cos a)) (* drop-radius (sin a)) drop-height))
                    (dir (normalize-vector (float-vector (- (cos a)) (- (sin a)) 0))))
               (drop-shape (elt bodies i) i pos)
               (send (elt bodies i) :velocity (scale inward-speed dir))))
           (setq stage :group settle-count 0)
           (warn "~%;; group drop: all ~a shapes together~%" n))
         (start-stream ()
           (dotimes (k n)
             (reset-body (elt bodies k))
             (send (elt bodies k) :locate (v+ parked-spot (float-vector 0 (* 250 k) 0)) :world))
           (setq stage :stream stream-index 0 release-timer 0 settle-count 0)
           (warn "~%;; stream drop: releasing all ~a shapes in succession from centre~%" n))
         (release-next-in-stream ()
           (drop-shape (elt bodies stream-index) stream-index (float-vector 0 0 drop-height))
           (warn ";; releasing ~a/~a: ~a~%" (1+ stream-index) n (car (elt *drop-shape-specs* stream-index)))
           (incf stream-index)
           (setq release-timer release-interval))
         (active-bodies ()
           (case stage
             (:group bodies)
             (:stream (subseq bodies 0 stream-index))))
         (all-settled-p ()
           ;; only vertical linear speed matters for "has it stopped
           ;; bouncing" -- with no ground friction on top of rolling
           ;; friction a body could keep a slow constant horizontal
           ;; drift or spin, which must not block noticing it landed
           (every #'(lambda (b) (< (abs (elt (send b :velocity) 2)) 0.3)) (active-bodies))))
      (start-group)
      (do-until-key
       (when (and (eq stage :stream) (< stream-index n))
         (if (<= release-timer 0)
             (release-next-in-stream)
           (decf release-timer)))
       (let ((active (active-bodies)))
         (dolist (b active) (send b :integrate-velocity :dt 1.0))  ;; gravity, once per frame
         (ccd-advance-frame
          active margin max-ccd-iters
          #'(lambda ()
              (dolist (b active) (bounce-against-floor b floor-top-z))
              (dolist (b active) (apply-rolling-friction b floor-top-z))
              (let ((k (length active)))
                (dotimes (i k)
                  (do ((j (1+ i) (1+ j))) ((>= j k))
                    (bounce-between (elt active i) (elt active j) margin)))))))
       (if (and (all-settled-p) (or (eq stage :group) (>= stream-index n)))
           (progn
             (incf settle-count)
             (when (>= settle-count 150)
               (if (eq stage :group) (start-stream) (start-group))))
         (setq settle-count 0))
       (send *irtviewer* :draw-objects)
       (unix::usleep (* 15 1000))
       (x::window-main-one))
      )))

(unless (boundp '*irtviewer*) (make-irtviewer))
(warn "(drop-primitives) ;; group drop, then all shapes dropped in succession from centre~%")
