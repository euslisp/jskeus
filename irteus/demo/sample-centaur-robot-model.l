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

(defclass sample-centaur-robot
  :super robot-model
  :slots (jc0 jc1 jh0 jh1
              jal0 jal1 jal2 jal3 jal4 jal5 jal6
              jar0 jar1 jar2 jar3 jar4 jar5 jar6
              jllf0 jllf1 jllf2 jllf3 jllf4 jllf5
              jlrf0 jlrf1 jlrf2 jlrf3 jlrf4 jlrf5
              jllb0 jllb1 jllb2 jllb3 jllb4 jllb5
              jlrb0 jlrb1 jlrb2 jlrb3 jlrb4 jlrb5))
(defmethod sample-centaur-robot
  (:init 
   (&rest args
    &key (name "sample-centaur-robot")
         ;; key word arguments for configure model size
         (leg-radius 50) (upper-leg-length 250) (lower-leg-length 250) (ankle-length 50)
         (crotch-width 75) (foot-depth 200) (foot-width 100) (foot-thickness 25) (foot-offset 50)
	 (arm-radius 50) (upper-arm-length 275) (lower-arm-length 195) (shoulder-width 150) (hand-length 50)
         (waist-height 100) (waist-length 500) (torso-height 200) (torso-length 100)
         ;; mass distribution
         ;;   refering to "WEIGHT, VOLUME, AND CENTER OF MASS OF SEGMENTS OF THE HUMAN BODY", C.E.Clauser et al., AMRL technical report TR-69-70(1969), http://www.dtic.mil/dtic/tr/fulltext/u2/710622.pdf
         ;;   ratio is '((:head 7.3) (:trunk 50.7) (:upper-arm 2.6) (:forearm 1.6) (:hand 0.7) (:tight 10.3) (:calf 4.3) (:foot 1.5))
         ;;   default total mass is 20.9[kg], which is calculated from total-height and avarage 6-years-old-Japanese Male weight http://www.mhlw.go.jp/toukei/youran/data25k/2-06.xls
         (head-weight 1525.7)
         (torso-weight (/ (* torso-height 10596.3) (+ torso-height waist-height)))
         (waist-weight (/ (* waist-length (/ (* waist-height 10596.3) (+ torso-height waist-height))) torso-length))
         (upper-arm-weight 543.4) (lower-arm-weight 334.4) (hand-weight 146.3)
         (upper-leg-weight 2152.7) (lower-leg-weight 898.7) (foot-weight 313.5)
         )
   (send-super* :init :name name args)
   ;; 1. make links and assoc all links
   (let* ((aroot-link (send self :make-root-link waist-height waist-length waist-weight))
          (torso-info (send self :make-torso-links torso-height torso-length torso-weight))
          (torso (cdr (assoc :links torso-info)))
          (torso-end-coords (cdr (assoc :end-coords torso-info)))
          (torso-root-link (cdr (assoc :root-link torso-info)))
          (head-info (send self :make-head-links head-weight))
          (head (cdr (assoc :links head-info)))
          (head-end-coords (cdr (assoc :end-coords head-info)))
          (head-root-link (cdr (assoc :root-link head-info)))
          (rarm-info (send self :make-arm-links :rarm
                           arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
                           upper-arm-weight lower-arm-weight hand-weight))
          (rarm (cdr (assoc :links rarm-info)))
          (rarm-end-coords (cdr (assoc :end-coords rarm-info)))
          (rarm-root-link (cdr (assoc :root-link rarm-info)))
          (larm-info (send self :make-arm-links :larm
                           arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
                           upper-arm-weight lower-arm-weight hand-weight))
          (larm (cdr (assoc :links larm-info)))
          (larm-end-coords (cdr (assoc :end-coords larm-info)))
          (larm-root-link (cdr (assoc :root-link larm-info)))
          (rfleg-info (send self :make-leg-links :rleg
                           leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                           upper-leg-weight lower-leg-weight foot-weight))
          (rfleg (cdr (assoc :links rfleg-info)))
          (rfleg-end-coords (cdr (assoc :end-coords rfleg-info)))
          (rfleg-root-link (cdr (assoc :root-link rfleg-info)))
          (lfleg-info (send self :make-leg-links :lleg
                           leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                           upper-leg-weight lower-leg-weight foot-weight))
          (lfleg (cdr (assoc :links lfleg-info)))
          (lfleg-end-coords (cdr (assoc :end-coords lfleg-info)))
          (lfleg-root-link (cdr (assoc :root-link lfleg-info)))
          (rbleg-info (send self :make-leg-links :rleg
                           leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                           upper-leg-weight lower-leg-weight foot-weight))
          (rbleg (cdr (assoc :links rbleg-info)))
          (rbleg-end-coords (cdr (assoc :end-coords rbleg-info)))
          (rbleg-root-link (cdr (assoc :root-link rbleg-info)))
          (lbleg-info (send self :make-leg-links :lleg
                           leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
                           upper-leg-weight lower-leg-weight foot-weight))
          (lbleg (cdr (assoc :links lbleg-info)))
          (lbleg-end-coords (cdr (assoc :end-coords lbleg-info)))
          (lbleg-root-link (cdr (assoc :root-link lbleg-info))))
     ;; arrange limbs
     (send aroot-link :translate (float-vector (/ (- torso-length waist-length) 2) 0 0) :world)
     (send (car rarm) :translate (float-vector 0 (- shoulder-width) (- torso-height 25)) :world)
     (send (car larm) :translate (float-vector 0 shoulder-width (- torso-height 25)) :world)
     (send (car rfleg) :translate (float-vector 0 (- crotch-width) (+ (- waist-height) -50)) :world)
     (send (car lfleg) :translate (float-vector 0 crotch-width (+ (- waist-height) -50)) :world)
     (send (car rbleg) :translate (float-vector (- torso-length waist-length) (- crotch-width) (+ (- waist-height) -50)) :world)
     (send (car lbleg) :translate (float-vector (- torso-length waist-length) crotch-width (+ (- waist-height) -50)) :world)
     (send (car head) :locate (float-vector 0 0 (+ 25 torso-height)))

     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc aroot-link)
     (send aroot-link :assoc (car torso))
     (send (cadr torso) :assoc (car head))
     (send (cadr torso) :assoc (car rarm))
     (send (cadr torso) :assoc (car larm))
     (send aroot-link :assoc (car rfleg))
     (send aroot-link :assoc (car lfleg))
     (send aroot-link :assoc (car rbleg))
     (send aroot-link :assoc (car lbleg))

     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (setq jc0 (instance rotational-joint :init :parent-link aroot-link :child-link (car torso) :name :torso-waist-y :axis :z :min -45 :max 45))
     (setq jc1 (instance rotational-joint :init :parent-link (car torso) :child-link (cadr torso) :name :torso-waist-p :axis :y))

     (setq jh0 (instance rotational-joint :init :parent-link (cadr torso) :child-link (car head) :name :head-neck-y :axis :z :min -150 :max 150))
     (setq jh1 (instance rotational-joint :init :parent-link (car head) :child-link (cadr head) :name :head-neck-p :axis :y))
     (setq jal0 (instance rotational-joint :init :parent-link (cadr torso) :child-link (elt larm 0) :name :larm-shoulder-p :axis :y :min -150 :max 150))
     (setq jal1 (instance rotational-joint :init :parent-link (elt larm 0) :child-link (elt larm 1) :name :larm-shoulder-r :axis :x :min -30 :max 180))
     (setq jal2 (instance rotational-joint :init :parent-link (elt larm 1) :child-link (elt larm 2) :name :larm-shoulder-y :axis :z))
     (setq jal3 (instance rotational-joint :init :parent-link (elt larm 2) :child-link (elt larm 3) :name :larm-elbow-p :axis :y :min -180 :max 0))
     (setq jal4 (instance rotational-joint :init :parent-link (elt larm 3) :child-link (elt larm 4) :name :larm-wrist-y :axis :z))
     (setq jal5 (instance rotational-joint :init :parent-link (elt larm 4) :child-link (elt larm 5) :name :larm-wrist-r :axis :x))
     (setq jal6 (instance rotational-joint :init :parent-link (elt larm 5) :child-link (elt larm 6) :name :larm-wrist-p :axis :y))

     (setq jar0 (instance rotational-joint :init :parent-link (cadr torso)  :child-link (elt rarm 0) :name :rarm-shoulder-p :axis :y :min -150 :max 150))
     (setq jar1 (instance rotational-joint :init :parent-link (elt rarm 0) :child-link (elt rarm 1) :name :rarm-shoulder-r :axis :-x :min -30 :max 180))
     (setq jar2 (instance rotational-joint :init :parent-link (elt rarm 1) :child-link (elt rarm 2) :name :rarm-shoulder-y :axis :-z))
     (setq jar3 (instance rotational-joint :init :parent-link (elt rarm 2) :child-link (elt rarm 3) :name :rarm-elbow-p :axis :y :min -180 :max 0))
     (setq jar4 (instance rotational-joint :init :parent-link (elt rarm 3) :child-link (elt rarm 4) :name :rarm-wrist-y :axis :-z))
     (setq jar5 (instance rotational-joint :init :parent-link (elt rarm 4) :child-link (elt rarm 5) :name :rarm-wrist-r :axis :-x))
     (setq jar6 (instance rotational-joint :init :parent-link (elt rarm 5) :child-link (elt rarm 6) :name :rarm-wrist-p :axis :y))

     (setq jllf0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt lfleg 0) :name :lfleg-crotch-y :axis :z))
     (setq jllf1 (instance rotational-joint :init :parent-link (elt lfleg 0) :child-link (elt lfleg 1) :name :lfleg-crotch-r :axis :x))
     (setq jllf2 (instance rotational-joint :init :parent-link (elt lfleg 1) :child-link (elt lfleg 2) :name :lfleg-crotch-p :axis :y :min -120 :max 120))
     (setq jllf3 (instance rotational-joint :init :parent-link (elt lfleg 2) :child-link (elt lfleg 3) :name :lfleg-knee-p :axis :y :min 0))
     (setq jllf4 (instance rotational-joint :init :parent-link (elt lfleg 3) :child-link (elt lfleg 4) :name :lfleg-ankle-p :axis :y))
     (setq jllf5 (instance rotational-joint :init :parent-link (elt lfleg 4) :child-link (elt lfleg 5) :name :lfleg-ankle-r :axis :x))

     (setq jlrf0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt rfleg 0) :name :rfleg-crotch-y :axis :-z))
     (setq jlrf1 (instance rotational-joint :init :parent-link (elt rfleg 0) :child-link (elt rfleg 1) :name :rfleg-crotch-r :axis :-x))
     (setq jlrf2 (instance rotational-joint :init :parent-link (elt rfleg 1) :child-link (elt rfleg 2) :name :rfleg-crotch-p :axis :y :min -120 :max 120))
     (setq jlrf3 (instance rotational-joint :init :parent-link (elt rfleg 2) :child-link (elt rfleg 3) :name :rfleg-knee-p :axis :y :min 0))
     (setq jlrf4 (instance rotational-joint :init :parent-link (elt rfleg 3) :child-link (elt rfleg 4) :name :rfleg-ankle-p :axis :y))
     (setq jlrf5 (instance rotational-joint :init :parent-link (elt rfleg 4) :child-link (elt rfleg 5) :name :rfleg-ankle-r :axis :-x))

     (setq jllb0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt lbleg 0) :name :lbleg-crotch-y :axis :z))
     (setq jllb1 (instance rotational-joint :init :parent-link (elt lbleg 0) :child-link (elt lbleg 1) :name :lbleg-crotch-r :axis :x))
     (setq jllb2 (instance rotational-joint :init :parent-link (elt lbleg 1) :child-link (elt lbleg 2) :name :lbleg-crotch-p :axis :y :min -120 :max 120))
     (setq jllb3 (instance rotational-joint :init :parent-link (elt lbleg 2) :child-link (elt lbleg 3) :name :lbleg-knee-p :axis :y :min 0))
     (setq jllb4 (instance rotational-joint :init :parent-link (elt lbleg 3) :child-link (elt lbleg 4) :name :lbleg-ankle-p :axis :y))
     (setq jllb5 (instance rotational-joint :init :parent-link (elt lbleg 4) :child-link (elt lbleg 5) :name :lbleg-ankle-r :axis :x))

     (setq jlrb0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt rbleg 0) :name :rbleg-crotch-y :axis :-z))
     (setq jlrb1 (instance rotational-joint :init :parent-link (elt rbleg 0) :child-link (elt rbleg 1) :name :rbleg-crotch-r :axis :-x))
     (setq jlrb2 (instance rotational-joint :init :parent-link (elt rbleg 1) :child-link (elt rbleg 2) :name :rbleg-crotch-p :axis :y :min -120 :max 120))
     (setq jlrb3 (instance rotational-joint :init :parent-link (elt rbleg 2) :child-link (elt rbleg 3) :name :rbleg-knee-p :axis :y :min 0))
     (setq jlrb4 (instance rotational-joint :init :parent-link (elt rbleg 3) :child-link (elt rbleg 4) :name :rbleg-ankle-p :axis :y))
     (setq jlrb5 (instance rotational-joint :init :parent-link (elt rbleg 4) :child-link (elt rbleg 5) :name :rbleg-ankle-r :axis :-x))

     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (append (list aroot-link) torso head larm rarm lfleg rfleg lbleg rbleg))
     (setq joint-list (list jc0 jc1 jh0 jh1
                            jal0 jal1 jal2 jal3 jal4 jal5 jal6
                            jar0 jar1 jar2 jar3 jar4 jar5 jar6
                            jllf0 jllf1 jllf2 jllf3 jllf4 jllf5
                            jlrf0 jlrf1 jlrf2 jlrf3 jlrf4 jlrf5
                            jllb0 jllb1 jllb2 jllb3 jllb4 jllb5
                            jlrb0 jlrb1 jlrb2 jlrb3 jlrb4 jlrb5))

     ;; add limbs
     (send self :add-limb :torso :links torso :end-coords torso-end-coords :root-link torso-root-link)
     (send self :add-limb :head :links head :end-coords head-end-coords :root-link head-root-link)
     (send self :add-limb :rarm :links rarm :end-coords rarm-end-coords :root-link rarm-root-link)
     (send self :add-limb :larm :links larm :end-coords larm-end-coords :root-link larm-root-link)
     (send self :add-limb :rfleg :links rfleg :end-coords rfleg-end-coords :root-link rfleg-root-link)
     (send self :add-limb :lfleg :links lfleg :end-coords lfleg-end-coords :root-link lfleg-root-link)
     (send self :add-limb :rbleg :links rbleg :end-coords rbleg-end-coords :root-link rbleg-root-link)
     (send self :add-limb :lbleg :links lbleg :end-coords lbleg-end-coords :root-link lbleg-root-link)

     ;;    These are for robot-model.
     (setq collision-avoidance-links (list aroot-link (elt torso 1) (elt larm 3) (elt rarm 3)))

     ;; set max torques
     (dolist (l (list aroot-link (elt torso 1) (elt head 1)))
       (let* ((valid-bodies (remove-if #'(lambda (x)
                                           (and (> (send x :volume) 0) (< (send x :volume) 0))) ;; nan check
                                       (send l :bodies))))
         (send l :centroid
               (if (= (length valid-bodies) 1)
                   (send (car valid-bodies) :centroid)
                 (scale (/ 1.0 (reduce #'+ (mapcar #'(lambda (x) (send x :volume)) valid-bodies)))
                        (reduce #'v+ (mapcar #'(lambda (x) (scale (send x :volume) (send x :centroid))) valid-bodies)))))
         ))
     (send-all (send self :joint-list) :max-joint-torque 7.5) ;; [Nm]
     ;; set sensors
     (setq force-sensors
           (mapcar #'(lambda (l)
                       (send (send self l :end-coords :parent) :assoc
                             (make-cascoords :coords (send self l :end-coords :copy-worldcoords)
                                             :name (format nil "~A-fsensor" (string-downcase l)))))
                   (list :rarm :larm :rfleg :lfleg :rbleg :lbleg)))
     (setq imu-sensors
           (list (let ((sen (make-cascoords
                             :worldpos (send (car (send self :torso :waist-p :child-link :bodies)) :centroid)
                             :name "torso-imusensor")))
                   (send (send self :torso :waist-p :child-link) :assoc sen)
                   sen)))
     (setq cameras
           (mapcar #'(lambda (pos name)
                       (let ((sen (instance camera-model :init (make-cylinder 10 20) :name name)))
                         (send sen :rotate pi/2 :y)
                         (send sen :rotate -pi/2 :z)
                         (send sen :locate (v+ pos (send (car (send (send self :head :neck-p :child-link) :bodies)) :centroid)) :world)
                         (send (send self :head :neck-p :child-link) :assoc sen)
                         sen))
                   (list (float-vector 0 30 0) (float-vector 0 -30 0))
                   (list "left-camera" "right-camera")))
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self))
  ;; Methods to define robot links
  (:make-root-link
   (waist-height waist-length waist-weight)
   (let ((bc0 (make-cube waist-length 200 waist-height)))
     (send bc0 :locate (float-vector 0 0 (+ (* -0.5 waist-height) -25)))
     (send bc0 :set-color :green)
     (instance bodyset-link :init (make-cascoords) :bodies (list bc0) :name :waist :weight waist-weight)))
  (:make-torso-links
   (torso-height torso-length torso-weight)
   (let ((bc1 (make-default-robot-link 0 50 :y :torso-link0))
	 (bc2 (make-cube torso-length 200 torso-height)))
     (send bc1 :locate #f(0 0 -12.5))
     (send bc2 :locate (float-vector 0 0 (* 0.5 torso-height)))
     (send bc2 :set-color :green)
     (setq bc2 (instance bodyset-link :init (make-cascoords :pos #f(0 0 -12.5)) :bodies (list bc2) :name :torso-link1 :weight torso-weight))
     (send bc1 :assoc bc2)
     (list (cons :links (list bc1 bc2))
           (cons :root-link bc1))))
  (:make-head-links
   (head-weight)
   (let ((bh0 (make-default-robot-link 0 50 :y :head-link0))
	 (bh2 (make-cube 120 100 150))
	 (bh2e (make-cylinder 10 30))
	 (bh1)
	 (end-coords))
     (send bh2 :locate #f(0 0 80))
     (send bh2 :set-color :green)
     (send bh2e :rotate pi/2 :y)
     (send bh2e :locate #f(60 0 70) :world)
     (send bh2e :set-color :green)
     (send bh2 :assoc bh2e)
     (setq bh1 (instance bodyset-link :init (make-cascoords) :bodies (list bh2 bh2e) :name :head-link1 :weight head-weight))
     (setq end-coords (make-cascoords :pos #f(60 0 100) :rpy (float-vector 0 pi/2 0)))
     (send bh1 :assoc end-coords)
     (send bh0 :assoc bh1)
     (list (cons :links (list bh0 bh1))
           (cons :root-link bh0)
           (cons :end-coords end-coords))))
  (:make-arm-links
   (l/r arm-radius upper-arm-length lower-arm-length shoulder-width hand-length
        upper-arm-weight lower-arm-weight hand-weight)
   (let ((ba1 (make-default-robot-link 0 arm-radius :y (read-from-string (format nil "~A-link0" l/r))))
	 (ba2 (make-default-robot-link 0 arm-radius :x (read-from-string (format nil "~A-link1" l/r))))
	 (ba3 (make-default-robot-link upper-arm-length arm-radius :z (read-from-string (format nil "~A-link2" l/r))))
	 (ba4 (make-default-robot-link lower-arm-length arm-radius :y (read-from-string (format nil "~A-link3" l/r))))
	 (ba5 (make-default-robot-link 0 arm-radius :z (read-from-string (format nil "~A-link4" l/r))))
	 (ba6 (make-default-robot-link 0 arm-radius :x (read-from-string (format nil "~A-link5" l/r))))
	 (ba7 (make-default-robot-link hand-length arm-radius :y (read-from-string (format nil "~A-link6" l/r))))
	 (end-coords))
     (send ba3 :weight upper-arm-weight)
     (send ba4 :weight lower-arm-weight)
     (send ba7 :weight hand-weight)
     (case l/r
       (:rarm
	(setq end-coords (make-cascoords))
	(send end-coords :locate (float-vector 0 0 (- hand-length)))
	(send end-coords :rotate pi/2 :y)
	(send ba7 :assoc end-coords))
       (:larm
	(setq end-coords (make-cascoords))
	(send end-coords :locate (float-vector 0 0 (- hand-length)))
	(send end-coords :rotate pi/2 :y)
	(send ba7 :assoc end-coords)))
     (send ba6 :assoc ba7)
     (send ba5 :assoc ba6)
     (send ba5 :translate (float-vector 0 0 (- lower-arm-length)) :world)
     (send ba4 :assoc ba5)
     (send ba4 :translate (float-vector 0 0 (- upper-arm-length)) :world)
     (send ba3 :assoc ba4)
     (send ba2 :assoc ba3)
     (send ba1 :assoc ba2)
     (list (cons :links (list ba1 ba2 ba3 ba4 ba5 ba6 ba7))
           (cons :root-link ba1)
           (cons :end-coords end-coords))))
  (:make-leg-links
   (l/r leg-radius upper-leg-length lower-leg-length ankle-length crotch-width foot-depth foot-width foot-thickness foot-offset
        upper-leg-weight lower-leg-weight foot-weight)
   (let* ((bl1 (make-default-robot-link 0 leg-radius :y (read-from-string (format nil "~A-link0" l/r))))
	  (bl2 (make-default-robot-link 0 leg-radius :x (read-from-string (format nil "~A-link1" l/r))))
	  (bl3 (make-default-robot-link (- upper-leg-length (/ leg-radius 2.0)) leg-radius :z (read-from-string (format nil "~A-link2" l/r))))
	  (bl4 (make-default-robot-link (- lower-leg-length (/ leg-radius 2.0)) leg-radius :y (read-from-string (format nil "~A-link3" l/r))))
	  (bl5 (make-default-robot-link 0 leg-radius :x (read-from-string (format nil "~A-link4" l/r))))
	  (bl6b (make-cube foot-depth foot-width foot-thickness))
	  (bl6)
	  (end-coords))
     (send bl3 :weight upper-leg-weight)
     (send bl4 :weight lower-leg-weight)
     (send bl6b :locate (float-vector foot-offset 0 (- ankle-length)))
     (send bl6b :set-color :green)
     (setq bl6 (make-default-robot-link ankle-length leg-radius :y (read-from-string (format nil "~A-link5" l/r)) (list bl6b)))
     (send bl6 :weight foot-weight)
     (case l/r
       (:rleg
	(setq end-coords (make-cascoords))
	(send end-coords :locate (float-vector 0 0 (- (+ ankle-length (/ foot-thickness 2.0)))))
	(send bl6 :assoc end-coords))
       (:lleg
	(setq end-coords (make-cascoords))
	(send end-coords :locate (float-vector 0 0 (- (+ ankle-length (/ foot-thickness 2.0)))))
	(send bl6 :assoc end-coords)))
     (send bl5 :assoc bl6)
     (send bl5 :translate (float-vector 0 0 (- lower-leg-length)) :world)
     (send bl4 :assoc bl5)
     (send bl4 :translate (float-vector 0 0 (- upper-leg-length)) :world)
     (send bl3 :assoc bl4)
     (send bl2 :assoc bl3)
     (send bl1 :assoc bl2)
     (list (cons :links (list bl1 bl2 bl3 bl4 bl5 bl6))
           (cons :root-link bl1)
           (cons :end-coords end-coords))))
  (:reset-pose ()
    (send self :angle-vector #f(0.0 0.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 10.0 20.0 0.0 -20.0 10.0 0.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0 0.0 0.0 -15.0 30.0 -15.0 0.0)))
  )

(defmethod sample-centaur-robot
  ;; user-defined joint
  (:torso-waist-y (&rest args) (forward-message-to jc0 args))
  (:torso-waist-p (&rest args) (forward-message-to jc1 args))
  (:head-neck-y (&rest args) (forward-message-to jh0 args))
  (:head-neck-p (&rest args) (forward-message-to jh1 args))
  (:larm-shoulder-p (&rest args) (forward-message-to jal0 args))
  (:larm-shoulder-r (&rest args) (forward-message-to jal1 args))
  (:larm-shoulder-y (&rest args) (forward-message-to jal2 args))
  (:larm-elbow-p (&rest args) (forward-message-to jal3 args))
  (:larm-wrist-y (&rest args) (forward-message-to jal4 args))
  (:larm-wrist-r (&rest args) (forward-message-to jal5 args))
  (:larm-wrist-p (&rest args) (forward-message-to jal6 args))
  (:rarm-shoulder-p (&rest args) (forward-message-to jar0 args))
  (:rarm-shoulder-r (&rest args) (forward-message-to jar1 args))
  (:rarm-shoulder-y (&rest args) (forward-message-to jar2 args))
  (:rarm-elbow-p (&rest args) (forward-message-to jar3 args))
  (:rarm-wrist-y (&rest args) (forward-message-to jar4 args))
  (:rarm-wrist-r (&rest args) (forward-message-to jar5 args))
  (:rarm-wrist-p (&rest args) (forward-message-to jar6 args))
  (:lfleg-crotch-y (&rest args) (forward-message-to jllf0 args))
  (:lfleg-crotch-r (&rest args) (forward-message-to jllf1 args))
  (:lfleg-crotch-p (&rest args) (forward-message-to jllf2 args))
  (:lfleg-knee-p (&rest args) (forward-message-to jllf3 args))
  (:lfleg-ankle-p (&rest args) (forward-message-to jllf4 args))
  (:lfleg-ankle-r (&rest args) (forward-message-to jllf5 args))
  (:rfleg-crotch-y (&rest args) (forward-message-to jlrf0 args))
  (:rfleg-crotch-r (&rest args) (forward-message-to jlrf1 args))
  (:rfleg-crotch-p (&rest args) (forward-message-to jlrf2 args))
  (:rfleg-knee-p (&rest args) (forward-message-to jlrf3 args))
  (:rfleg-ankle-p (&rest args) (forward-message-to jlrf4 args))
  (:rfleg-ankle-r (&rest args) (forward-message-to jlrf5 args))
  (:lbleg-crotch-y (&rest args) (forward-message-to jllb0 args))
  (:lbleg-crotch-r (&rest args) (forward-message-to jllb1 args))
  (:lbleg-crotch-p (&rest args) (forward-message-to jllb2 args))
  (:lbleg-knee-p (&rest args) (forward-message-to jllb3 args))
  (:lbleg-ankle-p (&rest args) (forward-message-to jllb4 args))
  (:lbleg-ankle-r (&rest args) (forward-message-to jllb5 args))
  (:rbleg-crotch-y (&rest args) (forward-message-to jlrb0 args))
  (:rbleg-crotch-r (&rest args) (forward-message-to jlrb1 args))
  (:rbleg-crotch-p (&rest args) (forward-message-to jlrb2 args))
  (:rbleg-knee-p (&rest args) (forward-message-to jlrb3 args))
  (:rbleg-ankle-p (&rest args) (forward-message-to jlrb4 args))
  (:rbleg-ankle-r (&rest args) (forward-message-to jlrb5 args))
  )
