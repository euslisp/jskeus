;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: irtcollada.l 664 2011-06-24 05:57:02Z snozawa $
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
;;; Permission to use this software for educational, research
;;; and non-profit purposes, without fee, and without a written
;;; agreement is hereby granted to all researchers working on
;;; the IRT project at the University of Tokyo, provided that the
;;; above copyright notice remains intact.  
;;;

(require :irtmodel)

(unless (find-package "COLLADA") (make-package "COLLADA"))
(in-package "COLLADA")

;;(export '(convert-irtmodel-to-collada))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; xml utilities
;; implementation reference is: 
;; http://homepage1.nifty.com/blankspace/scheme/nsx.html
;; really thanks to nnakamura!
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *collada-print-customize-symbols* '(collada |profile_COMMON|))

(defun symbol->string (sym)
  (if (find sym *collada-print-customize-symbols*)
      (string sym)
    (string-downcase (string sym))))

(defun ->string (val)
  (format nil "~s" val))

(defun string-append (&rest args)
  (apply #'concatenate string args))

(defun make-attr (l ac)
  (if (null l)
      ac
    (make-attr
     (cdr l) (string-append ac " " (symbol->string (caar l))
                            "=\"" (if (stringp (cadar l))
                                      (cadar l)
                                    (->string (cadar l)))
                            "\""))))

(defun make-xml (x bef aft)
  (cond ((stringp x) (append bef (list x) aft))
        ((and (listp x) (symbolp (car x)) (not (null x)))
         (let ((tag (symbol->string (car x))))
           (if (and (not   (null (cdr x)))
                    (listp (cadr x))
                    (eq (caadr  x) '@))
               (make-xml (cddr x)
                         (append bef (list 'BO tag (make-attr (cdadr x) "") 'BC))
                         (append     (list 'EO tag 'EC) aft))
             (make-xml (cdr x)
                       (append bef (list 'BO tag 'BC))
                       (append     (list 'EO tag 'EC) aft)))))
        ((and (not (null x))
              (listp x)) (make-xml ()
                             (make-xml (car x) bef ())
                             (make-xml (cdr x) () aft)))
        (t (append bef aft))))

(defun sxml->xml (sxml)
  ;; main
  (make-xml sxml () ()))

;; utility
(defun xml-output-to-string-stream (ss l)
  (dolist (ll l)
    (if (stringp ll)
        (format ss ll)
      (format ss
              (case ll
                (BO    "<")
                (EO    "</")
                ((BC EC) ">~%"))))))

(defun cat-normal (l s)
  (cond ((null l) s)
        ((stringp (car l))
         (cat-normal (cdr l) (string-append s (car l))))
        (t (cat-normal (cdr l)
                          (string-append s (case (car l)
                                             (BO    "<")
                                             (EO    "</")
                                             ((BC EC) ">~%")))))))
;; utility
(defun cat-clark (l s i)
  (cond ((null l) s)
        ((stringp (car l))
         (cat-clark (cdr l) (string-append s (car l)) i))
        (t
         (case (car l)
           (BO    (cat-clark (cdr l) (string-append s "<" ) (+ i 1)))
           (EO    (cat-clark (cdr l) (string-append s "</") (- i 1)))
           ((BC EC) (cat-clark (cdr l)
                               (string-append s "~%" (make-string i #\ ) ">") i))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; eusmodel-description.l
;;
;; this file provides an interface to euslisp model IForm.
;; in eusmodel_converter, all of the euslisp models (subclass of robot-frame),
;; will be converted into model-iform. model-iform is a simple associated-list.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; model-iform spec:
;;  model-iform := (<link-specs> . <joint-specs>)
;;  <link-specs> := (<link-spec> ...)
;;  <joint-specs> := (<joint-spec> ...)
;;  <link-spec> := ((:name . <string>)
;;                  (:origin . <coordinates>)
;;                  (:mesh . <mesh-spec>)
;;                  (:mass_frame . <coordinates>)
;;                  (:mass . <float>)
;;                  (:inertia . <float-vector>))
;;  <mesh-spec> := (<triangle faceset>)
;;   (all of the vertices has relative coordinates from the origin of the link)
;;  <joint-spec> := ((:name . <string>)
;;                   (:id . <integer>)
;;                   (:type . <joint-type>)
;;                   (:limit . <limit-spec>)
;;                   (:axis . <float-vector>)
;;                   (:parent . <string>)
;;                   (:child . <string>))
;;  <joint-type> := :rotational-joint|linear-joint
;;  <limit-spec> := ((:lower . <float>)
;;                   (:upper . <float>))

(defun eusmodel-description (model)
  "convert a `model' to eusmodel-description"
  ;; FIXME: make robots reset
  (let ((links (send model :links))
        (joints (send model :joint-list)))
    ;; verification
    (verificate-unique-strings (mapcar #'string (send-all links :name)))
    (verificate-unique-strings (mapcar #'string (send-all joints :name)))
    (dotimes (i (length joints)) (setf (get (elt joints i) :jointid) i)) ;; set :jointid to joint
    (list (eusmodel-link-specs links)
          (eusmodel-joint-specs joints)
          (eusmodel-endcoords-specs model))))

(defun verificate-unique-strings (names)
  (if (not (= (length names)
              (length (remove-duplicates names :test #'string=))))
      (error "NOT UNIQUE NAMES ~A" names)
    t))

(defun eusmodel-link-specs (links)
  "convert `links' to <link-specs>"
  (mapcar #'eusmodel-link-spec links))

(defun eusmodel-joint-specs (joints)
  "convert `joints' to <joint-specs>"
  (mapcar #'eusmodel-joint-spec joints))

(defun eusmodel-link-spec (link)
  (let* ((ac (link . user::acentroid))
         (it (send link :inertia-tensor))
         (rt-cds (let ((lnk link)) (while (send lnk :parent) (setq lnk (send lnk :parent)))
                      (send (car (send lnk :links)) :worldcoords)))
         (mframe (send link :copy-worldcoords))
         rot inertia)
    ;; centroid frame on rootlink coordinate
    (send mframe :translate ac)
    (setq mframe (send rt-cds :transformation mframe))
    ;; calc principal axis
    (setq rot (search-minimum-rotation-matrix (car (user::sv-decompose it))))
    (setq inertia (m* (m* (transpose rot) it) rot))
    (setq inertia (scale 1.0e-9 (float-vector (aref inertia 0 0) (aref inertia 1 1) (aref inertia 2 2))))
    ;; rotate inertia frame
    (send mframe :rotate rot)
    (list (cons :name (string (send link :name)))
          (cons :origin                   ;parent -> child transformation
                (let ((parent (send link :parent)))
                  (send parent :transformation link)))
          (cons :mesh (eusmodel-mesh-spec link))
          (cons :mass_frame mframe)
          (cons :mass (/ (send link :weight) 1000.0))
          (cons :inertia inertia)
          )))

(defun eusmodel-mesh-spec (link)
  ;; right?
  (let ((bs (send link :bodies)))
    (mapcar #'(lambda (b)
                (let ((trimesh
                       (instance geo::faceset :init
                                 :faces
                                 (send (geo::body-to-triangles b nil)
                                       :faces))))
                  (send trimesh :set-color (get b :face-color))
                  ;; move trimesh to origin...
                  (send trimesh
                        :move-to (send (send b :copy-worldcoords)
                                       :inverse-transformation))
                  trimesh))
            bs)))

(defun eusmodel-joint-spec (_joint)
  (list (cons :name (string (send _joint :name)))
        (cons :type (cond
                     ((derivedp _joint user::rotational-joint)
                      :rotational-joint)
                     ((derivedp _joint user::linear-joint)
                      :linear-joint)))
        (cons :id (get _joint :jointid))
        (cons :limit (eusmodel-limit-spec _joint))
        (cons :axis (case (_joint . user::axis)
                      (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                      (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                      (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                      (t (if (listp (_joint . user::axis))
                             ;; 6dof joint...
                             #f(0 0 0)
                             (_joint . user::axis)))))
        (cons :parent (string (send (send _joint :parent-link) :name)))
        (cons :child (string (send (send _joint :child-link) :name)))))

(defun eusmodel-limit-spec (_joint)
  (list (cons :upper (deg2rad (send _joint :max-angle)))
        (cons :lower (deg2rad (send _joint :min-angle)))))


(defun eusmodel-endcoords-specs (model)
  (labels ((get-links
            (link)
            (if (find-method model link)
                (let ((tmp (send model link)))
                  (if (and tmp (send model link :end-coords)
                           (send (send model link :end-coords) :parent))
                      (let ((link-list
                             (send model :link-list
                                   (send (send model link :end-coords) :parent)
                                   (car tmp))))
                        (cons (send (car tmp) :parent-link)
                              link-list)))))))
    (let ((head (get-links :head))
          (larm (get-links :larm))
          (rarm (get-links :rarm))
          (lleg (get-links :lleg))
          (rleg (get-links :rleg)))
      (list (cons "head"
                  (mapcar #'string (send-all head :name)))
            (cons "larm"
                  (mapcar #'string (send-all larm :name)))
            (cons "rarm"
                  (mapcar #'string (send-all rarm :name)))
            (cons "lleg"
                  (mapcar #'string (send-all lleg :name)))
            (cons "rleg"
                  (mapcar #'string (send-all rleg :name)))))))

;; accessor
(defun eusmodel-link-description (description)
  (car description))

(defun eusmodel-joint-description (description)
  (cadr description))

(defun eusmodel-endcoords-description (description)
  (caddr description))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; collada interface
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun eusmodel-description->collada (name description &key (scale 0.001))
  "convert eusmodel-descrption to collada sxml"
  (warn "start converting to collada sxml~%")
  `(collada
    (@ (version "1.5.0")
       (xmlns "http://www.collada.org/2008/03/COLLADASchema")
       ;; (xmlns:xsi "http://www.w3.org/2001/XMLSchema-instance")
       ;; (xmlns:math "http://www.w3.org/1998/Math/MathML")
       ;; (xsi:schemaLocation "http://www.collada.org/2008/03/COLLADASchema http://www.khronos.org/files/collada_schema_1_5")
       )
    (asset
     (contributor (authoring_tool "Eus2Collada"))
     (unit (@ (meter ,scale)
              (name "millimeter")))
     (up_axis "Z_UP"))
    ,(eusmodel-description->collada-library-visual-scenes name description)
    ,(eusmodel-description->collada-library-joints description)
    ,(links-description->collada-library-effects (eusmodel-link-description description))
    ,(links-description->collada-library-materials (eusmodel-link-description description))
    ,(eusmodel-description->collada-library-kinematics-models name description)
    ,(eusmodel-description->collada-library-kinematics-scenes name description)
    ,(eusmodel-description->collada-library-physics-scenes name description)
    ,(eusmodel-description->collada-library-physics-models name description)
    ,(eusmodel-description->collada-library-geometries description)
    ,(eusmodel-description->collada-library-articulated-systems description name)
    ;; default scene...
    ,(eusmodel-description->collada-scene description)))

(defun setup-collada-filesystem (obj-name base-dir)
  (if (not (eq (probe-file base-dir) :directory))
      (error "~A must be a directory" base-dir)))

(defun range2 (n)
  (if (= n 0)
      '(0)
    (append (range2 (1- n)) (list n))))

(defun eus2collada (obj full-root-dir
                        &key
                        (scale 0.001)
                        (file-suffix ".dae"))
  (let ((name (send obj :name)))
    (unless name
      (error "You need to specify name"))
    ;; support body or bodyset
    (if (or (eq (send (class obj) :name) 'geo::body) (eq (send (class obj) :name) 'geo::bodyset))
        (setq obj (let ((rb (instance user::cascaded-link :init :name (send obj :name))))
                    (setq (rb . user::links) (list (instance user::bodyset-link :init (user::make-cascoords) :bodies (list obj))))
                    (send rb :assoc (car (send rb :links)))
                    (send rb :init-ending)
                    rb)))
    (let ((descrpition (eusmodel-description obj)))
      (let ((sxml (eusmodel-description->collada
                   name descrpition :scale scale)))
        (setup-collada-filesystem name full-root-dir) ;only check directory existence
        (let ((xml-file (format nil "~A/~A.dae" full-root-dir name))
              (xml (sxml->xml sxml)))
          (with-open-file (f xml-file :direction :output)
            (format f "<?xml version=\"1.0\"?>~%")
            (xml-output-to-string-stream f xml)
            nil))))))


(defun collada-node-id (link-descrption)
  (format nil "~A.node" (cdr (assoc :name link-descrption))))

(defun collada-node-name (link-descrption)
  (format nil "~A_node" (cdr (assoc :name link-descrption))))

(defun links-description->collada-library-materials (links-desc)
  `(library_materials
    (@ (id "libmat"))
    ,@(mapcar #'link-description->collada-materials links-desc)))

(defun link-description->collada-materials (link-desc)
  (let* ((meshes (cdr (assoc :mesh link-desc)))
         (id-base (format nil "~A.geometry" (cdr (assoc :name link-desc))))
         (effect-base (format nil "~A.effect" (cdr (assoc :name link-desc))))
         (ids (range2 (length meshes))))
    (mapcar #'(lambda (i)
                (mesh-description->collada-material
                 (format nil "~A.~A.material" id-base i)
                 (format nil "~A.~A" effect-base i)))
            ids)))

(defun mesh-description->collada-material (mat effect)
  `(material
    (@ (id ,mat))
    (instance_effect
     (@ (url ,(format nil "#~A" effect))))))

(defun links-description->collada-library-effects (links-desc)
  `(library_effects
    (@ (id "libeffect"))
    ,@(mapcar #'link-description->collada-effects links-desc)))

(defun link-description->collada-effects (link-desc)
  (let* ((meshes (cdr (assoc :mesh link-desc)))
         (id-base (format nil "~A.effect" (cdr (assoc :name link-desc))))
         (ids (range2 (length meshes))))
    (mapcar #'(lambda (x i)
                (mesh-description->collada-effect
                 x (format nil "~A.~A" id-base i)))
            meshes ids)))

(defun mesh-description->collada-effect (mesh id)
  (unless (get mesh :face-color)
    (error ";; please set face-color for meshes ~A~%" mesh))
  `(effect
    (@ (id ,id))
    (|profile_COMMON|
     (technique
      (@ (sid "common"))
      (phong
       (ambient
        (color
         ,(format nil "~A ~A ~A 1.0"
                  (elt (get mesh :face-color) 0)
                  (elt (get mesh :face-color) 1)
                  (elt (get mesh :face-color) 2))))
       (diffuse
        (color
         ,(format nil "~A ~A ~A 1.0"
                  (elt (get mesh :face-color) 0)
                  (elt (get mesh :face-color) 1)
                  (elt (get mesh :face-color) 2)))))))))

(defun matrix->collada-string (mat)
  (let ((dims (array-dimensions mat))
        (ss (make-string-output-stream)))
    (dotimes (i (car dims))
      (dotimes (j (cadr dims))
        (format ss "~A " (aref mat i j))))
    (format ss "~%")
    (get-output-stream-string ss)))

(defun find-parent-liks-from-link-description (target-link desc)
  (let ((j (find-joint-from-link-description target-link (eusmodel-link-description desc))))
    (if (null j)
        nil
      (let* ((parent-link-name (cdr (assoc :parent j)))
             (parent-link (find-link-from-links-description
                           parent-link-name (eusmodel-link-description desc))))
        (append (find-parent-liks-from-link-description parent-link desc)
                (list parent-link))))))

(defun eusmodel-description->collada-node-transformations (target-link desc)
  (let ((parent-links
         (append (find-parent-liks-from-link-description target-link desc)
                 (list target-link))))
    (remove-if #'null
               (mapcar #'(lambda (l)
                           (let ((j (find-joint-from-link-description
                                     l (eusmodel-joint-description desc))))
                             (if j
                                 `((rotate
                                    (@ (sid
                                        ,(format nil "~A_axis0" (cdr (assoc :name j)))))
                                    "1 0 0 0")))))
                       parent-links))))

(defun eusmodel-description->collada-node (target-link desc)
  (let* ((joint-descriptions (eusmodel-joint-description desc))
         (link-descriptions (eusmodel-link-description desc))
         (j (find-joint-from-link-description target-link (eusmodel-joint-description desc)))
         (meshes (cdr (assoc :mesh target-link)))
         (origin (cdr (assoc :origin target-link)))
         (pos (position-if #'(lambda (l)
                               (string= (cdr (assoc :name target-link))
                                        (cdr (assoc :name l))))
                           (eusmodel-link-description desc))))
  `(node
    (@ (id ,(collada-node-id target-link))
       (name ,(collada-node-name target-link))
       (sid ,(format nil "node~D" pos)))
    ,@(eusmodel-description->collada-node-transformations target-link desc)
    ,@(mapcar
       #'(lambda (i)
           `(instance_geometry
             (@ (url
                 ,(format nil "#~A.~A"
                          (collada-geometry-id-base target-link)
                          i)))
             (bind_material
              (technique_common
               (instance_material
                (@ (symbol ,(format nil "~A.~A.material"
                                    (collada-geometry-id-base target-link) i))
                   (target ,(format nil "#~A.~A.material"
                                    (collada-geometry-id-base target-link) i))))))))
       (enum-integer-list (length meshes))))))

(defun eusmodel-description->collada-library-visual-scenes (name desc)
  `(library_visual_scenes
    (@ (id "vscenes"))
    (visual_scene
     (@ (id "vscene")
        (name "Eus2Collada Visual Scene"))
     (node
      (@ (id "visual1")
         (sid "visual1")
         (name ,name))
      ,@(mapcar #'(lambda (l)
                    (eusmodel-description->collada-node l desc))
                (eusmodel-link-description desc))))))

(defun mesh-description->instance-material (s)
  `(instance_material
    (@ (symbol ,s)
       (target ,(format nil "#~A" s)))))

(defun link-description->collada-bind-material (link)
  (let* ((meshes (cdr (assoc :mesh link)))
         (name (cdr (assoc :name link)))
         (ids (enum-integer-list (length meshes))))
    ;; material
    `(bind_material
      (technique_common
       ,@(mapcar #'(lambda (n)
                     (mesh-description->instance-material
                      (format nil "~A.geometry.~A.material" name n)))
                 ids)))))

(defun eusmodel-description->collada-library-kinematics-scenes (name desc)
  `(library_kinematics_scenes
    (@ (id "kscenes"))
    (kinematics_scene
     (@ (id "kscene")
        (name "Eus2Collada Kinematics Scene"))
     (instance_articulated_system
      (@ (sid "robot1_motion_inst")
         (url "#robot1_motion")
         (name ,name))
      (newparam
       (@ (sid "kscene_kmodel1_inst"))
       "<SIDREF>robot1_motion/robot1_motion.kmodel1_inst</SIDREF>")
      ;; joint axis
      ,@(mapcar #'(lambda (j)
                    (let ((joint-name (cdr (assoc :name j))))
                      `(newparam
                        (@ (sid ,(format nil "kscene_kmodel1_inst.~A_axis0" joint-name)))
                        ,(format nil "<SIDREF>robot1_motion/robot1_motion.kmodel1_inst.inst_~A_axis0</SIDREF>"
                                 joint-name))))
                (eusmodel-joint-description desc))
      ;; joint axis value
      ,@(mapcar #'(lambda (j)
                    (let ((joint-name (cdr (assoc :name j))))
                      `(newparam
                        (@ (sid ,(format nil "kscene_kmodel1_inst.~A.value" joint-name)))
                        ,(format nil "<SIDREF>robot1_motion/robot1_motion.kmodel1_inst.inst_~A_value</SIDREF>"
                                 joint-name))))
                (eusmodel-joint-description desc))))))

(defun eusmodel-description->collada-library-kinematics-models (name desc)
  `(library_kinematics_models
    (@ (id "kmodels"))
    ,(eusmodel-description->collada-kinematics-model name desc)))

(defun eusmodel-description->collada-kinematics-model (name desc)
  (let ((joints-desc (eusmodel-joint-description desc)))
    `(kinematics_model
      (@ (id "kmodel1")
         (name ,name)) ;the name of robot...?
      (technique_common
       ,@(joints-description->collada-instance-joints joints-desc)
       ,@(eusmodel-description->collada-links desc)))))

;; physics
(defun eusmodel-description->collada-library-physics-scenes (name desc)
  `(library_physics_scenes
    (@ (id "pscenes"))
    (physics_scene
     (@ (id "pscene")
        (name "Eus2Collada Physics Scene"))
     (instance_physics_model
      (@ (url "#pmodel1")
         (sid "pmodel1")
         (parent "#visual1"))
      ,(let* ((links (eusmodel-link-description desc))
              (ids (enum-integer-list (length links))))
         (mapcar #'(lambda (i l)
                     `(instance_rigid_body
                       (@ (body ,(format nil "rigid~a" i))
                          (target ,(format nil "#~a" (collada-node-id l))))))
                 ids links))))))

(defun eusmodel-description->collada-library-physics-models (name desc)
  `(library_physics_models
    (@ (id "pmodels"))
    (physics_model
     (@ (id "pmodel1")
        (name ,name))
     ,@(mapcar
        #'(lambda (i ld)
            (let* ((link-name (cdr (assoc :name ld))) ;; link-name without _link
                   (id (format nil "rigid~a" i)) ;; id
                   (sid (format nil "rigid~a" i)) ;; sid
                   (meshes (cdr (assoc :mesh ld))))
              `(rigid_body
                (@ (id ,(format nil "~a" id))
                   (sid ,(format nil "~a" sid))
                   (name ,(format nil "~a_link" link-name)))
                (technique_common
                 (mass ,(format nil "~A" (cdr (assoc :mass ld))))
                 (mass_frame
                  (translate
                   ,(float-vector->collada-string
                     (send (cdr (assoc :mass_frame ld)) :worldpos)))
                  (rotate
                   ,(float-vector->collada-string
                     (matrix->collada-rotate-vector
                      (send (cdr (assoc :mass_frame ld)) :worldrot)))))
                 (inertia
                  ,(float-vector->collada-string (cdr (assoc :inertia ld))))
                 ,@(mapcar
                    #'(lambda (ii)
                        `(shape
                          (instance_geometry
                           (@ (url ,(format nil "#~a.geometry.~a" link-name ii)
                                   )))
                          (translate "0 0 0")
                          (rotate "0 0 1 0")))
                    (enum-integer-list (length meshes)))))))
        (enum-integer-list (length (eusmodel-link-description desc)))
        (eusmodel-link-description desc)))))

;; returns the name of root link
;; TODO: this function does not work. we need to create a tree
;; of robot models and find the root of the tree.
(defun find-root-link-from-joints-description (joints-description)
  ;; build joint tree using cascaded-coords
  (let ((coords-set nil))
    (dolist (j joints-description)
      (let ((c (geo::make-cascoords :name (cdr (assoc :name j)))))
        (setf (get c :parent-link) (cdr (assoc :parent j)))
        (setf (get c :child-link) (cdr (assoc :child j)))
        (push c coords-set)))
    ;; resolve tree
    (dolist (c coords-set)
      (let ((parent-link (get c :parent-link)))
        (let ((parent-coords
               (find parent-link
                     coords-set
                     :test #'string=
                     :key #'(lambda (c) (get c :child-link)))))
          (unless parent-coords
            ;; this is ROOT!
            (return-from find-root-link-from-joints-description
              parent-link)))))))

(defun find-link-from-links-description (name links-description)
  (find name links-description
        :test #'string= :key #'(lambda (l) (cdr (assoc :name l)))))

(defun eusmodel-description->collada-links (description)
  (let ((joints-description (eusmodel-joint-description description))
        (links-description (eusmodel-link-description description)))
    (let ((root-link-description
           (find-link-from-links-description
            (find-root-link-from-joints-description joints-description)
            links-description)))
      (eusmodel-description->collada-links-tree
       root-link-description links-description joints-description))))

(defun find-joint-from-link-description (target joints)
  (find (cdr (assoc :name target))
        joints
        :test #'string=
        :key #'(lambda (j) (cdr (assoc :child j)))))

(defun find-child-link-descriptions (parent links joints)
  (let ((parented-joints
         (remove-if-not #'(lambda (j) (cdr (assoc :parent j))) joints)))
    (let ((joints-has-parent
           (remove-if-not
            #'(lambda (j)
                (string= (cdr (assoc :parent j))
                         (cdr (assoc :name parent))))
            joints)))
      (let ((child-link-names
             (mapcar #'(lambda (j)
                         (cdr (assoc :child j))) joints-has-parent)))
        (mapcar #'(lambda (n)
                    (find-link-from-links-description n links))
                child-link-names)))))

(defun matrix->collada-rotate-vector (mat)
  "convert a rotation matrix to axis-angle."
  (let ((q (user::matrix2quaternion mat)))
    (let ((qv (normalize-vector (float-vector (elt q 1) (elt q 2) (elt q 3))))
          (qa (elt q 0)))
      (float-vector (elt qv 0)
                    (elt qv 1)
                    (elt qv 2)
                    (rad2deg (* 2.0 (atan 
                                          (sqrt (+ (expt (elt q 1) 2)
                                                   (expt (elt q 2) 2)
                                                   (expt (elt q 3) 2)))
                                          qa)))))))

(defun eusmodel-description->collada-library-articulated-systems (desc name)
  `(library_articulated_systems
    (@ (id "asystems"))
    (articulated_system
     (@ (id "robot1_motion"))
     (motion
       (instance_articulated_system
        (@ (url "#robot1_kinematics"))
        ;; here we bind parameters...
        (newparam
         (@ (sid "robot1_motion.kmodel1_inst"))
         "<SIDREF>robot1_kinematics/robot1_motion_kmodel1_inst</SIDREF>")
        ,@(mapcar
           #'(lambda (j)
               (let ((joint-name (cdr (assoc :name j))))
                 `(newparam
                   (@ (sid ,(format nil "robot1_motion.kmodel1_inst.inst_~A_axis0" joint-name)))
                   ,(format nil "<SIDREF>robot1_kinematics/robot1_motion_kmodel1_inst.~A_axis0</SIDREF>"
                            joint-name)
                   )))
           (eusmodel-joint-description desc))
        ,@(mapcar
           #'(lambda (j)
               (let ((joint-name (cdr (assoc :name j))))
                 `(newparam
                   (@ (sid ,(format nil "robot1_motion.kmodel1_inst.inst_~A_value" joint-name)))
                   ,(format nil "<SIDREF>robot1_motion_kmodel1_inst.~A.value</SIDREF>"
                            joint-name))))
           (eusmodel-joint-description desc)))
       (technique_common
        ,@(mapcar
           #'(lambda (j)
               (let ((joint-name (cdr (assoc :name j))))
                 `(axis_info
                   (@ (axis ,(format nil "robot1_kinematics/~A_info" joint-name))))))
           (eusmodel-joint-description desc))))
     ;; for openrave profile
     ,@(mapcar
        #'(lambda (e)
            (eusmodel-endcoords-description->openrave-manipulator e desc))
        (eusmodel-endcoords-description desc)))
    (articulated_system
     (@ (id "robot1_kinematics"))
     (kinematics
      (instance_kinematics_model
       (@ (url "#kmodel1")
          (sid "kmodel1_inst"))
       (newparam
        (@ (sid "robot1_motion_kmodel1_inst"))
        "<SIDREF>robot1_kinematics/kmodel1_inst</SIDREF>")
       ;; joint axis
       ,@(mapcar #'(lambda (j)
                     (let ((joint-name (cdr (assoc :name j))))
                       `(newparam
                         (@ (sid ,(format nil "robot1_motion_kmodel1_inst.~A_axis0" joint-name)))
                         ,(format nil
                                  "<SIDREF>robot1_kinematics/kmodel1_inst/~A/axis0</SIDREF>~%" joint-name))))
                 (eusmodel-joint-description desc))
       ;; joint axis value
       ,@(mapcar #'(lambda (j)
                     (let ((joint-name (cdr (assoc :name j))))
                       `(newparam
                         (@ (sid ,(format nil "robot1_motion_kmodel1_inst.~A.value" joint-name)))
                         (float "0"))))
                 (eusmodel-joint-description desc)))
      (technique_common
       ,@(mapcar
          #'(lambda (j)
              (let ((joint-name (cdr (assoc :name j))))
                `(axis_info
                  (@ (axis ,(format nil "kmodel1/~A/axis0" joint-name)) ;??
                     (sid ,(format nil "~A_info" joint-name))))))
          (eusmodel-joint-description desc)))))
    ))

(defun eusmodel-endcoords-description->openrave-manipulator (end-coords description)
  (let ((name (car end-coords))
        (origin-link-name (car (cdr end-coords)))
        (tip-link-name (car (last (cdr end-coords)))))
    `(extra
      (@ (name ,name)
         (type "manipulator"))
      (technique
       (@ (profile "OpenRAVE"))
       (frame_origin
        (@ (link ,(format nil "kmodel1/~A_link" origin-link-name))))
       (frame_tip
        (@ (link ,(format nil "kmodel1/~A_link" tip-link-name))))))))

(defun eusmodel-description->collada-links-tree (target links joints)
  ;; recursive function call!
  (let ((child-links
         (find-child-link-descriptions target links joints)))
    (let ((name (cdr (assoc :name target))))
      ;; output xml of the target and call itself
      `(link
        (@ (sid ,(format nil "~A_link" name))
           (name ,(format nil "~A_link" name)))
        ,@(mapcar
           #'(lambda (l)
               (let ((link-coords (cdr (assoc :origin l)))
                     (j (find-joint-from-link-description l joints)))
                 `((attachment_full
                    (@ (joint ,(format nil
                                       "kmodel1/~A"
                                       (cdr (assoc :name j)))))
                    (translate
                    ,(float-vector->collada-string
                      (send link-coords :worldpos)))
                    (rotate
                     ,(float-vector->collada-string
                       (matrix->collada-rotate-vector
                        (send link-coords :worldrot))))
                    ,(eusmodel-description->collada-links-tree l links
                                                               joints))
                   ))) child-links)))))

(defun joints-description->collada-instance-joints (joints-desc)
  (mapcar #'joint-description->collada-instance-joint joints-desc))

(defun joint-description->collada-instance-joint (joint-desc)
  (let ((name (cdr (assoc :name joint-desc))))
    `(instance_joint
      (@ (url ,(format nil "#~A" name))
         (sid ,name)))))

(defun eusmodel-description->collada-library-joints (description)
  `(library_joints
    (@ (id "libjoints"))
    ,@(joints-description->collada-joints (eusmodel-joint-description description))))

(defun joints-description->collada-joints (joints-description)
  (mapcar #'joint-description->collada-joint joints-description))

(defun collada-joint-id (joint-description)
  (format nil "~A" (cdr (assoc :name joint-description))))


(defun joint-description->collada-joint (joint-description)
  (let ((id (collada-joint-id joint-description))
        (name (cdr (assoc :name joint-description))))
    `(joint
      (@ (id ,id)
         (name ,(format nil "~A" name))
         (sid ,(format nil "jointsid~A" (cdr (assoc :id joint-description)))))
      ,(cond ((eq (cdr (assoc :type joint-description)) :linear-joint)
              (linear-joint-description->collada-joint joint-description))
             ((eq (cdr (assoc :type joint-description)) :rotational-joint)
              (rotational-joint-description->collada-joint joint-description))
             (t (error "~A is not supported for collada converter"
                       (cdr (assoc :type joint-description))))))))

(defun linear-joint-description->collada-joint (joint-description)
  (let ((limit-description (cdr (assoc :limit joint-description))))
    `(prismatic
      (@ (sid "axis0"))
      (axis ,(float-vector->collada-string
              (cdr (assoc :axis joint-description))))
      (limits
       (min
        ,(format nil "~A" (rad2deg (cdr (assoc :lower limit-description)))))
       (max
        ,(format nil "~A" (rad2deg (cdr (assoc :upper limit-description)))))))))


(defun rotational-joint-description->collada-joint (joint-description)
  (let ((limit-description (cdr (assoc :limit joint-description))))
  `(revolute
    (@ (sid "axis0"))
    (axis ,(float-vector->collada-string
            (cdr (assoc :axis joint-description))))
    (limits
     (min
      ,(format nil "~A" (rad2deg (cdr (assoc :lower limit-description)))))
     (max
      ,(format nil "~A" (rad2deg (cdr (assoc :upper limit-description)))))))))

(defun eusmodel-description->collada-scene (description)
  `(scene
    (instance_physics_scene
     (@ (url "#pscene")))
    (instance_visual_scene
     (@ (url "#vscene")))
    (instance_kinematics_scene
     (@ (url "#kscene"))
     (bind_kinematics_model
      (@ (node "visual1/node0"))
      (param
       "kscene_kmodel1_inst"))
     ;; bind_joint_axis
      ,@(mapcar
         #'(lambda (j)
             ;; the initial joint angles
             `(bind_joint_axis
               (@ (target ,(format nil "~A.node/~A_axis0"
                                   (cdr (assoc :child j))
                                   (cdr (assoc :name j)))))
               (axis
                (param
                   ,(format
                     nil
                     "kscene_kmodel1_inst.~A_axis0"
                     (cdr (assoc :name j)))))
               (value
                (param
                 ,(format nil
                          "kscene_kmodel1_inst.~A.value" (cdr (assoc :name j)))))))
         (eusmodel-joint-description description)))))

(defun eusmodel-description->collada-library-geometries (description)
  `(library_geometries
    (@ (id "libgeom"))
    ,@(links-description->collada-geometries (eusmodel-link-description description))))

(defun collada-geometry-id-base (link-descrption)
  (let ((name (cdr (assoc :name link-descrption))))
    (format nil "~A.geometry" name)))

(defun collada-geometry-name-base (link-descrption)
  (let ((name (cdr (assoc :name link-descrption))))
    (format nil "~A_geometry" name)))

(defun links-description->collada-geometries (links-description)
  (mapcar #'link-description->collada-geometry links-description))

(defun mesh-object->collada-mesh (mesh id)
  `(mesh
   (source                            ;list of positions
    (@ (id ,(format nil "~A.positions" id)))
    (float_array
     (@ (id ,(format nil "~A.positions.float-array" id))
        ;;(count ,(* 3 (length (send mesh :vertices)))))
        (count ,(* 3 3 (length (send mesh :faces)))))
     ,(mesh-vertices->collada-string mesh))
    (technique_common
     (accessor
      (@ (source ,(format nil "#~A.positions.float-array" id))
         (count ,(* 3 (length (send mesh :faces))))
         (stride "3"))
      (param (@ (name "X") (type "float")))
      (param (@ (name "Y") (type "float")))
      (param (@ (name "Z") (type "float"))))))
   (source                            ;list of normals
    (@ (id ,(format nil "~A.normals" id)))
    (float_array
     (@ (id ,(format nil "~A.normals.float-array" id))
        (count ,(* 3 (length (send mesh :faces)))))
     ,(mesh-normals->collada-string mesh))
    (technique_common
     (accessor
      (@ (source ,(format nil "#~A.normals.float-array" id))
         (count ,(length (send mesh :faces)))
         (stride "3"))
      (param (@ (name "X") (type "float")))
      (param (@ (name "Y") (type "float")))
      (param (@ (name "Z") (type "float"))))))
   (vertices
    (@ (id ,(format nil "~A.vertices" id)))
    (input
     (@ (semantic "POSITION")
        (source ,(format nil "#~A.positions" id)))))
   (triangles
    (@ (count ,(length (send mesh :faces)))
       (material ,(format nil "~A.material" id)))
    (input
     (@ (offset "0")
        (semantic "VERTEX")
        (source ,(format nil "~A.vertices" id))))
    (input
     (@ (offset "1")
        (semantic "NORMAL")
        (source ,(format nil "~A.normal" id))))
    ;; indices
    ,(mesh->collada-indices mesh)
    )))

(defun link-description->collada-geometry (link-description)
  (let ((id-base (collada-geometry-id-base link-description))
        (name-base (collada-geometry-name-base link-description))
        (meshes (cdr (assoc :mesh link-description))))
    (let ((ids (enum-integer-list (length meshes))))
      (mapcar
       #'(lambda (i m)
           (let ((id (format nil "~A.~A" id-base i))
                 (name (format nil "~A.~A" name-base i)))
             `(geometry
               (@ (id ,id)
                  (name ,name))
               ,(mesh-object->collada-mesh m id))))
       ids meshes))))

(defun mesh->collada-indices (mesh)
  ;; little bit tricky. returns the list of <p> tags as a string!
  (let ((faces (send mesh :faces))
        (ss (make-string-output-stream))
        (vertex-count 0)
        (normal-count 0))
    (format ss "<p>")
    (dotimes (fi (length faces))
      (let ((f (elt faces fi)))
        (let ((vertices-num (length (cdr (send f :vertices)))))
          (dotimes (vi vertices-num)
            (format ss "~A " vertex-count) ;vertex index
            (incf vertex-count)           ;increment vertex per vertex
            (format ss "~A " normal-count)) ;normal index
          (incf normal-count)           ;increment normal per faces
          )
        (format ss "~%")
        ))
    (format ss "</p>~%")
    (get-output-stream-string ss)))

(defun mesh-vertices->collada-string (mesh)
  (let ((faces (send mesh :faces)))
    ;; vertices = list of float vector
    (let ((ss (make-string-output-stream)))
      (dolist (f faces)
        (let ((vertices (cdr (send f :vertices))))
          (dolist (v vertices)
            (dotimes (i 3)
              (format ss "~A " (elt v i)))
            (format ss "~%"))))
      (get-output-stream-string ss))))

(defun mesh-normals->collada-string (mesh)
  (let ((faces (send mesh :faces)))
    (let ((ss (make-string-output-stream)))
      (dolist (f faces)
        (let ((normal (send f :normal)))
          (dotimes (i 3)
            (format ss "~A " (elt normal i)))
          (format ss "~%")))
      (get-output-stream-string ss))))

(defun float-vector->collada-string (v)
  (let ((ss (make-string-output-stream)))
    (dotimes (i (length v))
      (format ss "~A " (elt v i)))
    (format ss "~%")
    (get-output-stream-string ss)))

;; util
(defun enum-integer-list (n)
  (let ((ids nil))
    (dotimes (i n)
      (push i ids))
    (reverse ids)))

(defun search-minimum-rotation-matrix (mat)
  (let* ((a (matrix-column mat 0))
         (b (matrix-column mat 1))
         (c (matrix-column mat 2))
         (src-lst (list a b c))
         (xx (float-vector 1 0 0))
         (yy (float-vector 0 1 0))
         (zz (float-vector 0 0 1))
         (dst-lst (list xx yy zz))
         (max-dist -2)
         ret)
    ;; search minimum distance of each axis on new matrix
    (dolist (src src-lst)
      (dolist (dst dst-lst)
        (let ((dist (abs (v. src dst))))
          (when (> dist max-dist)
            (setq max-dist dist)
            (setq ret (cons src dst))))))
    ;; choose order for making rotation matrix
    (transpose
    (cond
     ((eq (cdr ret) xx)
      (let ((r (apply #'matrix (if (> (v. (car ret) (cdr ret)) 0)
                                   (car ret) (scale -1.0 (car ret)))
                      (remove (car ret) src-lst))))
        (if (< (user::matrix-determinant r) 0)
            (apply #'matrix (if (> (v. (car ret) (cdr ret)) 0)
                                (car ret) (scale -1.0 (car ret)))
                   (reverse (remove (car ret) src-lst))) r)))
     ((eq (cdr ret) yy)
      (let* ((sc (remove (car ret) src-lst))
             (r (matrix (car sc) (if (> (v. (car ret) (cdr ret)) 0)
                                     (car ret) (scale -1.0 (car ret)))
                        (cadr sc))))
        (if (< (user::matrix-determinant r) 0)
            (matrix (cadr sc) (if (> (v. (car ret) (cdr ret)) 0)
                                (car ret) (scale -1.0 (car ret)))
                    (car sc)) r)))
     (t ;;(eq (cdr ret) zz)
      (let* ((sc (remove (car ret) src-lst))
             (r (matrix (car sc) (cadr sc) (if (> (v. (car ret) (cdr ret)) 0)
                                               (car ret) (scale -1.0 (car ret))))))
        (if (< (user::matrix-determinant r) 0)
            (matrix (cadr sc) (car sc) (if (> (v. (car ret) (cdr ret)) 0)
                                           (car ret) (scale -1.0 (car ret))))))))
      )))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; convert function
;;  irtmodel -> collada model
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun estimate-class-name (model-file)
  (string-upcase (pathname-name (pathname model-file))))

(defun remove-directory-name (fname)
  (let ((p (pathname fname)))
    (format nil "~A.~A" (pathname-name p) (pathname-type p))))

(defun convert-irtmodel-to-collada
  (model-file &optional (output-full-dir (send (truename "./") :namestring)) (model-name) (exit-p t))
  "convert irtmodel to collada model. (convert-irtmodel-to-collada irtmodel-file-path &optional (output-full-dir (send (truename \"./\") :namestring)) (model-name))"
  (format t "convert ~A and output to ~A~%" model-file output-full-dir)

  (format t "loading ~A~%" model-file)
  (load model-file)
  (let ((model-class-name
         (estimate-class-name
          (remove-directory-name model-file))))
    (let ((model (instance (eval (intern model-class-name)) :init)))
      (if model-name (send model :name model-name))
      (eus2collada model output-full-dir)))
  (if exit-p (exit)))

(in-package "GEOMETRY")
;;(import '(collada::convert-irtmodel-to-collada))
(provide :irtcollada)
