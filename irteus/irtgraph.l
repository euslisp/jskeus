;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; $Id: irtgraph.l $
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

;; this file is copied from euslib/jsk...

;;=======================================================
;; irt graph class
;;
;; You have to create subclass of graph class depending on
;; Your Application.
;;

;;=======================================
;;
;; pure node/arc class for directional graph
;;

(defclass node
  :super propertied-object
  :slots (arc-list))
(defmethod node
  (:init (n)
    (send self :name n)
    self)
  (:arc-list ()
    arc-list)
  (:successors ()
    (mapcar #'(lambda(a)(cons a (send a :to))) arc-list))
  (:add-arc (a)
    (push a arc-list))
  (:remove-arc (a)
    (setq arc-list (remove a arc-list)))
  (:remove-all-arcs ()
    (setq arc-list nil))
  (:unlink (n)
    (setq arc-list
	  (remove-if #'(lambda(a)(eq n (send a :to))) arc-list)))
  )

(defclass arc
  :super propertied-object
  :slots (from to))
(defmethod arc
  (:init (from_ to_)
    (setq from from_ to to_)
    (send from :add-arc self)
    self)
  (:from () from)
  (:to () to)
  (:prin1 (&optional (strm t) &rest msgs)
    (send-super :prin1 strm
		(format nil "~A->~A~A" from to (or msgs "")))))

;;
;; directed graph
;;
(defclass directed-graph
  :super propertied-object
  :slots (nodes))
(defmethod directed-graph
  (:init ()
    self)
  ;;
  (:successors (node &rest args)
    (send node :successors))
  (:node (name)
    (find name nodes :key #'(lambda (x) (send x :name)) :test #'equal))
  ;; manipulate nodes/arcs
  (:nodes (&optional arg)
    (when arg
      (dolist (rmnode (set-difference nodes arg))
	(send-all arg :unlink rmnode))
      (setq nodes arg))
    nodes)
  (:add-node (n)
    (push n nodes)
    n )
  (:remove-node (n)
    (setq nodes (remove n nodes))
    (send-all nodes :unlink n)
    nodes)
  (:clear-nodes ()
    (send-all nodes :remove-all-arcs)
    (setq nodes nil))
  (:add-arc-from-to (from to)
    (instance arc :init from to))
  (:remove-arc (arc)
    (send (send arc :from) :remove-arc arc))
  ;; calcurate graph property
  (:adjacency-matrix ()
    (let ((size (length nodes)) mat j)
      (setq mat (make-matrix size size))
      (dotimes (i size)
	(dolist (next (send-all (send (elt nodes i) :arc-list) :to))
	  (setq j (position next nodes))
	  (when j (incf (aref mat i j)))
	  ))
      mat))
  (:adjacency-list ()
    (mapcar #'(lambda(n)
		(sort (mapcar #'(lambda(m)(position m nodes))
			      (send-all (send n :arc-list) :to)) #'<))
	    nodes))
  )


(defclass costed-arc
  :super arc
  :slots (cost))
(defmethod costed-arc
  (:init (from to c)
    (setq cost c)
    (send-super :init from to))
  (:cost () cost))

(defclass costed-graph
  :super directed-graph
  :slots ())
(defmethod costed-graph
  (:add-arc (from to cost &key (both nil))
    (send self :add-arc-from-to from to cost :both both))
  (:add-arc-from-to (from to cost &key (both nil))
    (let ((ar (instance costed-arc :init from to cost)))
      (if both
	  (list ar (instance costed-arc :init to from cost))
	ar)))
  (:path-cost (from arc to)
    (send arc :cost)))

;;=======================================
;; jsk standard graph
;;  . costed-graph with start and goal
;;  . cost of arc = 1
;;  . nodes has :cost method

(defclass graph
  :super costed-graph
  :slots (start-state goal-state))

(defmethod graph
  ;; :goal-test method need to be overriden according to your application.
  ;; :goal-test must be fast.
  (:goal-test (gs)
    (eq goal-state gs))
  ;; :path-cost returns cost from 'from-node' to 'to-node' by 'action'.
  (:path-cost (from arc to)
    (+ (send from :cost) (send arc :cost)))
  ;;accessors
  (:start-state (&optional arg)
    (if arg (setq start-state arg))
    start-state)
  (:goal-state (&optional arg)
    (if arg (setq goal-state arg))
    goal-state)
  (:add-arc (from to &key (both nil))
    (unless (listp to) (setq to (list to)))
    (mapcar #'(lambda (next)
		(send self :add-arc-from-to from next :both both)) to))
  (:add-arc-from-to (from to &key (both nil))
    (send-super :add-arc-from-to from to 1 :both both))
  )

;;
;; write methods of directed-graph
;;
(defmethod directed-graph
  ;; :write-to-dot method is supposed to be meaningful
  ;; in only static graph.
  (:write-to-dot (fname &optional result-path (title "output"))
    (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
           (mapcar #'(lambda (n)
                       (cons n (string (gensym))))
                   nodes)))
      (labels ((graphviz-node-name
                (n)
                (cdr (assoc n node-alist))))
      (with-open-file (f fname :direction :output)
        (format f "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (format f "   ")
          ;; hoge = "hoge";
          (format f "~A [label = \"~s\"];~%"
                  (graphviz-node-name target-node)
                  (send target-node :name))
          )
        ;; first of all, write result-path
        (let ((drawed-arc nil))
          (let (anode)
            (while (and (setq anode (pop result-path)) result-path)
              (format f "   ")
              (format f "~A -> ~A [color = red];~%"
                      (graphviz-node-name (send anode :state))
                      (graphviz-node-name (send (car result-path) :state)))))
          (dolist (target-node nodes)
            (let ((neighbors
                   (mapcar #'cdr (send self :successors target-node))))
              (dolist (neighbor neighbors)
                (when (not (or (find (cons target-node neighbor) drawed-arc
                                     :test #'equal)
                               (find (cons neighbor target-node) drawed-arc
                                     :test #'equal)))
                    (if (member target-node
                              (mapcar #'cdr (send self :successors neighbor)))
                      (progn
                        (format f "   ")
                        (format f "~A -> ~A [dir = both];~%"
                                (graphviz-node-name target-node)
                                (graphviz-node-name neighbor)))
                    (progn
                      (format f "   ")
                      (format f "~A -> ~A;~%"
                              (graphviz-node-name target-node)
                              (graphviz-node-name neighbor))))
                  (push (cons target-node neighbor) drawed-arc)))))
          (format f "}~%")
          t)))))
  (:write-to-pdf (fname &optional result-path
                        (title (string-right-trim ".pdf" fname)))
    (let ((dot-fname
           (format nil "~A.dot" (string-right-trim ".pdf" fname))))
      (send self :write-to-dot dot-fname result-path title)
      (unix:system (format nil "dot ~A -Tpdf -o ~A" dot-fname fname))
      t))
  )

;;=======================================
;; obsolated methods and class for backward compatibility
;;
(defmethod node
  (:add-neighbor (n &optional a)
    (let ((ar (instance arc :init self n)))
      (when a (send ar :name a))
      (send self :neighbors)))
  (:neighbors (&optional args)
    (if args (dolist (n args) (instance arc :init self n)))
    (mapcar #'cdr (send self :successors)))
  )
(defclass arced-node
  :super node
  :slots ())
(defmethod arced-node
  (:init (&key name)
   (send-super :init name)
   self)
  (:find-action (n)
    (let ((act (find n arc-list :key #'(lambda(a)(send a :to)))))
      (when act (send act :name))))
  (:neighbor-action-alist ()
    (mapcar #'(lambda(a)(cons (send a :name) (send a :to))) arc-list))
  )

;;=======================================
;; solver-node class
;;
;; solver-node has history of the past and so on...
;; state slot is a solver-node's information.
;; So it is a mistake that you make a subclass of solver-node.
;; You have to device node class and graph(problem) class.
(defclass solver-node
  :super propertied-object
  :slots (state cost parent action memorized-path))

(defmethod solver-node
  (:init (st &key ((:cost c) 0)
                  ((:parent p) nil)
                  ((:action a) nil))
    (setq state st)
    (setq cost c
          parent p
          action a)
    self)
  ;; returns path to this node
  (:path (&optional (prev nil))
    (if prev
        (if (send self :parent)
            (send (send self :parent)
                  :path (cons self prev))
          (cons self prev))
      (if memorized-path
          memorized-path
        (setq memorized-path (if (send self :parent)
                                 (send (send self :parent)
                                       :path (cons self prev))
                               (cons self prev))))))
  ;; expand the next nodes using :successors method of prblm
  (:expand (prblm &rest args)
    (let ((successors (send* prblm :successors state args)))
      ;; successor = (action . next-state)
      (let ((ret nil))
        (dolist (successor successors)
          (push (instance solver-node :init (cdr successor)
                          :parent self
                          :action (car successor)
                          :cost (send prblm :path-cost self
                                      (car successor) (cdr successor)))
                ret))
        ret)))
  (:state (&optional arg)
    (if arg (setq state arg))
    state)
  (:cost (&optional arg)
    (if arg (setq cost arg))
    cost)
  (:parent (&optional arg)
    (if arg (setq parent arg))
    parent)
  (:action (&optional arg)
    (if arg (setq action arg))
    action)
  )

;;=======================================
;; solver
;;
;;solver class
;;
(defclass solver
  :super propertied-object
  :slots ())

(defmethod solver
  (:init ()
    self)
  (:solve (prblm)
    nil)
  (:solve-by-name (prblm s g &key (verbose nil))
    (send prblm :start-state (send prblm :node s))
    (send prblm :goal-state (send prblm :node g))
    (send self :solve prblm :verbose verbose)
    )
  )

;;=======================================
;; uninformed search algorithms
(defclass graph-search-solver
  :super solver
  :slots (open-list close-list))

(defmethod graph-search-solver
  (:solve-init (prblm)
    (setq close-list nil)
    (send self :add-to-open-list
          (instance solver-node :init (send prblm :start-state) :cost 0)))
  (:find-node-in-close-list (n)
    (find (send n :state) close-list))
  (:solve (prblm &key (verbose nil))
    (send self :solve-init prblm)
    (while (not (send self :null-open-list?))
;;;       (if verbose
;;;           (warn "current open-list num -> ~A -- ~A --~%"
;;;                 (length open-list)
;;;                 :solve))
      (let ((target-node (send self :pop-from-open-list :debug verbose)))
        ;; here, target-node is removed from open-list in :pop-from-open-list.
;;;         (if verbose
;;;             (warn "target-node is ~A -- ~A -- ~%"
;;;                   (send target-node :state) :solve))
        (cond ((send prblm :goal-test (send target-node :state))
;;;                (if verbose
;;;                    (warn "arrived at goal! -- ~A --~%" :solve))
               (return-from :solve (send target-node :path)))
              ((not (send self :find-node-in-close-list target-node))
               (push (send target-node :state) close-list)
               (send self :add-list-to-open-list
                     (send target-node :expand prblm :verbose verbose)))
              )))
    (warn "open-list is nil... -- ~A --~%" :solve)
    (warn "search was missed!! -- ~A --~%" :solve)
    (send self :clear-open-list)
    (setq close-list nil)
    nil)
  ;; open-list functions
  (:add-to-open-list (obj/list)
    (if (listp obj/list)
        (send self :add-list-to-open-list obj/list)
      (send self :add-object-to-open-list obj/list)))
  (:null-open-list? ()
    (null open-list))
  (:clear-open-list ()
    (warn "you have to override :clear-open-list method~%")
    nil)
  (:add-list-to-open-list (lst)
    (warn "you have to override :add-list-to-open-list method~%")
    nil)
  (:add-object-to-open-list (lst)
    (warn "you have to override :add-object-to-open-list method~%")
    nil)
  (:pop-from-open-list (&key (debug))
    (warn "you have to override :pop-from-open-list method~%")
    nil)
  ;; accessors
  (:open-list (&optional arg)
    (if arg (setq open-list arg))
    open-list)
  (:close-list (&optional arg)
    (if arg (setq close-list arg))
    close-list)
  )

;;======================================
;; bredth first
(defclass breadth-first-graph-search-solver
  :super graph-search-solver
  :slots ())

(defmethod breadth-first-graph-search-solver
  (:init ()
    (setq open-list (list))
    self)
  (:clear-open-list ()
    (setq open-list nil)
    nil)
  (:add-list-to-open-list (lst)
    ;; add to the tail of open-list
    (setq open-list (nconc open-list lst))
    open-list)
  (:add-object-to-open-list (obj)
    ;; add to the tail of open-list
    (setq open-list (nconc open-list (list obj)))
    open-list)
  (:pop-from-open-list (&key (debug))
    (pop open-list))
  )

;;=======================================
;; depth first
(defclass depth-first-graph-search-solver
  :super graph-search-solver
  :slots ())

(defmethod depth-first-graph-search-solver
  (:init ()
    (setq open-list (list))
    self)
  (:clear-open-list ()
    (setq open-list nil)
    nil)
  (:add-list-to-open-list (lst)
    ;; add to the head of open-list
    (setq open-list (nconc lst open-list))
    open-list)
  (:add-object-to-open-list (obj)
    ;; add to the head of open-list
    (push obj open-list)
    open-list)
  (:pop-from-open-list (&key (debug))
    (pop open-list))
  )

;;=======================================
;; informed search

;;=======================================
;; best first
(defclass best-first-graph-search-solver
  :super graph-search-solver
  :slots (aproblem))

(defmethod best-first-graph-search-solver
  (:init (p)
    (setq aproblem p)
    (setq open-list (list))
    self)
  (:clear-open-list ()
    (setq open-list nil)
    nil)
  (:add-list-to-open-list (lst)
    (setq open-list (nconc open-list lst))
    open-list)
  (:add-object-to-open-list (obj)
    (push obj open-list)
    open-list)
  (:pop-from-open-list (&key (debug nil))
    ;; returns the element which has minimum priority-value.
    ;;      => Priority Queue.
    ;; priority-value is calculated by :fn.
    ;; In AIMA , priority-value is equal to f(n).
    ;;
    ;; elements of open-list must be  propertied-objects
    (let* ((min-x (car open-list)))
      (let ((min-value (or (send min-x :get :priority-value)
                           (progn   ; if does not have :priority-value
                             (send min-x :put :priority-value
                                   (send self :fn min-x aproblem))
                             (send min-x :get :priority-value)))))
        (when debug
          (warn "<~A> v -> ~A~%"
                (send-all (send-all (send min-x :path)  :state) :name)
                min-value))
        (dolist (x (cdr open-list))
          (let ((v (or (get x :priority-value)
                       (progn       ; if does not have :priority-value
                         (setf (get x :priority-value)
                               (send self :fn x aproblem))
                         (get x :priority-value)))))
            (when debug
              (warn "<~A> v -> ~A~%"
                    (send-all (send-all (send x :path) :state) :name)
                    v))
            (if (< v min-value)
                (progn
                  (setq min-value v)
                  (setq min-x x)))
            ))
      (when debug
        (warn ":pop-from-open-list result...~%")
        (warn "min-value -> ~A~%" min-value)
        (warn "min-x -> ~A~%"
              (send-all (send-all (send min-x :path) :state) :name)))
      ;; remove from open-list
      (setq open-list (delete min-x open-list :count 1))
      min-x)))
  (:fn (n p)
    ;; calculate f(n)
    ;; in greedy search, f(n) = g(n), h(n) = 0.
    (send n :cost))
  )

;;=======================================
;; A*
;; f(n) = g(n) + h(n)
;; where g(n) is the cost from start node to n.
;;       h(n) is the estimated cost from n to goal node.
(defclass a*-graph-search-solver
  :super best-first-graph-search-solver
  :slots ())

(defmethod a*-graph-search-solver
  (:init (p)
    (send-super :init p)
    self)
  ;; n -> next node
  ;; p -> prnoblem
  (:fn (n p &key (debug nil))
    (when debug
      (warn "g(n) -> ~A -- F(N) --~%" (send self :gn n p))
      (warn "h(n) -> ~A -- F(N) --~%" (send self :hn n p)))
    (+ (send self :gn n p)
       (send self :hn n p)))
  (:gn (n p)
    (send n :cost))
  (:hn (n p)
    (warn "You have to override :hn~%")
    0.0)
  )

#|
(defclass kd-tree-node
  :super node
  :slots (parent children value axis))
(defmethod kd-tree-node
  (:init 
   (val axs)
   (setq value val axis axs)
   (send-super :init value))
  (:value () value)
  (:axis () axis)
  (:parent (&optional args) (if args (setq parent args)) parent)
  (:children (&optional args) (if args (setq children args)) children)
  )

(defclass kd-tree
  :super tree
  :slots (key))

(defmethod kd-tree
  (:init
   (plist &key ((:key k) #'identity))
   (let ()
     (setq key k)
     (setq root (send self :build plist))
     (send self :add-node root)
     self))
  (:select-sort-axis
   (plist)
   (let* ((axis 0)
          (v (v- (apply #'vmax plist) (apply #'vmin plist)))
          (m (elt v axis))
          ;;(vi (car plist)) (va (car plist))
          ;;(len (length plist))
          ;;(p (car plist)) (pp (cdr plist)) m
          )
#|
     (while pp
       (setq vi (vmin p vi))
       (setq va (vmax p va))
       (setq p (car pp) pp (cdr pp)))
     (setq v (v- va vi))
     (setq m (elt v axis))
|#
     (dotimes (i (length v))
       (if (> (elt v i) m) (setq m (elt v i) axis i)))
     axis))
  ;;
  ;; build kdtree with recuresive methods (deperecated)
  ;;
  (:build
   (plist)
   (let ((len (length plist))
         idx plist-short axis
         med nd lnd rnd lbuf rbuf (skipp t) mm pp)
     ;;
     (if (> len 1000)
         (setq plist-short (subseq plist 0 1000) idx 500)
       (setq plist-short plist idx (/ len 2)))

     (setq axis (send self :select-sort-axis (mapcar key plist-short)))
     (setq plist-short (sort plist-short
                             #'(lambda (x y) (<= (elt (funcall key x) axis)
                                                 (elt (funcall key y) axis)))))
     (setq med (elt plist-short idx))

     (dolist (p plist)
       (setq pp (funcall key p)
             mm (funcall key med))
       (cond ((and skipp (v= mm pp)) (setq skipp nil med p))
             ((< (elt pp axis) (elt mm axis)) (push p lbuf))
             ((>= (elt pp axis) (elt mm axis)) (push p rbuf))))

     (setq nd (instance kd-tree-node :init med axis))
     (when lbuf
       (setq lnd (send self :build lbuf))
       (send lnd :parent nd)
       (send self :add-node lnd)
       (send self :add-arc nd lnd))
     (when rbuf
       (setq rnd (send self :build rbuf))
       (send rnd :parent nd)
       (send self :add-node rnd)
       (send self :add-arc nd rnd))
     (send nd :children (list lnd rnd))
     nd))
  ;;
  ;; build kdtree with non-recuresive methods (ALIVE)
  ;;
  (:build
   (plist)
   (let* (len plist-short axis idx med nd lnd rnd lbuf rbuf pp mm skipp
          (build-seq (list (list plist nil t nil))) cur-build)
     ;;
     (setq cur-build build-seq)
     (while cur-build
       (setq plist (caar cur-build))
       (setq len (length plist))
       (if (> len 1000)
           (setq plist-short (subseq plist 0 1000) idx 500)
         (setq plist-short plist idx (/ len 2)))
       (setq axis (send self :select-sort-axis (mapcar key plist-short)))
       (setq plist-short (sort plist-short
                               #'(lambda (x y) (<= (elt (funcall key x) axis)
                                                   (elt (funcall key y) axis)))))
       (setq med (elt plist-short idx))

       (setq skipp t rbuf nil lbuf nil)
       (dolist (p plist)
         (setq pp (funcall key p)
               mm (funcall key med))
         (cond ((and skipp (v= mm pp)) (setq skipp nil med p))
               ((< (elt pp axis) (elt mm axis)) (push p lbuf))
               ((>= (elt pp axis) (elt mm axis)) (push p rbuf))))

       (setq nd (instance kd-tree-node :init med axis))
       (if (cdr build-seq) (send self :add-node nd))
       (when (elt (car cur-build) 2)
         (setf (elt (car cur-build) 2) nd))
       (when (elt (car cur-build) 3)
         (setf (elt (car cur-build) 3) nd))
       ;;
       (when lbuf
         (nconc build-seq (list (list lbuf nd t nil))))
       (when rbuf
         (nconc build-seq (list (list rbuf nd nil t))))
       (setq cur-build (cdr cur-build)))
     (setq cur-build (reverse build-seq))

     (while cur-build
       (let ((nd (elt (car cur-build) 1))
             (lnd (elt (car cur-build) 2))
             (rnd (elt (car cur-build) 3)))
         (unless nd (return lnd))
         (when lnd
           (send self :add-arc nd lnd)
           (send lnd :parent nd)
           (send nd :children (list lnd (cadr (send nd :children)))))
         (when rnd
           (send self :add-arc nd rnd)
           (send rnd :parent nd)
           (send nd :children (list (car (send nd :children)) rnd)))
         (setq cur-build (cdr cur-build))
         ))
     ))
  ;;
  ;; search same node from kdtree (deperecated)
  ;;
  (:search (&rest args) 
           (send (send* self :search-node args) :value))
  (:search-node
   (p &key ((:root rootnode) root) (debug) ((:epsilon eps) single-float-epsilon))
   (let (value d axis children (pp (funcall key p)) vv
               (nodeptr rootnode) (mindist 10000.0)
               (minnode rootnode))
     (if debug (warn ";; ~A << search target~%" p))
     (while nodeptr
       (setq value (send nodeptr :value)
             axis  (send nodeptr :axis)
             vv    (funcall key value))
       (if debug (warn ";; ~A (~A)~%" value axis))
       (if (< (setq d (distance pp vv)) mindist)
           (setq mindist d minnode nodeptr))
       (when (eps= mindist 0 eps)
         (if debug (warn ";; ~A (~A) << result~%" (send minnode :value) mindist))
         (return-from :search-node minnode))
       (setq children (send nodeptr :children))
       (cond ((< (elt pp axis) (elt vv axis))
              (setq nodeptr (car children)))
             (t
              (setq nodeptr (cadr children)))))
     (if debug (warn ";; ~A (~A) << result?~%" (send minnode :value) mindist))
     minnode))
  ;;
  ;; search nearest neighbor node (ALIVE)
  ;;
  (:search (p &rest args) 
           (send (send* self :search-nn-node root p args) :value))
  (:search-nn-node
   (nodeptr p &key (mindist 10000.0) ((:epsilon eps) single-float-epsilon)
            (minnode nodeptr) debug)
   (let (value axis pp vv d childptr)
     (if debug (warn ";; ~A << search target~%" p))
     (setq value (send nodeptr :value)
           axis  (send nodeptr :axis)
           pp    (funcall key p)
           vv    (funcall key value))
     (if debug (warn ";; ~A (~A)~%" value axis))
     (when (< (setq d (distance pp vv)) mindist)
       (setq mindist d minnode nodeptr)
       (when (eps= mindist 0 eps)
         (if debug (warn ";; ~A (~A) << result~%" (send minnode :value) mindist))
         (return-from :search-nn-node minnode)))

     (setq childptr (send nodeptr :children))
     (when (and (car childptr)
                (< (elt pp axis) (elt vv axis)))
       (setq minnode (send self :search-nn-node (car childptr) p
                           :mindist mindist :epsilon eps :minnode minnode
                           :debug debug))
       (setq mindist (min mindist
                          (distance pp (funcall key (send minnode :value)))))
       (when (and (cadr childptr)
                  (< (- (elt vv axis) (elt pp axis)) mindist))
         (setq minnode (send self :search-nn-node (cadr childptr) p
                             :mindist mindist :epsilon eps :minnode minnode
                             :debug debug))
         (setq mindist (min mindist
                            (distance pp (funcall key (send minnode :value)))))
         ))
     (when (and (cadr childptr)
                (>= (elt pp axis) (elt vv axis)))
       (setq minnode (send self :search-nn-node (cadr childptr) p
                           :mindist mindist :epsilon eps :minnode minnode
                           :debug debug))
       (setq mindist (min mindist
                          (distance pp (funcall key (send minnode :value)))))
       (when (and (car childptr)
                  (< (- (elt pp axis) (elt vv axis) ) mindist))
         (setq minnode (send self :search-nn-node (car childptr) p
                             :mindist mindist :epsilon eps :minnode minnode
                             :debug debug))
         (setq mindist (min mindist
                            (distance pp (funcall key (send minnode :value)))))
         ))
     minnode))
  )
|#

(defclass kd-tree
  :super propertied-object
  :slots (obj key return-type npoints points))
(defmethod kd-tree
  (:init
    (plist &key ((:key k) #'identity))
    (let (parray)
      (sys::dispose-hook self t)
      (setq key k)
      (cond
       ((arrayp plist)
        (setq points plist npoints (array-dimension plist 0))
        (setq parray plist return-type array))
       ((listp plist)
        (setq points plist npoints (length plist))
        (let ((i 0))
          (setq parray (make-matrix (length plist) (length (car plist)))
                return-type cons)
          (dolist (p plist)
            (setf (matrix-row parray i) p)
            (incf i))))
       (t (warn "unsupported type ~A~%" plist)))
      (setq obj (c-kdtree-init (array-entity parray)
                               (array-dimension parray 0)
                               (array-dimension parray 1)))
      self))
  ;;
  (:points () points)
  (:number-of-points () npoints)
  (:c-search (type p
              &key ((:epsilon eps) single-float-epsilon) (k 4) (radius) debug
        return-index)
    (let (ret idx-lst)
      (case
       type
       (:search
         (setq idx-lst (if return-index (make-array 1 :element-type :integer) 0))
         (setq ret (c-kdtree-k-search obj p 1 eps idx-lst))
         (return-from :c-search (if return-index (elt idx-lst 0) (matrix-row ret 0))))
       (:nearest-neighbor
         (setq idx-lst (if return-index (make-array k :element-type :integer) 0))
         (setq ret (c-kdtree-k-search obj p k eps idx-lst)))
       (:fixed-radius
   (if return-index
       (setq idx-lst (c-kdtree-fr-search obj p k radius eps 1))
     (setq ret (c-kdtree-fr-search obj p k radius eps 0))))
       (t (warn "unknown type ~A~%" type)))
      (cond
       (return-index (setq ret idx-lst))
       ((and ret (eq return-type cons))
  (let ((k (array-dimension ret 0)) r)
          (dotimes (i k)
            (push (matrix-row ret i) r))
          (setq ret (nreverse r)))))
      ret))
  (:search (&rest args)
    "search nearest neighbor (just one point)"
    (send* self :c-search :search args))
  (:nearest-neighbor (&rest args)
    "search nearest k neighbors, use :k keyword"
    (send* self :c-search :nearest-neighbor args))
  (:fixed-radius (p &rest args &key (k 0) &allow-other-keys)
    "search k points in fixed radius from p, if k=0 search all points in fixed radius"
    (send* self :c-search :fixed-radius p :k k args))
  (:dispose () (if obj (c-kdtree-quit obj)))
  (:quit () (if obj (prog1 (c-kdtree-quit obj) (setq obj nil))))
  (:workingp () obj)
  (:search-corresponding-index (mat &key (idx) (dist) (k 1) ((:epsilon eps) single-float-epsilon))
   (let* ((ret-idx (if idx idx (make-array (* k (array-dimension mat 0)) :element-type :integer)))
          (dlst (if dist dist 0)))
     (c-kdtree-k-search-matrix obj mat k eps ret-idx dlst)
     ret-idx))
  )

(defclass dynamic-kdtree
  :super propertied-object
  :slots (n trees))
(defmethod dynamic-kdtree
  (:init
   ()
   (setq n 12)
   (setq trees (make-array n))
   )

  (:insert
   (pt)
   (let (j)
     (block loop
       (dotimes (i n)
   (when (null (aref trees i))
     (setq j i)
     (return-from loop))
   )

       ;; expand arrays if necessary
       (warn "expand the table of kd trees~%")
       (setq j n)
       (setq n (* 2 n))
       (let ((newtrees (make-array (* 2 n))))
   (dotimes (i (length trees))
     (aset newtrees i (aref trees i)))
   (setq trees newtrees))
       )

     ;; collect points in the smaller trees
     ;; clean points in the trees
     (let (pts)
       (dotimes (i j)
   ;; (format t "pts[~a]: ~a~%" i (send (aref trees i) :points))
   (setq pts (append (send (aref trees i) :points) pts))
   (sys::dispose-hook (aref trees i) nil)
   )
       ;; (format t "new pts[~a]: ~a~%" j (cons pt pts))
       (aset trees j (instance kd-tree :init (cons pt pts)))
       )
     (dotimes (i j) (aset trees i nil))
     )
   )

  (:npoints
   ()
   (let ((np 0))
     (dotimes (i n)
       (when (aref trees i)
   (setq np (+ np (send (aref trees i) :number-of-points)))))
     np)
   )

  (:nearest-neighbor
   (pt)
   (let ((mind most-positive-float)
   nnp)
     (dotimes (i n)
       (if (aref trees i)
     (let* ((nnp-candidate
       (car (send (aref trees i) :nearest-neighbor pt :k 1)))
      (d (distance nnp-candidate pt)))
       (when (< d mind)
         (setq mind d nnp nnp-candidate))
       )))
     (list mind nnp))
   )

  )

#|
;;(setq k (instance kd-tree :init
;                  (list #f(2 3) #f(5 3) #f(8 3) #f(3 4) #f(3 4) #f(3 4) #f(6 2))))
;;(print (send k :search #f(5 3)))

(defun bench-kd-tree-1 (&optional (pnum 1000))
  (let (tm points ret)
    (setq *random-state* (coerce (unix::gettimeofday) integer-vector))
    (setq tm (instance mtimer :init))
    (dotimes (i pnum)
      (setq pp (float-vector (- 0.0 (random 100.0))
                             (- 0.0 (random 100.0))
                             (- 0.0 (random 0.0))))
      (push pp points))
    (let (dist minp (mindist 1000))
      (dolist (p points)
        (setq dist (distance p #f(0 0 0)))
        (when (< dist mindist) (setq mindist dist minp p)))
      (print (list 'full-search minp minp)))
    ;;
    (send tm :start)
    (setq k (instance kd-tree :init points))
    (print (list 'tree-creationg (send tm :stop)))
    (send tm :start)
    (setq ret (send k :search #f(0 0 0)))
    (print (list 'tree-retrieving (send tm :stop)))
    (print ret)
    ))
;(bench-kd-tree-1)

(defun bench-kd-tree-2 (&key (node-num 1000) ;; up to 50000
                     (data-size 5)
                     (debug))
  (let (points data-vec search-vector
               (tm (instance mtimer :init)))
    (format t "node-num: ~a node-num: ~a~%" node-num data-size)
    (dotimes (i node-num)
      (setq data-vec (instantiate float-vector data-size))
      (dotimes (j data-size)
        (setf (elt data-vec j) (random 100.0)))
      (setf (elt data-vec 0) 0)
      (push data-vec points))
    (setq search-vector (elt points (random node-num)))
    ;;(setq search-vector (instantiate float-vector data-size))
    (send tm :start)
    (setq k (instance kd-tree :init points))
    (print (list 'tree-creationg (send tm :stop)))
    (send tm :start)
    (setq ret (send k :search search-vector :debug debug))
    (print (list 'tree-retrieving (send tm :stop)))
    (print (list 'ret ret))
    (print (list 'ans search-vector))
  ))
 
;(bench-kd-tree-2) 
;;(bench-kd-tree-2 :node-num 10000 :data-size 100 :debug t)
(bench-kd-tree-2 :node-num 10 :debug t)
;(send k :write-to-pdf "kd-tree.pdf")

(setq *random-state* (integer-vector (elt (unix::gettimeofday) 0) (elt (unix::gettimeofday) 1)))
(setq *tm* (instance mtimer :init))

(defun kdt-test (&key (iteration 1) ((:size ss) 100) (eps single-float-epsilon))
  (setq smat (make-matrix ss 3))
  (setq dmat (make-matrix ss 3))

  (setq dlst0 (instantiate float-vector ss))
  (setq ilst0 (instantiate integer-vector ss))
  (setq dlst1 (instantiate float-vector ss))
  (setq ilst1 (instantiate integer-vector ss))

  (setq ret nil)

  (dotimes (i iteration)
    (let (tmp)
      (dotimes (i ss)
        (setf (matrix-row smat i)
              (scale 500 (random-vector)))
        (setf (matrix-row dmat i)
              (scale 500 (random-vector))))

      (send *tm* :start)
      (setq kdt (instance kd-tree :init dmat))
      (push (* 1000 (send *tm* :stop)) tmp)

      (send *tm* :start)
      (send kdt :search-corresponding-index smat :idx ilst0 :dist dlst0 :epsilon eps)
      (push (* 1000 (send *tm* :stop)) tmp)

      (send *tm* :start)
      (c-kdtree-k-search-matrix-raw dmat smat 1 single-float-epsilon ilst1 dlst1)
      (push (* 1000 (send *tm* :stop)) tmp)

      ;;check
      (dotimes (i ss)
        (unless (= (elt ilst0 i) (elt ilst1 i))
          (warn "error : ~A != ~A / ~F != ~F (~F)~%"
                (elt ilst0 i) (elt ilst1 i) (elt dlst0 i) (elt dlst1 i)
                (- (elt dlst0 i) (elt dlst1 i)))
          ))
      (push (coerce (nreverse tmp) float-vector) ret)
      (sys::gc)
      ))
  (vector-mean ret))
;; time check
(dotimes (i 21)
  (let ((ss (round (expt 10 (/ (+ i 5) 5.0))))
        itr)
    (setq itr (min 300 (/ 300000 ss)))
    (pprint (list ss itr (kdt-test :iteration itr :size ss)))))

;; Z800 3.2GHz
 point num, iteration, #f(kdtree-initialize kdtree-search all-search)
(10 300 #f(0.020607 0.01184 0.00527))
(16 300 #f(0.023363 0.01502 0.005663))
(25 300 #f(0.02536 0.0196 0.007447))
(40 300 #f(0.02975 0.02929 0.011687))
(63 300 #f(0.04038 0.045263 0.02073))
(100 300 #f(0.052463 0.071803 0.04253))
(158 300 #f(0.07182 0.114343 0.094477))
(251 300 #f(0.10217 0.18909 0.2217))
(398 300 #f(0.152513 0.312573 0.531877))
(631 300 #f(0.236863 0.517203 1.30164))
(1000 300 #f(0.375293 0.85893 3.20897))
(1585 189 #f(0.608921 1.41394 8.21947))
(2512 119 #f(1.00201 2.44978 20.4663))
(3981 75 #f(1.65612 4.295 51.1048))
(6310 47 #f(2.71764 7.35921 127.994))
(10000 30 #f(4.5811 12.3736 323.977))
(15849 18 #f(7.55239 20.7259 817.415))
(25119 11 #f(12.2907 34.6122 2069.27))
(39811 7 #f(20.4027 58.1197 5185.1))
(63096 4 #f(33.9372 109.535 13027.1))
(100000 3 #f(56.9617 222.269 32723.4))
|#

;;=======================================
;; samples
#|
(let ((gr (instance graph :init)) a b c ar1 ar2)
  (setq a (instance node :init 'a)
	b (instance node :init 'b)
	c (instance node :init 'c))
  (send gr :add-node a)
  (send gr :add-node b)
  (send gr :add-node c)
  (setq ar1 (send gr :add-arc-from-to a b)
	ar2 (send gr :add-arc-from-to b c))
  ;;
  (format t "~a" (send gr :successors a)) ;; -> (arc . b)
  (send gr :remove-arc ar1)
  (format t "~a" (send gr :successors a)) ;; -> nil
  ;;
  (send gr :remove-node b)
  (send gr :add-arc-from-to a c)
  (format t "~a" (send gr :node 'a)) ;; -> a
  ;;
  (send gr :clear-nodes)
  )

;; To see more samples, please check euslisp/test/graph.l

|#

(provide :irtgraph "$Id$")

