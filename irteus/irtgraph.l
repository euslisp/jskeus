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
  (:write-to-dot-stream (&optional (strm *standard-output*) result-path (title "output"))
   "write graph structure to stream as dot(graphviz) style
Args:
  strm: stream class for output
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
"
    (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
           (mapcar #'(lambda (n)
                       (cons n (string (gensym))))
                   nodes))
          (draw-both-arc (send self :get :draw-both-arc))
          (draw-arc-label (send self :get :draw-arc-label))
          (draw-result-only (not (send self :get :draw-merged-result)))
          )
      (labels ((graphviz-node-name
                (n)
                (cdr (assoc n node-alist))))
        (format strm "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (format strm "   ")
          (format strm "~A [label = \"~A\"];~%"
                  (graphviz-node-name target-node)
                  (send target-node :name))
          )
        ;; first of all, write result-path
        (let ((result-tbl (make-hash-table :test #'equal))
              (draw-tbl (make-hash-table :test #'equal))
              )
          ;; write result-path
          (let (anode)
            (while (and (setq anode (pop result-path)) result-path)
              (setf (gethash (cons (send anode :state)
                                   (send (car result-path) :state))
                             result-tbl)
                    (list "color = red"))
              ))
          ;; write arcs
          (dolist (target-node nodes)
            (let ((neighbors
                   (send self :successors target-node)
                   ))
              (dolist (neighbor neighbors)
                (let ((narc (car neighbor))
                      (nnode (cdr neighbor))
                      (draw-arc t) result
                      options)
                  ;; Is there in result path ?
                  (when (and (not draw-result-only)
                             (gethash (cons target-node nnode) result-tbl))
                    (setq result t)
                    (push "color = red" options))
                  (when draw-arc-label
                    (push (format nil "label = \"~A\"" (send narc :name)) options))
                  ;; checked drawed for both direction
                  (unless draw-both-arc
                    (cond
                     ((gethash (cons target-node nnode) draw-tbl)
                      (setq draw-arc nil)
                      )
                     ((gethash (cons nnode target-node) draw-tbl)
                      (setq draw-arc nil)
                      )
                     ((member target-node
                              (mapcar #'cdr (send self :successors nnode)))
                      (push "dir = both" options)
                      )
                     ))
                  (unless options (setq options :no-option))
                  (when draw-arc
                    (setf (gethash (cons target-node nnode) draw-tbl)
                          options))
                  ))
              ))
          (flet ((print-arcs (tbl)
                  (maphash
                   #'(lambda (key options)
                       (format strm "   ")
                       (format strm "~A -> ~A"
                               (graphviz-node-name (car key))
                               (graphviz-node-name (cdr key)))
                       (if (eq options :no-option)
                           (format strm ";~%")
                         (progn
                           (format strm "[")
                           (dolist (opt options)
                             (unless (eq opt (car options))
                               (format strm ", "))
                             (format strm "~A" opt))
                           (format strm "];~%"))))
                   tbl)))
            (when draw-result-only (print-arcs result-tbl)) ;; draw-resut
            (print-arcs draw-tbl)
            )
          (format strm "}~%")
          t))))
  (:write-to-dot (fname &optional result-path (title "output"))
   "write graph structure to dot(graphviz) file
Args:
  fname: filename for output
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
"
   (with-open-file
    (f fname :direction :output)
    (send self :write-to-dot-stream f result-path title))
   t)
  (:write-to-file (basename &optional result-path title (type "pdf"))
   "write graph structure to various type of file
Args:
  basename: basename for output (output filename is 'basename.type')
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
  type: type of output
"
    (let ((dot-fname
           (format nil "~A.dot" basename)))
      (send self :write-to-dot dot-fname result-path title)
      (unix:system (format nil "dot ~A -T~A -o ~A"
                           dot-fname type (format nil "~A.~A" basename type)))
      t))
  (:write-to-pdf (fname &optional result-path
                        (title (string-right-trim ".pdf" fname)))
   "write graph structure to pdf
Args:
  fname: filename for output
  result-path: list of solver-node, it's result of (send solver :solve graph)
  title: title of graph
"
   (when (substringp ".pdf" fname)
     (let ((str (string-right-trim "pdf" fname)))
       (when (= (elt str (1- (length str))) #\.)
         ;; fname finished with '.pdf', remove it
         (setq fname (subseq str 0 (1- (length str))))
         )))
   (send self :write-to-file fname result-path title))
  (:original-draw-mode ()
   "change draw-mode to original mode"
   (send self :draw-both-arc nil)
   (send self :draw-arc-label nil)
   (send self :draw-merged-result nil))
  (:current-draw-mode ()
   "change draw-mode to latest mode"
   (send self :draw-both-arc t)
   (send self :draw-arc-label t)
   (send self :draw-merged-result t))
  (:draw-both-arc (&optional (bothq :both))
   "change draw-mode, if true is set, draw bidirectional arc as two arcs"
   (unless (eq bothq :both)
     (send self :put :draw-both-arc bothq))
   (send self :get :draw-both-arc))
  (:draw-arc-label (&optional (writeq :write))
   "change draw-mode, if true is set, draw label(name) of arcs"
   (unless (eq writeq :write)
     (send self :put :draw-arc-label writeq))
   (send self :get :draw-arc-label))
  (:draw-merged-result (&optional (mergeq :merge))
   "change draw-mode, if true is set, draw result arc as red. if not, draw red arc independently"
   (unless (eq mergeq :merge)
     (send self :put :draw-merged-result mergeq))
   (send self :get :draw-merged-result))
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

