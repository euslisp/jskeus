(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest coerse
  (assert (equal (coerce #f(-2.4 -1.9 -1.1 -0.9 -0.1 0.1 0.9 1.1 1.9 2.4) integer-vector)
		 #i(-2 -2 -1 -1 0 0 1 1 2 2)) "cerse"))

(deftest operator
  (let* ((n 10)
	 (a (instantiate float-vector n))
	 (b (instantiate float-vector n))
	 (c (instantiate float-vector n)) d)
    (dotimes (i n)
      (setf (elt a i) (- (random 200) 100))
      (setf (elt b i) (- (random 200) 100)))
    ;;
    (dotimes (i n)
      (setf (elt c i) (+ (elt a i) (elt b i))))
    (assert (equal (v+ a b) c) "v+")

    (dotimes (i n)
      (setf (elt c i) (- (elt a i) (elt b i))))
    (assert (equal (v- a b) c) "v-")

    (setq d 0)
    (dotimes (i n)
      (incf d (* (elt a i) (elt b i))))
    (assert (equal (v. a b) d) "v.")

    (assert (equal (v* #f(1 2 3) #f(4 5 6)) #f(-3 6 -3))  "v*")
    ))

(deftest func
  (let* ((n 10)
	 (a (instantiate float-vector n))
	 (b (instantiate float-vector n))
	 (c (instantiate float-vector n)) d)
    (dotimes (i n)
      (setf (elt a i) (- (random 200) 100))
      (setf (elt b i) (- (random 200) 100)))
    (setq d (elt a 0))

    (dotimes (i n)
      (setf (elt c i) (* d (elt a i))))
    (assert (equal (scale d a) c) "scale")

    (setq d 0)
    (dotimes (i n)
      (incf d (* (elt a i) (elt a i))))
    (assert (equal (norm2 a) d) "norm2")

    (setq d 0)
    (dotimes (i n)
      (incf d (* (elt a i) (elt a i))))
    (setq d (sqrt d))
    (assert (equal (norm a) d) "norm")

    (assert (eps= (norm (normalize-vector a)) 1.0) "normalize-vector")

    (setq d 0)
    (dotimes (i n)
      (incf d (* (- (elt a i) (elt b i)) (- (elt a i) (elt b i)))))
    (setq d (sqrt d))
    (assert (equal (distance a b) d) "distance")

    (setq d 0)
    (dotimes (i n)
      (incf d (* (- (elt a i) (elt b i)) (- (elt a i) (elt b i)))))
    (assert (equal (distance2 a b) d) "distance2")

    (setq a (instantiate float-vector 4))
    (dotimes (i 4)
      (setf (elt a i) (- (random 200) 100)))
    (setq b (geo::homogenize (geo::homo2normal a)))
    (assert (eps= (/ (abs (v. a b)) (* (norm a) (norm b))) 1.0))

    (setq a (instantiate float-vector 3))
    (dotimes (i 3)
      (setf (elt a i) (- (random 200) 100)))
    (setq b a)
    (dotimes (i 360)
      (setq a (rotate-vector a (deg2rad 1) :x)))
    (assert (eps-v= a b) "rotate-vector")

    (assert (eps-v= (vplus (list a b a b)) (scale 2 (v+ a b))) "vplus")

    (assert (eps-v= (vector-mean (list a b b a)) (scale 0.5 (v+ a b))) "vector-mean")

    ))

(deftest reader
  (let (a f i a2 f2 i2)
    (setq a #a(1 2 3))
    (setq f #f(1 2 3))
    (setq i #i(1 2 3))
    (setq a2 #2a((0 1)(2 3)))
    (setq f2 #2f((0 1)(2 3)))
    (setq i2 #2i((0 1)(2 3)))
    (setq a (make-array (list 2)))
    (setq i (make-array (list 2) :element-type :integer))
    (setq f (make-array (list 2) :element-type :float))
    (setq a2 (make-array (list 2 2)))
    (setq i2 (make-array (list 2 2) :element-type :integer))
    (setq f2 (make-array (list 2 2) :element-type :float))
    ))

(run-all-tests)
(exit)
