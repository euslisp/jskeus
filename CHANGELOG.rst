^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jskeus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2019-01-07)
------------------
* .travis-osx.sh: need install make by brew (`#535 <https://github.com/euslisp/jskeus/issues/535>`_)
* try to run more irt tests on arm machine (`#533 <https://github.com/euslisp/jskeus/issues/533>`_)
  * ubuntu_arm64:trusty takes more than 50 min, so skip irteus-demo.l
  * irteus/nr.c: check rv1/w/f + anorm == anorm with eusfloat_t type
  * irteus/{irtc.c,nr.c}: use double for nr codes
  * irteus/irtutil.l interpolator :pass-time : calculate segment-time from time minus previous time-list, (incf segment-time dt) accumulate errors, specially 32bit machine
  * irteus/test/test-irt-motion.l: run all test with compiled code
  * irteus/test/geo.l: (body+ c1 b d1 c2 d2 c3 d3 c4 d4) could not pass the test on arm 32bit, use truncated version
  * irteus/test/geo.l somehow both 32/64 arm architecture needs to return #f(0 0 0) for small normalized vectors
  * irteus/test/mathtest.l: 9.656790+06 is too large for 32bit machine
  * irteus/test/mathtest.l: add test for pseudo-inverse2 to check if svdcmp failed to converge
  * irteus/test/test-pointcloud.l To test test-pointcloud.l with compiled code requires https://github.com/euslisp/EusLisp/pull/357
* [irtmodel.l]fix bug of :angle-vector for multi-dof joint (`#532 <https://github.com/euslisp/jskeus/issues/532>`_)
  * add test code to check bug of :angle-vector for multi-dof joint `#529 <https://github.com/euslisp/jskeus/issues/529>`_
  * add more test on min-max-joint-table (`#534 <https://github.com/euslisp/jskeus/issues/534>`_)
* travis.{yml,sh}: set travis to fail when gcc outputs compile warnings (`#527 <https://github.com/euslisp/jskeus/issues/527>`_e)
* [irteus/irtviewer.l] support changing floor color. (`#528 <https://github.com/euslisp/jskeus/issues/528>`_ from mmurooka/floor-color
  * [jskeus/doc] add irtviewer.tex and figures for irtviewer manual.
  * [irteus/irtviewer.l] add :draw-origin, :draw-floor, and :floor-color.
  * [irteus/irtviewer.l] support changing floor color.
* remove all compile warning (`#526 <https://github.com/euslisp/jskeus/issues/526>`_)
  * add -DLinux to Makefile.LinuxARM
  * enable debian_arm64:stretch
  * add WFLAGS to Makefile
  * remove all compile warning for eus0
* .travis.yml: add debian:stretch test (`#525 <https://github.com/euslisp/jskeus/issues/525>`_)
  * Fix the compile on Debian stretch. `#524 <https://github.com/euslisp/jskeus/issues/524>`_
  * set debian_arm64:stretch allow_failures
  * Fix the compile on Debian stretch.
    Must like the comment above says, gcc 6 on Debian
    Stretch doesn't like redefining sin, cos, etc and throws
    an error.  Fix this problem by not redefining on Debian Stretch.
* [irtrobot.l] check if target coords is function or coordinates, also work if target coords is list of functions. (`#503 <https://github.com/euslisp/jskeus/issues/503>`_)
  * [irteus/irtmodel.l] Add :xm :ym :zm comment for rotation-axis keyword-argument.
  * [irtrobot.l] allow to use both list and atom of function(s) for target-coords, Fixed `#476 <https://github.com/euslisp/jskeus/issues/476>`_
* add test with compiled lisp (`#522 <https://github.com/euslisp/jskeus/issues/522>`_)
  * .travis.sh: add compiled test
  * remove do-until-key x/10 message, debug (print r) code
  * test/robot-model-usage.l : reduce output message for travis
  * test/irteus-demo.l : reduce output message for travis
  * test/irteus-demo.l: use debug-veiw nil to reduce output message
  * test/full-body-ik.l: add debug-view option
  * test/geo.l: test-body+ test-vector-angle needs irteus to avoid 'outer circuit not found' and 'undefined function c-isnan
  * test/matrix.l  : c-isnan test, disable test-matrix-concatenate-noargs,test-matrix-concatenate-single-matrix,test-matrix-concatenate-multiple-matrices becaues of 'undefined function concatenate-matrix needs irteus
  * test/mathtest.l : diagnoal, minor-matrix, atan2, outer-product-matrix, quaternion, matrix-log, pseudo-inverse, sr-inverse, manipulability, eigen decompose, sv/ql solve, lu-solve2, matrix-determinant, qr/ql-decompose needs irteus
  * test/coords.l : test-makecoords-quaternion needs quaternion2matrix defined in irteus
  * remove test codes which are moved to EusLisp https://github.com/euslisp/EusLisp/pull/316
  * add test with compiled lisp
  * re-define method within defun is not working with compiled code (`#523 <https://github.com/euslisp/jskeus/issues/523>`_)
    - do not run test code on compile, but run when load
* fix for osx and newer compiler (gcc >= 7 OR clang >=9) (`#520 <https://github.com/euslisp/jskeus/issues/520>`_)
  * disable DISPLAY for osx
  * gcc >= 7 OR clang >=9 dislike redefinition of sqrt/cos/fin/fabs ...
  * Disable redefinition of sqrt/cos/... to avoid some errors on macOS
  * fix osx travis
    - remove homebrew/x11, which is deprecated
    - do not run brew update, because brew update compile all module from source and it takes more than 1 hour
    - copy setup process from euslisp/.travis-os.sh
    - do not install x11 server
    - export LIBGL_ALLOW_SOFTWARE=1 suggested by https://api.travis-ci.org/v3/job/403422947/log.txt
    - make -j with 2
    - add exit 0 end of .travis-osx.sh
    - run as script, instead of source
* [doc] Add example when IK target-coords function introduced in `#514 <https://github.com/euslisp/jskeus/issues/514>`_ (`#518 <https://github.com/euslisp/jskeus/issues/518>`_)
  * Add sample of ik using functional target-coords
* Contributors: Chris Lalancette, Guilherme Affonso, Kei Okada, Kentaro Wada, Masaki Murooka, Naoki Hiraoka, Iori Yanokura

1.2.0 (2018-07-19)
------------------
* to use IMPLIB we need run LIBNR first (`#512 <https://github.com/euslisp/jskeus/issues/512>`_, `#513 <https://github.com/euslisp/jskeus/issues/513>`_)
* update function using defun() for https://github.com/euslisp/EusLisp/pull/116 (`#511 <https://github.com/euslisp/jskeus/issues/511>`_)
* Fix invalid indexing in collision link pair calculation (`#461 <https://github.com/euslisp/jskeus/issues/461>`_)
  * increase assert error for [test-samplerobot-walk-pattern-ntimes]
  * add more information (test-samplerobot-walk-pattern-ntimes)
  * returns (i - 1) ??
  * remove (= (length link-list) 1) code, that should be included in subseq logic
  * Add checking of index in collision link pair calculation (`#459 <https://github.com/euslisp/jskeus/issues/459>`_)
  * Add test code to check invalid indexing of collision link pair calculation (https://github.com/euslisp/jskeus/issues/459)
* [irteus/test/geo.l] add test-vector-angle. (`#485 <https://github.com/euslisp/jskeus/issues/485>`_)
* [irteus/irtgeo.l] move :move-coords method from cascaded-coords class to coordinates class. (`#486 <https://github.com/euslisp/jskeus/issues/486>`_)
* add ubuntu:bionic to .travis.yml (`#508 <https://github.com/euslisp/jskeus/issues/508>`_)
  * add ARCHDIR/lib to LD_LIBRARY_PATH
  * gnu gcc 7 complains with re-definition of math.h
  ```
  In file included from irteus/PQP/src/PQP.h:44:0,
  from irteus/CPQP.C:25:
  irteus/PQP/src/PQP_Compile.h: In function 'float sqrt(float)':
  irteus/PQP/src/PQP_Compile.h:47:26: error: 'float sqrt(float)' conflicts with a previous declaration
  inline float sqrt(float x) { return (float)sqrt((double)x); }
  ^
  In file included from /usr/include/c++/7/math.h:36:0,
  from irteus/PQP/src/PQP_Compile.h:46,
  from irteus/PQP/src/PQP.h:44,
  from irteus/CPQP.C:25:
  /usr/include/c++/7/cmath:463:3: note: previous declaration 'constexpr float std::sqrt(float)'
  sqrt(float __x)
  ^~~~
  ```
  * from trusty, use libpng-dev instead of libpng12-dev
  * add ubuntu:bionic to .travis.yml
* set minjerk end position to last given value (`#469 <https://github.com/euslisp/jskeus/issues/469>`_)
* [irteus/test/matrix.l] add test case of rotating around x-axis (`#483 <https://github.com/euslisp/jskeus/issues/483>`_)
* debug :copy-state-to of rikiya-bvh-robot-model to map euscollada-robot correcly (`#437 <https://github.com/euslisp/jskeus/issues/437>`_)
  * add rikiya-file tum and cmu files
  * debug :copy-state-to of rikiya-bvh-robot-model to map euscollada-robot correctly
  * debug :copy-state-to method of rikiya-bvh-robot-model to accept euscollada-robot correctly
* cascaded-link: inverse-kinematics, fix error message both :link-list and :move-target is required (`#435 <https://github.com/euslisp/jskeus/issues/435>`_)
* CMakeList.txt: enable to run compile when .o is removed (`#430 <https://github.com/euslisp/jskeus/issues/430>`_)
* fix load time.l in current directory (`#429 <https://github.com/euslisp/jskeus/issues/429>`_)
* add documentation to :move-coords (`#471 <https://github.com/euslisp/jskeus/issues/471>`_)
* Allow one to specify euslisp repository and branch from command line (`#505 <https://github.com/euslisp/jskeus/issues/505>`_)
* [irtrobot.l] Fix typo, contenious rotatoin -> continuous rotation (`#504 <https://github.com/euslisp/jskeus/issues/504>`_)    
* Add test to check gensym reading. (`#501 <https://github.com/euslisp/jskeus/issues/501>`_)
* Fix typo of 'midrot' function,  "returns .... given two matrix r1 and r2 instead of r1 and r1" (`#498 <https://github.com/euslisp/jskeus/issues/498>`_)
* [test, test-pointcloud] fix eps of significant figure for 32bit system (`#497 <https://github.com/euslisp/jskeus/issues/497>`_)
* [test, test-pointcloud] fix eps for 32bit system (`#495 <https://github.com/euslisp/jskeus/issues/495>`_)
* [irteus/test/geo.l] test distance method of face class. (`#484 <https://github.com/euslisp/jskeus/issues/484>`_)
  * [irteus/test/geo.l] add test-face-distance to check distance method of face class. https://github.com/euslisp/EusLisp/pull/261 problem is checked.
* Fix typo and misunderstanding in README.md (`#488 <https://github.com/euslisp/jskeus/issues/488>`_)
* fixed :convert-to-world in pointcloud (`#490 <https://github.com/euslisp/jskeus/issues/490>`_)
  * [irtpointcloud] fix :convert-to-world is wrong if it was assoced
  * [test, pointcloud] add test for pointcloud
* Fix bug in calculation of :cog-translation-axis (`#481 <https://github.com/euslisp/jskeus/issues/481>`_)
  * [irteus/irtmodel.l] Fix bug of :cog-translation-axis (https://github.com/euslisp/jskeus/pull/481). Do not define case sentence. Use :calc-target-axis-dimension.
  * [irteus/test/test-irt-motion.l] Add test to check :cog-translation-axis dimensions.
* [jskeus/irteus/irtmodel.l][test/joint.l] fix target joint table in :angle-vector and update test (`#478 <https://github.com/euslisp/jskeus/issues/478>`_)
  * [jskeus/irteus/irtmodel.l] fix joint variable of getting target joint min-max-table.
  * [jskeus/irteus/test/joint.l] check whether joint angle overwritten with min-max-table is on the edge of table shape. min-max-table test becomes failure with this update because of https://github.com/euslisp/jskeus/pull/477 issue.
  * [jskeus/irteus/test/joint.l] fix incorrect condition about min-max-table-view in min-max-table test.
  * [jskeus/irteus/test/joint.l] update min and max angle of joint.
  * [jskeus/irteus/test/joint.l] update algortihm to set and check min-max-table matrix value. generated table shape and check result do not change. this update enables to change min or max joint angle to other value.
  * [jskeus/irteus/test/joint.l] fix mistaken idx to access table matrix.
  * [jskeus/irteus/test/joint.l] fix joint for setting table matrix size.
  * [jskeus/irteus/test/joint.l] fix incorrect keyword for setting min /max angle of joint.
  * [jskeus/irteus/test/joint.l] fix color symbol typo.
* [irtsensor.l, camera-model] fix lighting when generating camera images by camera-model (`#475 <https://github.com/euslisp/jskeus/issues/475>`_)
* irtscene.l: remove optional args from :spots (`#462 <https://github.com/euslisp/jskeus/issues/462>`_)
* add x::window-main-one within display code of `:inverse-kinematics`, `:calc-walk-pattern-from-footstep-list` and `:preview-control-dynamics-filter`. So that we can interactively change viewpoint during demo programs(`#457 <https://github.com/euslisp/jskeus/issues/457>`_)
  * demo: add x::window-main-one after :flush or :draw-objects
  * irteus/irtdyna.l: add x::window-main-one in :preview-control-dynamics-filter
  * irteus/irtrobotl.l: add x::window-main-one in :calc-walk-pattern-from-footstep-list
  * irteus/irtmodel.l: add x::window-main-one in :inverse-kinematics with :debug-view t
* Fix typo (segmnet -> segment) in irtutil.l (`#458 <https://github.com/euslisp/jskeus/issues/458>`_)
* circle.yml run apt-get update before apt-get install (`#456 <https://github.com/euslisp/jskeus/issues/456>`_)
* add kbhit() (`#438 <https://github.com/euslisp/jskeus/issues/438>`_)
* [irteus/pqp.l, doc/pqp.tex] Add examples for pqp collision check functions in jmanual. Add documentation string for pqp collision functions. (`#448 <https://github.com/euslisp/jskeus/issues/448>`_)
* .travis.yml : add travis job to run doc generation with BUILD_DOC=true (`#453 <https://github.com/euslisp/jskeus/issues/453>`_)
* [doc, irtmodel] documentation string should be compiled by latex (`#449 <https://github.com/euslisp/jskeus/issues/449>`_)
* [irteus/irtrobot.l,irtmodel.l] Fix bug in https://github.com/euslisp/jskeus/issues/445, add documentation to with-append-root-joint, and fix comment message in :fullbody-inverse-kinematics. (`#446 <https://github.com/euslisp/jskeus/issues/446>`_)
* Fix camera model ( `#433 <https://github.com/euslisp/jskeus/issues/433>`_)
  * irteus/demo/sample-camera-model.l: add sample-robot-camera
  * irteus/demo/sample-robot-model.l: fix camera orientation
  * irteus/irtsensor.l: (defmethod camera-model (:draw-sensor )) use screen of viwing
* add documentation string
  * irtutils.l (`#439 <https://github.com/euslisp/jskeus/issues/439>`_)
  * speed-to-angle/angle-to-speed, joint-velocity,joint-aceleration,joint-torque,max-joint-velocity,ax-joint-torque (`#440 <https://github.com/euslisp/jskeus/issues/440>`_)
* [irtscene.l] error when spot name is empty (`#441 <https://github.com/euslisp/jskeus/issues/441>`_)
* [irteus/kalmanlib.l] adaptive kalman filter demo / cleanup kalman demo (`#426 <https://github.com/euslisp/jskeus/issues/426>`_ from furushchev/kalman
* [irtdyna.l] describe coordinates used in :calc-zmp and :calc-static-balance-point. refer `#415 <https://github.com/euslisp/jskeus/issues/415>`_, `#419 <https://github.com/euslisp/jskeus/issues/419>`_ (`#423 <https://github.com/euslisp/jskeus/issues/423>`_ )
* irtdyna.l: :inertia-tensor fix documentation for tex (`#424 <https://github.com/euslisp/jskeus/issues/424>`_)
* [irteus/irtgraph.l] update, graph output to dot file (add label to arcs) (`#421 <https://github.com/euslisp/jskeus/issues/421>`_)
* Contributors: Guilherme de Campos Affonso, Ryo Koyama, Kei Okada, Masaki Murooka, Masayuki Inaba, Naoya Yamaguchi, Shunichi Nozawa, Yohei Kakiuchi, Yuki Furuta, Iori Yanokura, Éricles Lima, Óscar Carrasco

1.1.0 (2017-03-07)
------------------
* Add vector-variance and covariance-matrix (`#418 <https://github.com/euslisp/jskeus/issues/418>`_)
  * add documentation to vector-variance and covariance-matrix
  * add vector-variance and covariance-matrix
* [irteus.irtscene.l] add :remove-wall for scene-model (`#417 <https://github.com/euslisp/jskeus/issues/417>`_)
* [irteus/irtpointcloud.l] fix bug of :set-color. (`#416 <https://github.com/euslisp/jskeus/issues/416>`_)
* Adapt to moved formulae: homebrew/homebrew-x11 -> euslisp/homebrew-jskeus
  Fix `#412 <https://github.com/euslisp/jskeus/issues/412>`_
* irtgl.l/irtpointcloud.l: add :aarch64 (`#410 <https://github.com/euslisp/jskeus/issues/410>`_)
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Yohei Kakiuchi, Yuki Furuta

1.0.14 (2016-12-30)
-------------------
* update bvh2eus (`#400 <https://github.com/EusLisp/jskeus/issues/400>`_)
  * fix documentation of bvh2eus
  * support :objects keyword for bvh2eus
  * check if the robot has correspond joint
  * add copy-state-to to rikiya-bvh-model
  * add {rikiya/cmu/tmu}-bvh2eus
* add :makecurrent in :draw-on, this will fix https://github.com/euslisp/jskeus/issues/401 (`#402 <https://github.com/EusLisp/jskeus/issues/402>`_ )
* irtrobot.l: :inverse-kinematics-loop-for-look-at : use joint-list from joint of link-liste (`#408 <https://github.com/EusLisp/jskeus/issues/408>`_ )
* load lib/llib/time.l from .so object. Fix https://github.com/euslisp/jskeus/issues/292 (`#409 <https://github.com/EusLisp/jskeus/issues/409>`_ )
* irtviewer.l: Add :makecurrent in :change-background. Fix https://github.com/euslisp/jskeus/issues/404 (`#406 <https://github.com/EusLisp/jskeus/issues/406>`_ )
* Update kalmanlib (`#396 <https://github.com/EusLisp/jskeus/issues/396>`_)
  * [irteus/kalmanlib.l] add kalmanlib sample of accelerated motion
  * [irteus/kalmanlib.l] allow larger dimension of H than A in kalman filter model
  * [irteus/kalmanlib.l] add controller term in model
* add circle.yaml : run make doc in circleci (`#395 <https://github.com/EusLisp/jskeus/issues/395>`_)
* add test/queue.l for https://github.com/euslisp/EusLisp/pull/185 (`#394 <https://github.com/EusLisp/jskeus/issues/394>`_ )
* Contributors: Ryo Koyama, Kei Okada, Shun Hasegawa

1.0.13 (2016-08-02)
-------------------
* add linear/minjerk-interpolator https://github.com/euslisp/jskeus/pull/391
  * add linear/minjerk-interpolator
  * add irteus/test/interpolator.l
  * add doc for interpolators

* mathtest.l : fix wrongly comment out-ed at https://github.com/euslisp/jskeus/pull/364 (https://github.com/euslisp/jskeus/pull/390 )

* add test for eps>, eps<, eps>=, eps<= https://github.com/euslisp/jskeus/pull/385 (#385)
  * mathtest.l : fix wrongly comment out-ed at https://github.com/euslisp/jskeus/pull/364
  * mathtest.l : add test for eps<, eps>, eps<=, eps>=, see section 14 of the manual.pdf
  * mathtest.l : add test for eps= and eps<>

* Enable to set eps for orient-coords-to-axis https://github.com/euslisp/jskeus/pull/392
  * [irteus/irtgeo.l] Enable to set eps for orient-coords-to-axis (*epsilon* is used for acos and this is too big in terms of angle error).

* implement function for transpose image (https://github.com/euslisp/jskeus/pull/388 )
  * [irteus/irtglc.c,irtues/irtext.l,Makefile] add irtglc.c / implement function for transpose image (https://github.com/euslisp/jskeus/pull/388 )
  * [irteus/irtgl.l] add interface for transpose image function in irtglc.c; replace lisp code for transpose image with C code.
  * [irteus/test/rendering.l] add test for transpose
  * [irteus/test/pr2.png] add test image

* irtdyna.l : add condition whther parent is root-link or not in irtdyna.l, and add test for calculate torque when using with-append-root-joint in irteus/test/test-irt-motion.l https://github.com/euslisp/jskeus/pull/382
* irtdyna.l: set analysis level :coords in :calc-torque-from-vel-acc (irtdyna.l) https://github.com/euslisp/jskeus/pull/383

* [irteus/irtrobot.l] Add all-limbs as robot limb information to motion result from calc-walk-pattern (https://github.com/euslisp/jskeus/pull/379 )

* Calculate smooth swing foot rotation for walking motion (https://github.com/euslisp/jskeus/pull/378 )
  * [irtdyna.l] Enable to calculate ZMP from total force/moment when update is t in :calc-zmp
  * [demo/walk-motion.l] Reduce execution time for small robot walking sample by reducing footstep number
  * [demo/walk-motion.l] Fix default rotation-axis to use all t by default.
  * [irtdyna.l] Calculate smooth swing foot rotation for walking motion.

* Fix calculation of swing-leg-proj-coords to be smooth trajectory. (https://github.com/euslisp/jskeus/pull/371 )
  * [irteus/irtdyna.l] Add hoffarbib calculation for swing-leg-proj-coords calculation to smooth trajectory. Previous sigmoig sometimes return discontinuous trajectory.

* Update for stair walk https://github.com/euslisp/jskeus/pull/366
  * [demo/walk-motion.l,test/irteus-demo.l] Add sample for stair climb. Add test for stair-climb and single-support walking.
  * [irtdyna.l] Use sigmoid to smooth z (cog, root, ...) trajectory.

* [irteus/irtdyna.l] Add getting method for robot total inertia-tensor. Add and update documentation strings for mass prop calculation. https://github.com/euslisp/jskeus/pull/363

* add armv8(jessie) returns aarch64 (https://github.com/euslisp/jskeus/pull/364)
  * add arm8(jessie) returns aarch64
  * docker 16.04 does not have sudo installed
  * .travis.yml: add testing on ARM arach for both ubuntu/debian, debian for amd64, osx
  * .travis.sh: do not install tex
  * .travis.yml : allow_failure for osx

* Support parallel Compile
  * compile\_*.log depends on .l files (https://github.com/euslisp/jskeus/pull/375 )
  * irteus/Makefile: support parallel compile (https://github.com/euslisp/jskeus/pull/373)
  * Makefile: use $(MAKE) instaed of make
  * Makefile: irteus-installed depends on eus-installed

* OSX support
  * test/transparent.l: fix for osx, ru_maxrss retuns huge value on osx, so we check if the growth rate of vmrss (https://github.com/euslisp/jskeus/pull/377)
  * [.travis.yml] add .travis-osx.sh for building on OSX (https://github.com/euslisp/jskeus/pull/302 )
  * Test installing jskeus by Homebrew via Push to master (https://github.com/euslisp/jskeus/pull/370 )

* update travis.yml to use 16.04 (https://github.com/euslisp/jskeus/pull/357) 
  * .travis.sh: docker 16.04 does not have sudo nor ptex-bin
  * .travis.yml : test on both 14.04 and 16.04
  * .travis.sh add make for docker
  * .travis.yml: use docker to run travis.sh
  * .travis.sh: verbose apt-get update
  * [.travis.sh, travis.yml] split build file into .travis.sh
  * enable unittest.l cehcking #359
  * update travis.yml to use 14.04
  * [.travis.sh] Add apt-get update before apt-get execution https://github.com/euslisp/jskeus/pull/367

* README.md: make is not installed on plain debian/ubuntu, need to apt-get intall make (https://github.com/euslisp/jskeus/pull/365 )

* Update Testing
  * [irteus/test/vector.l] fix compare NaN  (https://github.com/euslisp/jskeus/pull/361) 
  * test/vector.l: test for nan/info read https://github.com/euslisp/EusLisp/pull/162 (https://github.com/euslisp/EusLisp/pull/354 )
  * Enable unittest.l checking. (https://github.com/euslisp/EusLisp/pull/359 )
  * [irteus/test/unittest.l, .travis.yml] Enable unittest.l checking. Remove removing of failure (https://github.com/jsk-ros-pkg/jsk_roseus/pull/21#issuecomment-205101195) and add neglection of exitting.
  * [irteus/test/unittest.l] Add more print message to debug unittest.l. (https://github.com/euslisp/EusLisp/pull/358)

* Contributors: Furushchev, Kei Okada, Kentaro Wada, Ryo Terasawa, Shunichi Nozawa

1.0.12 (2016-03-20)
-------------------

* Walking for Quadruped Robot (https://github.com/euslisp/jskeus/issues/353)

  * [irteus/demo/walk-motion.l, irteus/test/irteus-demo.l] add a calc-walk-pattern-from-footstep-list test for rotation-axis option
  * [irteus/irtdyna.l] match the order of target-coords and link-list and rotation-axis, translation-axis, thre, rthre and so on

* Fullbody Look at (https://github.com/euslisp/jskeus/issues/351)

  * [jskeus/irteus/demo,test] add test for look-at ik.
  * [jskeus/irteus/demo] add look-at-ik.l

* Speed up access to end-coords (https://github.com/euslisp/jskeus/pull/342)

  * [irteus/irtrobot.l] Fasten access for end-coords and root-link by not using assoc searching

* Update :calc-grasp-matrix (https://github.com/euslisp/jskeus/pull/341)

  * [irteus/test/test-irt-motion.l] Add test code for calc-grasp-matrix
  * [irteus/irtmodel.l] Update :calc-grasp-matrix. Add documentation string and support rotation matrices.

* Add example for irtscene (https://github.com/euslisp/jskeus/pull/340)

  * [irteus/demo/scene.l] add sample code for irtscene

* update Collision codes  (https://github.com/euslisp/jskeus/pull/339)

  * [irteus/irtmodel.l] Enable to change distance-limit for collision-checking
  * [irteus/irtmodel.l] Add comment for what is included in col-list.

* add sample program for virtual joint  (https://github.com/euslisp/jskeus/pull/338)

  * [demo/demo.l,virtual-joints.l,test/irteus-demo.l] Add example for virtual joint and include it in unittest for irt demos.

* add make-fan-cylinder function (https://github.com/euslisp/jskeus/pull/337)

  * [irteus/irtgeo.l] Add make-fan-cylinder function from euslib/jsk/jskgeo.l

* Fix rotation nchange bug of sphere/6dof-joint (https://github.com/euslisp/jskeus/pull/336)

  * [irteus/irtmodel.l] Fix rotation change bug of sphere-joint and 6dof-joint zero orientation setting, tested in the previous commit (test/test-irt-motion.l).
  * [irteus/test/test-irt-motion.l] Add test for zero setting for orientation of 6dof-joint and sphere-joint. If zero vector is set as :joint-angle :relative t, worldrot should not change.

* :calc-static-balance-point returns nan for the 1st time (https://github.com/euslisp/jskeus/issues/330)

  * [irteus/irtrobot.l] Fix bug of omission of update-mass-properties.
  * [irteus/test/test-irt-motion.l] Add test code to check the bug reported in https://github.com/euslisp/jskeus/issues/330#issuecomment-169544613

* Fix coginit bug of calc-walk-pattern-from-footste-list (https://github.com/euslisp/jskeus/issues/286)

  * [irteus/irtrobot.l] Use :centroid method. This is :update-mass-properties and getting :c-til. So, we can update total COG information on initializing of calc-walk-pattern-from-footstep-list.
  * [irteus/test/test-irt-motion.l] Add test code for calc-walk-pattern-from-footstep-list. (https://github.com/euslisp/jskeus/issues/327)

* Support output/input dimension more than 1 for Preview Control (https://github.com/euslisp/jskeus/issues/324)

  * [irteus/irtdyna.l] Support output dimension > 1 and input dimension > 1 for Preview Control classes

* OSX support (https://github.com/euslisp/jskeus/issues/315)

  * Add installing description via homebrew
  * Test HEAD version euslisp which is installed via Homebrew

* Fixed code for displaying objects with hidden-line mode. (https://github.com/euslisp/jskeus/issues/308, https://github.com/euslisp/jskeus/issues/309)

  * [irteus/demo/sample-camera-model.l] add sample for using hidden line mode with camera-model
  * [irteus/irtsensor.l] add :select-drawmode method to camera-model
  * [irteus/irtscene.l] add :(add|remove)-(object(s)|spot(s)) methods
  * [irteus/irtgl.l] fix drawing hidden-line
  * [irteus/irtviewer.l] fix :select-drawmode method in irtviewer

* Enable to clear :ik-draw-on-params (https://github.com/euslisp/jskeus/issues/323)

  * [irtmodel.l] Add explanations for inverse-kinematics visualization.
  * [test/test-irt-motion.l] Add test for :ik-draw-on-params clear check
  * [irteus/irtmodel.l] Clear :ik-draw-on-params at the end of :inverese-kinematics-loop (by garaemon)

* write fk section in doc (https://github.com/euslisp/jskeus/issues/325)
  * [doc/irtmodel.tex] add fk section in doc
  * [doc/irtmodel.tex] add sample code for updating :analysis-level in manual

* irtmodel, disdyna : misc updates

  * [irteus/irtmodel.l] Check additional-check function if exists regardless of success flag. (https://github.com/euslisp/jskeus/pull/345)
  * [irteus/irtdyna.l] Enable to set jacobi from outside of :calc-torque-from-ext-wrenches (https://github.com/euslisp/jskeus/pull/344)
  * [irteus/irtmodel.l] Remove unnecessary calculation of target joint dimension (https://github.com/euslisp/jskeus/pull/343)
  * [irteus/irtmodel.l] Remove unused local variables (https://github.com/euslisp/jskeus/pull/339)
  * [jskeus/irtmodel.l] Add comment about joint order for :calc-jacobian-from-link-list (https://github.com/euslisp/jskeus/pull/335)
  * [irteus/irtdyna.l] Add mass properties calculation doc. (https://github.com/euslisp/jskeus/issues/328)
  * [doc/irtmodel.tex] Fix typo in doc/irtmodel.tex (https://github.com/euslisp/jskeus/issues/300)
  * [irteus/demo/sample-camera-model.l] fix (sample-get-camera-image-2) https://github.com/euslisp/jskeus/issues/268
  * [irteus/test/mathtest.l] fix random with random state (https://github.com/euslisp/jskeus/issues/298)

* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Kohei Kimura, Masaki Murooka, Shunichi Nozawa, Yohei Kakiuchi, Eisoku Kuroiwa,  Shinaro Noda

1.0.11 (2015-11-02)
-------------------

* Misc updates

  * README.md : re-organize documents, put all export information to the end of the page
  * README.md: update to 14.04
  * Added Gitter badge
  * add slack notification

* irtsensor.l : update get-image

  * irtsensor.l: add method documents
  * [irtsensor.l] fix #294 for correct fx/fy
  * [test/robot-model-usage.l] add test for sample-get-camera-image, disabled for now
  * [demo/sample-camera-model.l] add sample-get-camera-image-1 and sample-get-camera-image-2
  * irtsensor.l: support (&optinal cv) for create-viewer
  * [irtsensor.l] overwrite default perspective view using :newprojection
  * [test/robot-model-usage.l] add test for :ray and :screen-point of camera class of samplerobot
  * add test for :ray and :screen-point of camera class
  * fix image position from models
  * add :create-viewer method to camera-model
  * add sample for getting image and pointcloud from camera sensor
  * [irtsensor] fix get image and pointcloud from camera model

* add test for random and make-random-state

  * [irteus/test/mathtest.l] add test for random / make-random-state

* irtrobot : update fullbody ik

  * [irteus/irtrobot.l] Use cog-null-space as nil in  :fullbody-inverse-kinematics by default.
  * [irtrobot.l] Add comments and check force, moment, target-coords checking
  * [irteus/irtrobot.l] Calculate ext-wrench torque from total-wrench
  * [irteus/irtmodel.l] Fix cog translation axis t
  * [irteus/irtmodel.l, irtrobot.l] Reduce default min-loop for :fullbody-inveser-kinematics and :inverse-kinematics-for-closed-loop-forward-kinematics
  * [irtmodel.l] Add usec to ik log file name to prevent conflict of file names called within one second.
  * [test/test-irt-motion.l] Add test for new ik success/fail file and debug log file
  * [irtdyna.l, irtmodel.l] Add ik debug information log including max loop count and target error transition.

* irtrobot : update footstep

  * [irteus/irtrobot.l] fix unit system of footstep-parameter
  * [irteus/irtrobot.l] fix a step-count-method for y-axis in go-pos-params->footstep-list
  * [irteus/demo] use go-pos-quadruped-params->footstep-list fucntion in order to get foot step list in quadruped walking motion
  * [irteus] add a foot step generator function for quadruped walking
  * [irteus/irtrobot.l, irteus/test/test-irt-motion.l] Add static balance point method and test for it.
  * [irteus/irtrobot.l] Align default limbs based on order of force-sensors

* irtrobot : support-polygon

  * [irteus/irtrobot.l, irteus/demo/crank-motion.l] Modify  support-polygon method to support convex hull of given name and    add example to crank-motion (#263)

* irtgl.l :

  * add :make-pqpmodel method to glvertices

* irtgeo.l :

  * Add :worldcoords method to line to objects in irtviewer

* euspng.c :

  * Do not print debug message when writing a png file

* irtmath.l :

  * [irteus/test/matrix.l] Add test codes for concatenate-matrix functions which check arguments and concatenation results
  * [irteus/irtmath.l] Add concatenate matrix functions moved from euslib/jsk/jsk.l.

* Contributors: Eisoku Kuroiwa, Yuki Furuta, Kei Okada, Ryo KOYAMA, Ryohei Ueda, Shunichi Nozawa, The Gitter Badger, Yohei Kakiuchi

1.0.10 (2015-08-18)
-------------------
* [irteus/test/time.l] add test for time add/subtruct (https://github.com/euslisp/EusLisp/pull/128)
* [irteus/kalmanlib.l] add adaptive flag
* [irteus/kalmanlib.l] add show P_k method
* [irteus/test/string.l] add test for url encoding
* .update-doc.sh:  skip contents, commit png/jpg files
* [irteus/demo/*.l] Use ;; at the begenning of print message to escape documentation from demo function usage.
* [irteus/demo/closed-loop.l, special-joints.l, walk-motion.l] Add print message for demo functions.
* [jskeus/irteus/test/character.l] add test for character
* [irteus/test/number.l] add test for eus number
* [irteus/irtmodel.l] Make pqpmodel in :init-ending
* [irteus/irtmodel.l] Add check collision argument to :inverse-kinematics and nil by default according to https://github.com/euslisp/jskeus/issues/95
* (irtpointcloud) fix removing global variables https://github.com/euslisp/jskeus/pull/223#issuecomment-120209460

* irteus/irtdyna (preview controller)

  * [irteus/irtdyna.l] Fix documentation string to avoid eus documentation tex error
  * [demo/walk-motion.l, irtdyna.l] Rename preview controller cog generator class
  * [demo/walk-motion.l, test/irteus-demo.l] Update preview control example (long walk, impulsive force, liear zmp transition).
  * [irteus/irtdyna.l] Add documentation and use extended-preview-controller for COG generator by default.
  * [irteus/demo/walk-motion.l, irteus/irtdyna.l] Add :pass-preview-controller to add reference list
  * [irteus/irtdyna.l] Return cart zmp
  * [demo/walk-motion.l, irtdyna.l, test/irteus-demo.l] Update class names. Rename old preview-control => preview-control-cogxy-trajectory-generator.
  * [irteus/demo/walk-motion.l, irteus/irtdyna.l] Use preview controller classes and remove extended-preview-control
  * [irteus/irtdyna.l] Use preview-controller-base queue value directly and remove preview-dynamics-filter class
  * [irteus/irtdyna.l] Fix indent
  * [irteus/irtdyna.l] remove duplicated codes
  * [irteus/irtdyna.l] Move queue handling and initialization and finalization check to preview-controller-base
  * [irteus/irtdyna.l] Add queue data to preview-controller-base class
  * [irteus/irtdyna.l] Update state getter methods
  * [irteus/irtdyna.l] Enable to select queue initalization
  * [irteus/irtdyna.l] Add extended-preview-control-base class
  * [irteus/irtdyna.l] Update preview-controller-base updating and add methods for getter.
  * [demo/walk-motion.l] Update quad sample. Especially in the :go-backward-over nil case, target cog was infeasible
  * [irteus/demo/sample-robot-model.l] Inrease joint range for shoulder pitch and crotch pitch according to existent real robot joint ranges.
  * [irteus/irtmodel.l, irteus/irtrobot.l] Do not use lambda function for additional-jacobi and additional-vel by default.
  * [irteus/demo/walk-motion.l, irteus/test/irteus-demo.l] Add test for both preview-control and extended-preview-control
  * [irteus/irtdyna.l] Add base class for preview controller which
    does not include cog and zmp

* irteus/irtgl.l (glvertices)

  * add write-wrl-from-glvertices function
  * fix glbody, same name of slot veriable and class
  * add :convert-to-world method to glvertices

* irteus/irtmodel.l

  * [irteus/irtmodel.l] Use dump-command as :fail-only by default.
  * [irteus/irtmodel.l] Set success flag in one line
  * [test/test-irt-motion.l] Add unittest for dump-command
  * [irteus/irtmodel.l] Introduce dump-command as mode according to discussion in https://github.com/euslisp/jskeus/commit/8f9e79e4f24b86cc66fdf3d4bec1a71b878b099e#commitcomment-12334678

* irteus/irtmodel.l closed-loop

  * [irteus/irtmodel.l, irteus/demo/special-joints.l] Add method to calculate velocity for interlocking joint constraint. Use it in sample program.
  * [irteus/irtrobot.l] Remove unused arguments.
  * [irteus/irtrobot.l] Reduce too match mass properties calculation.
  * [demo/closed-loop.l, demo/special-joints.l] Fix print usage style to match demo.l style.
  * [irteus/demo/demo.l, irteus/test/irteus-demo.l] Add closed-loop sample and special-joint sample to demo.l and unittest.
  * [irteus/demo/special-joints.l] Add special joints examples. Currently, interlocking joint are defined.
  * [irteus/irtmodel.l] Add interlocking joint methods.
  * [irteus/irtrobot.l] Fix additional-vel to argument
  * add argument cog-null-space. set cog jacobian to additional-jacobi in :fullbody-inverse-kinematics
  * add arguments additional-jacobi and additional-vel to cascaded-link :inverse-kinematics
  * [irteus/test/test-irt-motion.l] Add test program for check-collision

* Contributors: Yuki Furuta, Kamada Hitoshi, Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Masaki Murooka

1.0.9 (2015-07-09)
------------------
* irtpointcloud.l: impliment :append methods on pointcloud

* irtgl.l:

  * fix transparent method on glvertices
  * add :mirror-axis method to glvertices

* irtmodel.l:

  * Set (/ stop 10) as min-loop default value not to change ik behavior
  * Update documentations for :inverse-kinematics-loop and add  documentations for min-loop argument
   * Remove unncessary loop checking and add min-loop argument  discussed in https://github.com/euslisp/jskeus/issues/107

* Contributors: Kei Okada, Shunichi Nozawa, Yohei Kakiuchi

1.0.8 (2015-06-19)
------------------
* [README.md] add documents of null-space-ik

  * downsized null-space-ik.png
  * added null-space-ik sample in README.md
  * added null-space-ik sample image
  * Update README.md

* [irtrobot] fix torque-vector for those who does not have two leg, ex) pepper
* [irteus/test/const.l] add code to check intern/shadow/defconst
* [irteus/irtrobot.l] fix :look-at problem (https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/72)

  * fix :look-at, check othre direction
  * add test code for :look-at
  * check if look-at try to across non-valid joint angle
  * set +-150 for neck-p range

* Contributors: Kazuhiro Sasabuchi, Kei Okada

1.0.7 (2015-06-11)
------------------

* update documentation

  * [doc/{Makefile,jmanual.tex}] fix for compile on 14.04 too
  * [irteus/irtgl.l] add documentation string to glvertices
  * [irteus/irtviewer.l] add more comment on make-irtviewer

* irtgl update

  * [irteus/irtgl.l] add flat shader mode to glvertices and add shader mode option to :calc-normals
  * [irteus/irtgl.l] add glbody, which is copied from collada-body of https://github.com/jsk-ros-pkg/jsk_model_tools/blob/master/euscollada/src/collada2eus.cpp#L1196

* misc updates

  * [irteus/irtrobot.l] fix for one leg robot (for example, pepper)
  * [irteus/test/object.l] fix test conditoin, not sure why but previous commit needs this modification
  * [.travis] update doc to jsk-ros-pkg/euslisp-docs

* Contributors: Kei Okada, Yohei Kakiuchi

1.0.6 (2015-04-23)
------------------

* fix minor bugs

  * [irteus/PQP/Makefile.LinuxARM] add -fPIC for arm
  * [Makefile] add .PHONE: doc
  * [irtrobot.l] Fix initial refzmp pos.

* Contributors: Kei Okada, Shunichi Nozawa

1.0.5 (2015-04-22)
------------------

* ARM suppoort

  * [Makefile.LinuxARM] add -fPIC for arm
  * [Makefile] fix to work with armv7l

* [irtrobot.l/irtdyna.l] support :ik-thre and :ik-rthre keyword for :calc-walk-pattern-from-footstep-list
* [irtgraph.l] Add :debug keyword to :pop-from-open-list for consistency in API
* [irtrobot.l] Update calculation of sole polygon (do not use end-coords)

* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa

1.0.4 (2015-03-27)
------------------
* add closed-loop support

 * [demo/closed-loop.l] Add sample program for closed-loop robots
 * [irtrobot.l] Add example method to generate default link function such as make-default-robot-link function
 * [irtmodel.l] Add inverese-kinematics method for closed loop forward kinematics
 * [irtmodel.l] Add end-coords-list for list of end-coords in  cascaded-link slots

* make support-polygon in init-endinghttps://github.com/euslisp/jskeus/pull/177/files

 * [irtrobot.l] :make-sole-polygon  Trace all descendants bodies (fix for hrp4r and ystleg)
 * [irtrobot.l] :init-ending Add support polygon methods and make support polygon in :init-ending if legged robots

* Utility function to choose good color for 10 and 20 categories https://github.com/euslisp/jskeus/pull/178

 * [irtutil.l] hvs2rgv set default value for i, s
 * [irtgl.l] find-color: add new rule, float-vecto [0-1], list [0-255]
 * [irtutil.l] add color-category10, color-category20: Utility function to choose good color for 10 and 20 categories and add  function to convert from hex color to colormaterial

* misc updates

 * [irteus/nr.c] add tred2, this fixed https://github.com/euslisp/jskeus/issues/186
 * [test/mathtest.l] add mathtest.l
 * [irtrobot.l] :calc-walk-pattern-from-foot-step-list Enable to choose  use :calc-zmp or not. Without calc-zmp, we can calculate pattern  fast.
 * [irtrobot.l] :print-vector-for-robot-limb / Add print vector method documentation
 * [irtgeo.l] quaternion-from-two-vector / Add function to compute quaternion from two vectors
 * [package.xml] removed, it is now stored in release repository
   https://github.com/tork-a/jskeus-release/tree/master/patches

* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa

1.0.3 (2015-02-22)
------------------
* [PQP.cpp, euspng.c] cast for printf
* [irteus/irtgeo.c] inlucde math.c for function prototype
* output euscomp results to dev/null
* add test code for vplus/vector-mean
* [irteus/irtmath.l] add lms/lmeds and sv-decompose from euslib
* [irtc.c] add ql-decompose and qr-decompose
* Check only {name} and {name}-robot in maek-robot-model-from-name
* Add make-robot-model-from-name function to create instance of
  robot-model from the name of the robot
* Disable DISPLAY environmental variable during compilation
* Contributors: Kei Okada, Ryohei Ueda

1.0.2 (2015-01-13)
------------------
* Set ${EUSDIR}/irteus as symlink
* Move plot joint min max function to irtmodel.l and define it as method
* Contributors: Kei Okada, Shunichi Nozawa

1.0.1 (2014-12-22)
------------------
* Fix for when euslisp is already installed as deb, if objdir is given from upper script, we use them
* use INSTALL{BIN,LIB,OBJ}DIR  and IRTEUSDIR when euslisp and jskeus is separately installed
* Contributors: Kei Okada

1.0.0 (2014-12-22)
------------------

* (irtdyna, irtrobot, walk-motion) : Fix :refzmp->:zmp
* (irteus-demo.l, walk-motion.l) : Add PrevewController testing and update plotting
* (irtviewer.l) : Resize viewport when irtviewer is resized.
* change :faces in glvertices
* fix: update README.md (modify the instraction to use jskeus in Raspberry Pi)
* add document to irtgl.l
* add utility function for creating glvetices from faceset and list of faces
* Merge pull request #142 from k-okada/add_warn_quaternion2matrix
* fix irtviewer light
* add document calc-jacobian-from-link-list
* (irtrobot) : Add documentation string for :calc-force-from-joint-torque
* add kalmanlib.l moved from euslib/jsk
* add random-gauss and gaussian-random (moved from jsk.l)
* Update README.md, fix according to https://github.com/euslisp/jskeus/issues/132
* add test code for bignum (multiple and devide), [ euslisp/EusLisp/issues/62 ]
* (irtpointcloud) : Fix order of documentation string
* add document to irtpointcloud.l
* (irtrobot, irtdyna, walk-motion) : Use :name instead of plist for footstep l/r
* (irtmodel.tex) : Remove sample description for :solve-ik
* (sample-arm-model, hanoi-arm) : Use :inverse-kinematics instead of :solve-ik (according to https://github.com/euslisp/jskeus/issues/125#issuecomment-54590070)
* (irtrobot) : Update irtrobot methods documentation string
* update document strings
* fix order of author
* use travis to generate pdf and htmls, add texlive-latex-base ptex-bin latex2html nkf ebb to install
* add jmanual.pdf jmanual.dvi
* add documentation tools
* add deftest for reader
* (sample-*-model) : Define sample-arm-robot and sample-hand-robot as robot-model subclass
* (sample-multidof-arm-model) : Enable to set joint class for sample-multidof-arm-robot
* (irtmodel.l) : Add comments for addtional-check argument
* (null-space-ik) : Add additional-check for null-space example to wait for being enough distance
* (test-irt-motion.l) : Add test codes for calc-torque with external force and moment
* (irtmodel.l, test-irt-motion.l) : Update dump ik fail log to escape all links and list to fix https://github.com/jsk-ros-pkg/jsk_roseus/issues/138 and fix bug in move-target or target-coords
* (test-irt-motion) : Clear ik fail log file for one test
* (irtmodel.l, test-irt-motion.l) : Fix link dump for move-target and search link included in :links and add test code
* (test-irt-motion.l) : Set *sample-robot* instead of *robot* to avoid conflict
* (.travis.ymml) : Add apt-get update to fix apt-get error reported in https://github.com/euslisp/jskeus/pull/101
* (irtmodel.l, test-irt-motion.l) : Escape string link name and add test for string-name case
* (irtmodel.l, test-irt-motion.l) : Fix link access in ik fail log and add test code for it. This bug is reported in https://github.com/jsk-ros-pkg/jsk_roseus/issues/139
* owverwrite face-normal-vector, see https://github.com/euslisp/EusLisp/pull/21
* irteus/test/geo.l: add test code for geometry functions (https://github.com/euslisp/EusLisp/pull/21)
* (irtdyna.l, test-irt-motion.l) Fix bug of :cog-convergence-check and add test codes
* (irtmodel.l, joint.l) Add methods to make joint-min-max-table based on collision check and update test codes
* (joint.l) Execute test even if display is not found
* (joint.l) Add min-max violation test ;; Update joint.l to replace magic number by min-angle or max-angle
* (all-robots-objects.l) Add unittest for scene models corresponding to https://github.com/euslisp/EusLisp/pull/29
* revert codes for collision model making according to https://github.com/euslisp/jskeus/pull/93 and https://github.com/jsk-ros-pkg/jsk_model_tools/pull/46
* (jsk-ros-pkg/jsk_model_tools/issues/41) irtrobot.l : move codes for collision model from euscollada-robot*.l
* (jsk-ros-pkg/jsk_model_tools/issues/18) irtrobot.l, test/robot-model-usage.l, demo/sample-robot-model.l : add sensor accessosr and test codes ;; original PR is euslisp/jskeus/pull/72
* Update README.md, install git instead of subversion
* add test code for multidof joints ;; omniwheel-joint, sphere-joint, and 6dof-joint ;; these test code are migrated from euslib/demo/ik/ik-test.l
* check all ik solved in irteus-demo.l walk test
* fix unsolved dual-arm-ik in irteus-demo.l by fix-leg-to-coords ;; replace deprecate fix-leg codes by :fix-leg-to-coords
* find test codes in irteus/test directory to reduce description in .travis.yml according to the discussion in https://github.com/euslisp/jskeus/commit/83432a3d75c950481e9f3079b077f47f73c1059dp ;; currently unittest.l is neglected because it does not work
* add test code for irteus/demo into irteus-demo.l and remove direct writing of test lines in .travis.yml according to discussion in https://github.com/euslisp/jskeus/commit/83432a3d75c950481e9f3079b077f47f73c1059d
* add democode for null space ik (#83)
* generate tempolary image files to tempolary directory if x::display is available ;; do not use rospack
* move test codes in euslisp/test ;; move test-irtviewer.test and test-euslisp.test
* reduce output of crank-motion ik debug-view message
* update .travis.yaml to use irteus/test/irteus-demo.l to reduce code for do-until-key overwriting
* move test codes from jsk_roseus/euslisp/test according to discussion in https://github.com/euslisp/jskeus/issues/78
* add single support starting and ending for calc-walk-pattern-from-footstep-list
* use end-coords-list and contact states instead of swing-leg-coords and support-leg-coords
* Travis passed and separation of file has been completed.
  https://github.com/euslisp/jskeus/pull/73
* Add sample arm robot which we can configure links and joints parameters.
* update ik parameter for walk sample according to mass update of sample robot
* enable to set weight for samplerobot ;; update default weight according Japanese Male data
* enable to configure torso and waist length
* convert to float because new-weight might be int and (eps= int float) does not work
* add contact states and end-coords to return values from walking generator
* set COG-Z to enable to generate up-down walking
* check gcc -dumpmachine for deb build
* try to download euslisp repository for 10 times, with GIT_SSL_NO_VERIFY=true
* add argument to specify axis for angular momentum jacobian ;; calculate COG around angular momentum by default
* add update-mass-property argument to reduce too much calling of mass property propergation
* fix index of starting point of rotation component in inertia matrix
* enable to change translation-axis for cog jacobian ;; enable to control cog x,y,z
* use git rev-parse --short HEAD instaed of svnversion, see Issue #58
* Add instraction to install more fonts in README.md.
* add .travis.yml
* fix for moving to github
* Create README.md
  add images for README.md
* comment out :expand-vertices
* decrease torellance of noramlize-vector zero division check because original 1.0e-5 is too large ;; 1.0e-20 is sufficient both for 32bit os and 64bit os (in 32bit os, exponent parts in floating-point can be manage 10^36 order)
* update generate-histogram-hs
* update ;; reduce duplicate codes in difference-cog-position
* fix wrong size of indices on :convert-to-faces
* fix to use eps ;; use 1.0e-5 which is previous version's value
* enable to set normalize vector zero division check ;; reduce normalize-vector torellance in matrix-log because default threshold 1e-5 is too large for radian representation
* check 0 division reported by kuroiwa
* use limb-cop-fz-list2 instead of limb-cop-fz-list reported by kuroiwa
* merge reference zmp list to robot state list
* update robot state list
* add example for preview control dynamics filter
* fix calculation of torellance
* fix generate histogram function
* reduce torelance of eps= for rotation of sphere-joint and 6dof-joint
* add generate-histogram method to irtpointcloud
* support linear-joint in joint vel acc calculation
* fix rotation difference ;; reduce threshold of rotation difference ;; this should be implemented in :difference-rotation
* calc root link velocity and acceleration considring spatial velocity formulation
* just fix indent
* use 'if' instead of 'return-from'
* remove I-J#J which is not used in :collision-avoidance
* remove x::wind-w-main-one ;; enable to use x::wind-w-main-one by user
* add :calc-vel-for-cog
* check collicion for all link which distance <= avoid-collision-distance ;; at previous revision, a link which has min-distance was checked. But, in the case that several links ware near the min-distance, min-distance links ware frequently switched and that cause joint velocity discontinuousness
* add darwin example and fix comment
* remove move centorid on foot ;; add limitation of rotation
* add move-centroid-on-foot in calculating foot reachability
* add endlink I-til setting
* remove nil in limb-cop-fz-list reported by kuroiwa
* fix typo ;; end-coorde -> end-coords
* enable to set periodic-time
* add preview control example ;; add comment to plot on gnuplot
* add piped-fork function which returns list
* add comments to specify units
* remove unused zmp-z argument
* use calc-cop-from-force-moment
* add wrench-vector wrench-list conversion
* add return-all-values to return both fz and cop
* fix indent in calc-cop-from-force-moment
* add method to calc worldcoords cop
* add weight for calc contact force
* set default-zmp-offsets based on all-limbs
* use get-limbs-zmp instead of get-limbs-zmp-list
* fix initialize of refzmp-next
* increase loop count for solving riccati equation
* remove temporary debug message
* fix selecting of start leg
* return pattern list
* add crawl example
* add all-limbs arguments and support crawl walk
* use get-limbs-zmp-list
* use rest of limbs
* fix order of joints and set list of transform-coords
* add :calc-torque-from-ext-wrenches
* enable to set external force and moment in :calc-torque method
* set default total wrench
* add calc-contact-wrenches-from-total-wrench and use it in :torque-vector
* add comment for unit system
* fix position of calculation of :weight and fix indent
* use dynamics in calculating zmp
* enable to select statics or dynamics ;; currently default = statics according to previous specification
* remove print in :calc-torque
* set vel and acc from av and root-coords in calc-torque fix calc-torque in crank-motion sample
* add root-spacial-velocity and root-angular-velocity arguments
* add calculation of root-angular-velocity and root-spacial-velocity
* add root-angular-acceleration and root-spacial-acceleration
* separate calc-root-coords-vel-acc-from-pos and calc-av-vel-acc-from-pos
* enable to return ik return value for walking angle-vector solution
* add go-backward-over mode
* fix orientation calculation ;; add additional-nspace-list
* just fix indent
* use absolute-p version 6dof-joint
* add absolute-p for 6dof-joint
* enable to set limbs
* fix calling of function for additional-weight-list and additional-nspace-list
* add joint-angle-limit-nspace-for-6dof which is experimental currently
* multiply weight for additional nspace
* fix order of null-space setting
* flush starndard-input in do-until-key to use several do-until-key in one function
* add dif-pos-ratio and dif-rot-ratio only for velocity , see issue #261
* revert wrong commit (r996): add dif-pos-ratio, and dif-rot-ratio option, see Issue #261
* add dif-pos-ratio, and dif-rot-ratio option, see Issue #261
* return float-vector because angle-vector should be represented as float-vector
* fix dispay det(JJt) not det(J), for non square matrix
* add min/max, matrix-determinant for debug-view
* do nothing when x::window-main-one is called in headless system, Issue #46
* create irtviewer-dummy for headless system, Issue #33
* check existence of objs
* add interpolation of joint-angle from min-max-table according to [#43]
* fix bug of defun ) shortage
* output ik-failed, see issue #42 (https://sourceforge.net/p/jskeus/tickets/42/)
* find joint value with in joint limit, in case of joint is out of limit, http://sourceforge.net/p/jskeus/tickets/43/
* remove debug codefix :angle-vector consider mutual effect ,add :joint-min-max-table-{min/max}-angle, add min-max-table for rotationa joint, see #43
* fix :angle-vector consider mutual effect ,add :joint-min-max-table-{min/max}-angle, add min-max-table for rotationa joint, see #43
* clean up code, add :joint-min-max-table-{min/max}-angle, add min-max-table for rotationa joint, see #255
* add min-max-table for rotationa joint, see #255
* extract root-link coordinates as :root-coords ;; (send robot :worldcoords) != root-coords for some euscollada robots
* rename :root-link -> :root-coords because of mis naming (typo)
* add :draw-floor and :draw-origin option in make-irtviewer
* update :cog-convergence-check to support numberp, functionp, vectorp for centroid-thre argument
* fix null-space calculation
* use additional-nspace-list instead of null-space
* add quad walking sample
* support quad walkint
* add root-link-virtual-joint-weight arguments and init-pose-function
* support :get-limbs-zmp, :get-counter-footstep-limbs
* enable to set list footsteps
* add read-char-case which can impelement y-or-n-p
* add name arguments to bumpser-sensor
* support specifying wrt in convert-to-faces
* impliment :glvertices for getting single mesh
* update to use additional-weight-list
* add init-pose to robot-model
* add torque-ratio-vector to cascaded-link
* fix face direction (euslisp use cw)
* change back face color on glvertices
* use :set-color instead of :put :face-color
* use set-color instead of :put face-color
* add error for missing face-color
* export eus2collada function to user package
* suppor conversion of body or bodyset
* add :move-coords method to cascaded-coords
* add calc-zmp-from-forces-moments method
* check existence of index of additional-nspace-list or additional-weight-list
* add macro to re-direct output and error output
* add bench macro to measure time using mtimer
* support 6dof joint for additional nspace and weight
* add additional-nspace-list and additional-weight-list to set nspace or weight of specific joints
* fix: moved link coords while computing inertia frame
* add print-vector-for-robot-limb
* remove debug print
* add values of mass, inertia and mass_frame to collada file in irtcollada.l [#38]
* add sid to collada node for removing root_node in irtcollada.l, [#38]
* add library_physics_models and library_physics_scene in irtcollada.l, but now adding dummy mass and inertia [#38]
* swap order of <articulated_system> in irtcollada.l [#39]
* rename attribute kinsystem -> robot1_kinematics, rename libarticulated_systems.kinScene_libarticulated_systems.kinScene -> robot1_motion,add attribute id=asystems to library_articulated_systems, in irtcollada.l [#39]
* rename attribute kinsystem_motion -> robot1_motion, inst_kinsystem -> robot1_motion_inst, kinsystem_inst -> kmodel1_inst in irtcollada.l [#39]
* update attribute names in irtcollada.l [#39]
* modify Makefile.Cygwin for the latest Cygwin gcc
* gcc on cygwin can't pass option to linker
* add Makefile for ARM (raspberry pi, debian)
* add Makefile for ARM (raspberry pi, debian)
* rename functions for walk-motion
* support small robot in walk-motion examples
* increase cog-gain in :calc-walk-pattern-from-footstep-list
* enable to set ratio parameter
* add samples for robots in eus/models robots
* fix: copy curvature in filter-with-indices
* fix: resizing height/width when filtering at irtpointcloud
* do not use angle-limit for rotational-joint [#25], [#29]
* fix copy height and width in pointcloud :copy-from
* add transparent option to :set-color method
* use centroid method in drawing cenroid pos
* add :set-color method to glvertices for using overwrighting object color
* update :append-vertices
* add move-joints-hook to configure FK's root-link
* add documentation string for move-centroid-on-foot
* clean up delete-displaylist-id code [#26]
* fix memory leak on overwriting displaylist-id without release old one
* update glvertices
* add debug info to Makefile
* set doc/latex, doc/jlatex directory as externals and use symlink to under euslisp/jskeus directory
* use relative path for EUSDIR/irteus in case that we need to install euslisp
* remove unsupported directories for non-svn environment
* check if unsupported directories is checkout from svn
* fix drawing glvertices on 32bit environment
* check if wget/svn exists
* set eus/modles to externals
* use adequate glcontext for each viewer ;; at previous version, single euslisp object cannot be displayed in multiple irtviewer
* remove print in joint method
* change min/max limit of infinite rotational joint from 180 to 270, see [#25]
* add :expand-vertices method to glvertices
* check head link existence
* separate :look-at method because of this method is too long for using in :limb method
* add remove-nan option to :point-list
* set link-list by default, which is discussed in ticket [#20]
* look-at-hand supports :rarm, :larm, :arms and '(:rarm :larm)[#22]
* support vector and coordinates in look-at-target [#21]
* revert make-cube and make-cylinder, use :translate-vertices
* add overwrite functions make-cube and make-cylinder with :origin-coords method
* add walk-motion to demo.l
* implement :clear-display-id method to glvertices
* remove unused resetting
* enable to set init-xk from preview-controller
* add go-pos param method
* add walk-motion example
* remove jsk codes and :calc-walk-pattern-from-footstep-list
* add preview control and gait generator class
* enable to set target-coords for move-centroid-on-foot
* add :view-name keyword to set :name for viewer
* update svn:externals use https
* remove eus/lib/llib from svn:externals
* upgrade sourceforge
* upgrade euslisp repository
* fix pixel format for texture image
* add code for drawing texture on glvertices
* add implementation for :calc-normals on glvertices
* add methods for creating faces to glvertices
* add method :change-background for changing background color
* :foot-midcoords moved from irt_proposals and add comments
* add some filters for pointcloud
* add fix-leg-to-coords and move-centroid-on-foot to robot-model
* push (:sphere r) to :csg slot
* add documentation string for joint-angle methods
* remove global variable
* add :get-image method to camera-model to getting image and depth
* add keyword :fill to adding blank points and colors
* remove global variable
* add methods to glvertices
* return vertices on bounding box
* fix: drawing glvertices
* add glvertices for displaying triangle mesh in OpenGL
* add curvatures to pointcloud
* fix: width and height must be set
* change: directory for architectures should not be needed in advance
* fix for mac/homebrew #3576504
* fix previous commit
* use png_get_ interface functions for png > 1.5
* fix bashrc.eus message for Cygwin
* uname -s never returns Linux32
* use sed to strip _NT-6.1...
* add header message
* Rm command must be called with -f option., by mkojima
* enable to set sample robot name
* fix typo ;; ret -> rot
* add joint-list argument to set order of joint for rbrain robots
* check ret = nil ;; euslisp's rotation-matrix returns nil ;; euslib/jsk/jsk.l's rotation-matrix is overwrited not to return nil
* mvoe :draw-torque which can be defined for cascaded-link instead of robot-model
* fix height of current-centroid-pos drawing and change color
* enable to draw float-vectors in ik result drawing ;; draw target-centroid-pos and current-centroid-pos
* remove unused viewer :clear for ik debug
* add do-until-key-with-check and re-define do-until-key using it ;; add do-until-key-with-timer
* add color conversion (rgb <-> his) functions
* remove unused variable c in :torque-vector
* important change;; use association list instead of hash table because hash table directly uses sys::address and assoc list is faster than hash table in the case of small table
* add debug message for user set weight
* use prog1 in :inverse-kinematics-loop and :inverse-kinematics
* add calc-torque-buffer-args for torque methods
* use calc-torque not of super class but of self class for basicmodel robots
* use :calc-torque method in :calc-zmp
* use :update-mass-properties in :weight and :centroid methods
* separate update-mass-properties from :calc-inertia-matrix-from-link-list
* remove unnecessary allow-other-keys
* use all-child-link instead of :exec-func-for-all-links and remove :exec-func-for-all-links
* add comment for :cog-jacobian-balance-nspace
* add grasp matrix and update calculation of default force & moment in :torque-vector
* just fix indent in :torque-vector
* remove unnecessary ext-force moment setting for non-legged robot
* import make-ring function to user package
* add make-ring function to generate ring primitive
* rename handles -> handle according to rbrain robot-object
* update g-vec value ;; 9.8 -> 9.80665
* enable to set sweep parameter
* enable to set crank model parameter
* add deftest to obtain unittest result
* rename eusmodel-validity-check -> eusmodel-validity-check-one ;; eusmodel-validity-check becomes single unittest program
* remove labels function and fix indent
* add handle function according to committed irteus models
* add sample-broom class ;; separate labels function make-sweep
* add eusmodel-validity-check ;; this code is not pr2 dependent and only dependent irtmodel specification, so i copy this function from euscollada-pr2-test.l
* add :calc-force-from-joint-torque
* add set-stereo-gl-attribute for quad buffer stereo
* fix eus->collada conversion
* use :joint-angle method in crank-motion
* add comments to model making functions
* just fix indent
* define sample-crank class instead of local functoin
* remove labels and fix indent
* separate limb generating function to class method
* add ;; to warning message in :joint-angle method
* fix typo
* use :arrow-scale argument in :draw-circle to configure scaling of tip of arrow
* add :dump-command nil
* consider all links included in robot model using all-child-links
* add self collision check for all links using pqp-collision-check
* define max-joint-torque for sample robot and draw torques in crank-motion sample
* add link/joint method, link/joint name might be string or symbole, thus do use (send robot :link name) instaed of (send robot name)
* fix for month
* add c-isnan for c implimentation of isnan
* add irtpoint.l irtgeoc.c for adding pointcloud class
* fix dump-command
* add funcall because target-coords support lambda definition;; TODO -> Should lambda function be dumped??
* dump ik log both sucess/fail
* add calc-weight from joint max velocity
* add :calc-joint-max-velocity, and update truncate to speed limit algorithm in :move-joints, set periodic-time is 0.5 (10 times faster than default speed) when :inverse-kinemtaics
* print scaled dav as debug message
* add angle-to-speed, calc-joint-angle-min-max-for-limit-calculation uses angle-to-speed
* joint-angle violation warning only when non-relative mode
* j . axis is not always vector for rotational-joint
* warning-message with yellow color and use > instead of >= for min/max check
* use copy-list instead of copy-object for args->print-args
* dump ik-command under /tmp if :inverse-kinematics filed
* clean up :command debug code in :inverse-kinematics
* do not destroy arguments copy args to print-args
* minor fix : print s-expresion that re-produce failed ik commmand #666
* print s-expresion that re-produce failed ik commmand #666
* back to r777, no need to set target-coord
* fxo typo ans->and
* do not overwrite target-coords arguments inside the methods
* do not overwrite target-coords arguments inside the methods
* fix bvh2eus to use load-mcd
* remove mis commit (r->self) for add :joint-order for irtbvh
* add :joint-order for irtbvh
* all-descendants is now changed to all-child-links, as irtrobot is cascaded-links that contains only bodyset-link (or cascaded-coords i.e end-coords)
* add default :gripper method to irtrobot
* add all-descendants
* update eusbvh.l
  - send bvh-model :angle-vector (rbrain like reset-pose)
  - send bvh-joint :bvh-offset-rotate (bvh parameters)
  - send bvh-joint :bvh-offset-rotate-offset (bvh parameters in rbrain-linke reset-pose coordinates)
* :joint-euler-angle : add :child-rot keyword
* fix violate-min/max-angle warning
* check if m is 3x3 matrix for matrix-to-euler-angle
* print warning if joint-angle exceeds min/max angle
* fix : back to r628
* do not use joint-angle-bvh (bvh-offset-rotate) in :dump-motion methods
* update sensor-model to allow-other-keys and camera-model to pass args to instantiate sensor-model class
* support :name key to make-camera-from-param
* fix manipulability-limit and gain according to use-leg mode
* remove unnecessary update-mass-properties in cog-nullspace calculation
* use :weight method ;; support euscollada robots in calculation of cog-jacobian
* do not update mass property in :calc-inertia-matrix-from-link-list
* add comment for axis-for-angular
* remove unnecessary debug-view argument
* remove default parameter for target-centroid-pos ;; use :centroid method to calculate whole-body centroid
* add irtgraph.l to irteus, this is originally euslib/jsk/graph.l
* remove trace of :parent in :find-link-route <- this is obsolete
* check weight == 0 in calculation of centroid
* trace all child-links from root link to calculate mass property information
* update find-link-route to support robots which has links not included in (send robot :links)
* use  error instead of wrning-message for negative max-joint-torque and negative max-joint-velocity
* add find-parent for when parent-link is not included in the link-list :calc-jacobian-from-link-list
* fix negative max-joint-velocity/torque for non scalar joint
* fix clac-angle-speed-gain-scalar/vector #704
* print warning when max-joint-velocity and max-torque-value is minus
* fix bug of fix leg to coords in full-body-ik sample
* enable to set faces as an argument for pqp collision model
* add debug message on :inverse-kinematics
* set 6dof-joint's weight by default in :fullbody-inverse-kinematics ;; note that i defined additional weight set by using (memq :weight args) instead of weight argument because (memq :weight args) does not require default weight value
* add method :inverse-rotate-vector to coordinates
* enable to set collision-avoidance-link-pair outside of limb's inverse-kinematics methods
* fix typo ;; cascaded-link -> coordinates
* remove deprecated argument dt
* update dual-manip-ik according to r725 commit
* add arguments for jacobi and take function as union-link-list ;; update calculation of jacobian for object manip ik
* add comments for :fullbody-inverse-kinematics and :cog-jacobian functions
* fix *viewer* existence check ;; *viewer* is always bound in eus/lisp/geo/viewport.l ;; if viewer is available, *viewer* does not nil
* add :fullbody-inverse-kinematics using root-link virtual joint and cog-jacobian balancing
* fix typoes ;; cog-thre -> centroid-thre
* add union-link-list for funcall argument in calc weight by lambda form
* add cog-convergence debug message if ik fail
* move centroid convergence check codes to check centroid convergence at the ending of IK
* add :ik-convergence-check and remove duplicated codes
* extract loop count check from convergence check dotimes loop
* add centroid-offset-func ;; this can be used for balancing against simulation external force
* just fix arrangement of local variables and indents
* fix typo ;; cog-target-pos -> target-centroid-pos ;; add additional check for target-centroid-pos
* add comments to use cog-jacobian in :calc-inverse-kinematics-nspace-from-link-list
* add cog-jacobian null-space to :calc-inverse-kinematics-nspace-from-link-list ;; cog-jacobian is not used by default for non-legged robots
* update crank-motion sample to use legs' constraint in fullbody :inverse-kinematics
* just fix indent according to r718 commit
* use *g-vec* instead of using 9.8
* fix bug when rotation angle between v and axis equals to 180[deg] ;; support :-x, :-y, and :-z for axis
* fix automatic change of perspective-far and perspective-near
* :spots returns all spots in sub-scenes ;; patching saito's diff
* assoc both spots and other objects to scene model
* assoc spot to scene model
* support function target-coords in ik fail message
* add orient-coords-to-axis function and use it in :look-at ik
* fix only indent according to r710 commit
* support lambda target-coords to update target-coords in every ik loop ;; for example, target-coords must be updated in look-at ik
* remove unused variable dt from :calc-torque and :torque-vector
* fix calculation of default moment ;; minimal interanl moments
* fix typo ;; rleg-end-coords -> lleg-end-coords
* fix bug of axes of ankle joints ;; ankle-p -> :y, ankle-r -> :x
* fix direction of z axis for rarm-wrist-y
* fix typo ;; rleg-kneep-p -> rleg-knee-p
* reset ext-force and ext-moment because these parameters are always set before :calc-torque
* enable to configure arms and legs sizes
* add labels functions to reduce duplicate codes to generate limbs
* remove unused variable c in sample-robot-init
* rename rleg link names ;; rarm-link? -> rleg-link?
* set centroid of links other than default-robot-link
* set a mass center of default-robot-link as a volume center
* set objs to *irtviewer-objects*
* fix typos ;; irtiewer-objects -> irtviewer-objects
* do not occur error when mkae-irtviewer/objects was called with out X server
* remove implicit mkdir;; use must specify output directory explicitly
* add make-package to irtext.l to import collada functions
* add exit-p to indicate whether exit or not
* update default argument for output-full-dir
* add collada converter codes to irteus
* revert dummy commit
* dummy commit for jenkins
* dummy commit for jenkins
* increase max-joint-velocity of rotational-joint because (/ pi 4) is too small
* add use-line-break argument to neglect linebreak
* fix typo warnig-color -> warning-color
* add draw-torque method to robot-model class
* add comments and add buffer reuse codes
* update :propagate-mass-properties method ;; separate append-mass-properties calculation from this method
* set wrt to :local if wrt is not specified by user
* fix auto far/near adjustment
* print warning message if joint name not found ;; this is behaviour before revision 644
* fix typoes in angle-speed-collision-blending setting
* fix angle-speed-blending ;; previous -> null-space vector for collision-avoidance is affected by blending coefficient ;; disable angle-speed blending if avoid-collision-joint-gain equals zero
* fix :look-all paramater
* add code to change perspective-near and paerspective-far according to object size in :look-all
* add method to calculate torque-vector using robot's knowledge
* add method to calculate fullbody mass properties
* fix for missing parent-coords
* add max-torque-vector method to cascaded-link
* add max-joint-torque to slots of joint class
* add make-camera-from-param and send :draw-on to camera-model to draw viewings
* fix header size for using defined constant
* fix defforeign shared libraries because file name is not needed for linked library in Linux
* svn propdel svn:executable CPQP.C
* propset svn:executable on
* add shebang in demo.l
* remove parsing of joint instamce using member because naming rules are standardized to use :[limb name]-[joint name]
* add user-defined joint methods to sample-hand and fix typoes
* add user-defined joint methods to sample-robot
* if robot have limb-method named method, call them in :limb
* update draw-things coordinates before :look-all
* add write-to-image-file to glsurface
* fix to use float number
* update :angle-vector for infinite rotational-joint, crop between -180 and 180
* revert r634, #493
* add revert-if-fail option
* add :move-arm limb when call :inverse-kinematics from send *robot* :rarm/:larm
* remove previous commit, make OS=Linux32 to create 32bit eus on x86_32 machine, make create 64bit eus
* modify Makefile in order to make 32bit jskeus when ARCHDIR=Linux on x86_64 machine. if ARCHDIR=Linux64 on x86_64, it creates 64bit jskeus
* standardize unit system of moment ;; fix yaw moment, [Nmm]->[Nm]
* add irtbvh.l in compile_irtg, irtext.l and Makefile
* add joint-eular-angle to sphere-joint
* add irtbvh.l
* add matrix-to-euler-angle
* add :calc-zmp method ;; add joint velocities and joint accelerations arguments to :calc-torque and remove av argument
* add CYGWIN_NT-6.1-WOW64 cygwin on Windows 7 x64
* modify Makefile.Cygwin and euspng.c for most recent Cygwin-6.1 1.7.8, this is checked on Linux version
* set tmp-dim from tmp-dims and remove unused allocation for tmp-dim ;; use j instead of i for index of dotimes of vel-p and vel-r because i is already used in outer dotimes
* warn failed information when :look-at did not converge
* pass :target-coords to :inverse-kinematics-loop in :look-at for debug-view
* add :joint-list and :links method interface to robot-model's :limb method
* fix typo in with-assoc-move-target macro
* fix local variable name to follow implicit euslisp naming rules
* rename local variables in with-assoc-move-target to avoid symbol conflict
* add with-assoc-move-target macro for associating and di-associating move-target
* add allow-other-keys to :joint-angle method like other joint classes
* enable usage of list or function to set inverese-kinematics weight
* add :spot, :object
* add max-joint-velocity to each joint classes
* tmp-weight and tmp-nspace ;; local variables -> argument for fast calculation and buffering
* get manuals from /svnroot
* fix typo ;; mqme -> memq ;; increase max loop count because in some case max = 30 is too few
* add warninge message when eus-server fails
* fix :angle-vector method to support joints which joint-angle is float-vector such as sphere-joint, omniwheel-joint, wheel-joint and 6dof-joint
* fix sample-robot's head link configuration ;; number of head links was 3, which is not corresponding to number of head joints 2
* remove circular warning to compile irtc.o
* execute (make-irtviwer) in (objects) if no viewer is exectued before
* check if objs has :bodes method in :bodies method of irtscene
* add eus/models for eus-installed
* add provide, in-package, and
* add irtscene for scene-model class
* add make-faceset-from-vertices, which is used in converter irteus model and objecets
* add to read image from eusdir relative directories
* check (get texture-img (:texture-max-width,:texture-max-height)) for max texture image size
* add x::window-main-one to update-particle to accpet mouce event
* irtc depend on everything for SVNVERSION
* remove (print args) debug code from bumper-model
* remove debug code
* add draw-objects for camera-model
* add :3d-point method for camera-model
* fix face-to-tessel-triangle to manually copy edge instance, do not use copy-object
* add :viewing method to camera-model
* copy edge before make new face
* add comment
* fix bumper-model wich new sensor-model class
* fix end-coords position
* send *irtviewer* :title to each demo
* change ik :step 300 -> :stop 500 for safety reason
* fix l6 and b6
* support middle button to move view-target and use *persepctive-far* to limit look-all
* add MACHINE=i386 for 10.5 OSX
* move :look-at method in :inverse-kinematics* from cascaded-link to robot-model because :head depends on not cascaded-link but robot-model
* update for revision 577 commit ;; fix weight of translation component of 6dof-joint
* mm->m does not needed in calc-jacobian-linear
* add extern to CPQP.c for explicit declaration
* rewrite sample-arm-model.l, do not use translate/rotate-vertices
* fix :find-link-route method for links which parent-link is not prepared adequately ;; fix :look-at method
* matrix-log retuns [-pi, pi]
* add find-parent labels function for move-target which parent is not included in link-list
* fix bug #356, when q0 is 0, by r.ueda
* fix for old Makefile, patch by r.ueda
* source bashrc.eus -> source ~/bashrc.eus
* add Makefile.Darwin
* darwin, use macport libraries from /opt/local/
* add function definition for PQP_MakeModel
* define gluTessBegin/EndPolygon in eus/opengl/src/oglfunction.c.c
* r563 is wrong, this code was ok
* remove redundant gluTestEndContour and gluTessBeginCountour
* add Makefile.Darwin
* set path for Darwin
* fix make-sphere
* extern make-sphere x-of-cube y-of-cube z-of-cube  height-of-cylinder radius-of-cylinder radius-of-sphere
* add make-sphere, accessors
* add body-to-faces
* move body-to-triangles from jskeus to irtgeo
* remove :area check before make-face-from-vertices
* return if (send face :area) is too small
* do not use *vertex-neighborhood-threshold*, it did not work well
* fix: infinity loop when geo::face-ear-clipping could not find mutually visible vertices
* fix bug of weight in :calc-joint-angle-speed ;; in null space, weight is not always necessary ;; necessary to collision avoidance and limit avoidance
* support Darwin, set ARCHDIR=Darwin if uname -m is Darwin
* divide demo.l to several files, add hanoi-arm.l
* add particle simulation and timer-obj samples
* make link from /irteus to jskeus/irteus, so that users is able to load irteus/irtmodel.l
* add new line to warn in geo::face-to-triangle-make-simple
* rename (camera . viewing) -> (camera vwing)
* rename to bumper->bumer-model, sensor->sensor-model
* rename to bumper->bumer-model, sensor->sensor-model
* fix for CYGWIN
* fix to write bashrc.eus for Cygwin
* comment, add jskeus directory to load-path
* load file relative to jskeus directory for new *load-path* rule
* add jskeus/ directory, which is up directory of eus/ or irteus/ directory to *load-path*
* change messege to instruct how to setup bashrc
* svn up again if failed
* comment out (load "irdmodel.l")
* move demo.l to demo directory
* checkout eus directory with -N
* add SVNVERSION for cygwin
* fix for CYGWIN_NT-6.1
* fix for CYGWINNT-6.1
* show bashrc.eus sestting in the last of make process, and fix typo LD_LIBRARY_PATHPATH->LI_LIBRARY_PATH
* change viewing in camera, viewing is squal to coordinates if left camera, and translate if right, camera, so that :ray use original (float-vector (* screenx u) (* screeny v) viewsitance), :screen-point uses viewing (which is align to left camera frame) to calculate
* :screen-point does not use :view since left-handed viewing coordinates is not fit modern camera information, so we use send self inverse-transformation and :project3
* set :view-up default to #f(0 -1 0)
* fix typo on camera (:width () pheight) -> (:height () pheight)
* remove with-append-obj-virtual-joint macro;; alternatively add append-obj-virtual-joint function ;; user can append virtual joint using this function and add sample to test5
* fix :ray divide ix iy with pwidth/2  and pheight/2
* fix :ray divide ix iy with float-converter pwidth and pheight
* fix typo in :ray pwdith -> pwidth
* viewing :ray takes normalized (NDC) inputs and retuns ray in global coordinate frames,
* add :draw-circle for draw circle around a coordinates
* fix bugs on ray, (send viewing :ray) retuns  ray in viewer coordinate frames, (send cam :ray) retuns ray in world coordinate frames
* rename sensor to sensor-model
* rename pwidth/pheight -> width/height as rbrain/basicsensors.l
* add :pwidth and :pheight accessor in sensors, and support :name keyword
* add irtsensor.l for robot modeling
* add additional-check to :inverse-kinematics-loop ;; user can define optional convergence conditions by using :additional-check
* use with-gensyms in with-** macro
* add with-append-virtual-obj-joint and calc-jacobian-from-link-list-including-robot-and-obj-virtual-joint for multi-armed manipulation of an object ;; add dual-armed manipulation example to demo.l as test5
* add with-gensyms macro for multiple gensym
* enable offsetting jacobian column index
* add transform-coords to :calc-jacobian-from-link-list ;; jacobian is represented in transform-coords ;; transform-coords is move-target by default
* remove unnecessary argument pos-r and pos-i
* use worldrot and transform instead of :inverse-transform-vector in calc-jacobian-default-rotate-vector ;; this may cause fast calculation ;; remove unused buffer tmp-v3a and tmp-v3b from calc-jacobian-default-rotate-vector
* change how to set default joint name because the name generated by gensym cannot be dumped
* :paste-texture-to-face searches image file in *eusdir*/img
* add union-link-list and jacobian to arguments ;; user can set these parameters outside of :inverse-kinematics and :inverse-kinematics-loop
* remove unused variable jacobi in :collision-avoidance and fix typo avoid-collision-joint-gain -> avoid-collision-null-gain
* move codes for calculating nspace  from :move-joints-avoidance to :calc-inverse-kinematics-nspace-from-link-list ;; this method can be called outside of :move-joints-avoidance
* move codes for calculating weight from :move-joints-avoidance to :calc-inverse-kinematics-weight-from-link-list ;; this method can be called outside of :move-joints-avoidance
* fix weight multiplication in :collision-avoidance ;; separate null component from joint component
* add PQPDELETEMODEL to delete PQP model pointer
* align to 2^n when scale big image image
* set viewpoint larget thatn *perspectiv-neer*, this works if target is small
* update :move-viewing-around-viewtarget
* return to initial joint-angle ;; not only joints included in union-link-list but also joint-list ;; for example, if using :look-at-target t, head joints must return to initial joint-angle
* :collision-avoidance-link-pair-from-link-list supports link-list which length is 1
* return string when strm is nil likewise format
* fix: :look-all can take bbox, bodies,links,robtos... as an arguments
* fix :look-all, when the target is not #f(0 0 0)
* call :draw-objects end of :create
* send-all objects :worldcords before draw-things
* glNormali3fv needs :inverse-rotate-vector
* key of hash table gl-texturecoords is changed from vertices to edge, since vartices is updated if you call :translate etc.
* initialize m-til, c-til and I-til in :reset-dynamics, so all bodyset-link's parameters become initialized when :init
* fix calculation of moment ;; user must set ext-moment based on ext-force
* fix typo ;; angular-velocity -> angular-momentum
* calculate world axis from worldcoords of default-coords
* add accessor to :angular-acceleration and :spacial-acceleration
* update target in every iteration in :look-at
* support :move-target in :look-at
* move joint-torque, joint-velocity and joint-acceleration from bodyset-link's slots to joint's slots
* remove calculation of momentum-velocity and angular-momentum-velocity ;; these parameters are not essential to :inverse-dynamics ;; remove unused tmp-vd and tmp-ve
* add buffers to :inverse-dynamics methods for fast computation
* fix bug of multiple declaration of I in :inverse-dynamics and propagate debug-view argument to :forward-all-kinematics and :inverse-dynamics
* support rotational-joint and linear-joint and arrange codes by using let
* modify unit system for inverse dynamics variables for fast computation and support joint classes other than rotational-joint at following revision
* move inverse-dynamics codes from irtmodel.l to irtdyna.l
* move gluScaleImage from :paste-texture-image to draw-glbody
* add SVNVERSION to lisp-imprementation-veresion
* fix typo #:+jsk -> #+:jsk
* rename test4->test-j1 and add #+:jsk for non-jsk users
* check pathname-type before read-pnm file
* use glBindTexture to execute glTexImage2D once per image
* fix bug of calc-inertia-matrix-linear ;; axis-for-angluar [mm] -> [m]
* add update-mass-propeties to reduce computation of mass propeties
* update inertia matrix calculation ;; use world-default-coords instead of child-link ;; refer to revision 452 commit
* fix bug in :joint-angle of 6dof-joint and sphere-joint ;; use anguler velocity for orientation instead of RPY angle ;; modify axis (list :z :y :x) -> (list :x :y :z)
* remove :assoc and :dissoc because this assoc is essentially unnecessary ;; only after revision 452, this unused assoc has a bad effect because parent-link's coords is refered in calculation of world-default-coords
* add manuals target
* add to download jmanual.pdf and manual.pdf
* reduce duplicate codes for joint-angle-limit-weight and joint-angle-limit-nspace
* remove unsupported directories in eus/lib
* svn co euslisp if eus directory is not exists
* use svn:externals to chekout euslisp
* remove *-update, *-installed to svn up and make everytime you type make
* remove comment-outed code for jacobian calculation
* update jacobian calculation ;; use world-default-coords instead of child-link because axis is fixed to world-default-coords
* revise diffs commited at revision 439 and revision 443 about wheel-joint and omniwheel-joint
* back to 424 ;; need to rearrange codes about calc-jacobian-child-rotate-vector ;; diffs commited at revision 439 and revision 443 will be revised
* axis of omniwheel-joint, sphere-joint and 6dof-joint are defined default-coords relative axis
* convert to 8bit if bit_depth is 16bit in PNG_READ_IMAGE
* :pasate-texture-to-face support :image and :tex-coords as jskeusgl
* do not convert rgb->bgr in PNG_COLOR_TYPE_RGB
* propset snv:keywords Author Data Id Revision
* do not :add-port (display-id) when *display* is nil, this happens when DISPLAY is not defined and init-xwindow is not called from eusrt.l
* update :joint-angle method of omniwheel-joint ;; support both relative = t and relative = nil
* (get self :gl-textureimage) holds texture images pasted on it's faces, for read-image-file once per same file
* set max size of texture image to 256*256
* set name to gluScale-d-Image
* fix dimension of joint-angle of wheel-joint ;; wheel-joint is 2-dof joint
* remove vpush, since function cons(a,b) push a and b internally
* change makestring -> makebuffer, since make-string assume buffer is already allocated by someone else
* whien image type is PNG_COLOR_TYPE_RGB_ALPHA, convert RGBA->rgb
* update to support png platte image
* support grayscale-image texture mapping
* fix samples ;; define :reset-pose method for simple-robot as a initial posture ;; update for no-leged-robot
* fix bug of 6dof-joint's axis calculation ;; paxis shuold not be child-link relative ;; wheel-joint, omniwheel-joint and sphere-joint are not fixed yet.
* remove GL-DISPLAYLIST-ID when new texture is settled to a face
* make the origin of the sample-robot to the origin of the root link ;; add transform in order to fix simple-robot's legs to some coords
* compile lisp/image/jpeg before eusjpeg.l
* remove bashrc.eus when make clean
* remove pgsql.c,h which is generated when compile pgsql in lib/llib directory
* use image/jpeg/makefile to clean the directory
* add irtdyna.l for the mass properties and dynamics computation ;; currently irtdyna.l supports cog jacobian and inertia matrix ;; add test5 as an example of fullbody motion generation
* add with-append-root-joint macro in order to add joint such as 6dof-joint and omniwheel-joint
* .bashrc use # for comment, changed from lisp style comment ;;
* fix compile error, pqp->euspqp
* fix read-image-file
* fix read-png-file, add file name to :name of image object as jpeg, remove old api
* add irtimage.l file for read-image-file, write-image-file
* add read-png-file and write-png-file
* fix for cygwin, load cygGL-1.dll/libGL.so to defforeign glPolygonOffset
* add to compile libirtimg.so for eusjpeg
* add :select-drawmode to support hid mode in irtviewer
* generate bashrc.eus file
* add makefile to compile euslisp and irteus
* move converter codes to rbrain directory because the converter program depends on rbrain codes
* fix computation of child-reverse ;; if len = l+1, child-reverse is not detected in previous code
* this is a commit for fixes #178 ;; use child-reverse for axis of joint
* this is a commit for refs #178
* :lookup support :debug-view and display current count, max count was changed from 10->30
* use weight in :collision-avoidance
* fix with-different-positions-and-rotations macro by nozawa
* fix a-link-list -> col-link-list because al in a-link-list may go out of range of ddav-col
* rename the latter ddav-col -> dav-col-null in order to make codes easy-to-read ;; add some comments to :collision-avoidance method
* fix evaluation in with-difference-positions-and-rotations
* add joint-args instead of if sentence about 6dof-joint
* remove unused jacobi# from :collision-avoidance and add jacobi# check in :calc-joint-angle-speed
* fix nan check ;; when using abs, (eq r *nan*) cannot be used for nan check
* reduce diffuse calculation of I-J#J ;; previously I-J#J is calculated in :calc-joint-angle-speed and :collision-avoidance ;; at new revision I-J#J is calculated only in :move-joints-avoidance and passted to :collision-avoidance and :calc-joint-angle-speed as a :key argument
* change specification of :calc-collision-avoidance-distance to reduce codes ;; previously difference between the nearest-points of link pairs is used as ret ;; at the new version that is used as col-list
* rearrange codes in :collision-avoidance and remove unused variables
* add comments to joint-angle-limit-nspace and joint-angle-limit-weight and rearange several lines using let
* add utility macro with-move-target-link-list
* add &allow-other-keys to :joint-angle ;; see basicmodel.l and irtmodel.l at revision 40469
* remove debug code from rotational-joint class
* add &rest args in arguments of :speed-angle method of joint
* add :joint-dof, :speed-to-angle, :calc-jacobian in joint class
* move fstringdouble for working with eus2, and change name of fstring2double, double2fstring
* fix weight calculation in :move-joints-avoidance ;; limitation of weight to ristrict weight value over 0.001 seems not to be necessary ;; add sample function to ik-test.l
* add codes for Linux64 bit. In order to use Linux64, envirornment variable have to be set ARCHDIR=Linux64
* use gensym name for joint class for uniq naming
* support function for null-space ;; eval(when load) does not work when using local variables defined outside of :inverse-kinematics
* fix setf of dav-col ;; 6dof-joint, wheel-joint, omniwheel-joint and sphere-joint ware not supported at previous version ;; use calc-target-joint-dimension to calculate joint index
* remove unused let from 6dof-joint and sphere-joint ;; add args to 6dof-joint ;; at pvrevious version, 6dof-joint does not work well because of hrp2legs-joint's commit at revision 38978
* fix convergence check of :inverse-kinematics-loop ;; likewise convergence check of :inverse-kinematics, use dif-pos and dif-rot instead of vel-p and vel-r because dif-pos and dif-rot directly represent whether ik converged or not
* bring back p-limi, r-limit argument for calc-vel-from-*
* reduce duplicate codes for format array or vector ;; add format-array function based on print-array
* add argument to :joint-angle if joint is derived from 6dof-joint when ik-fail
* add weight modification to :move-joints-avoidance ;; reduce weight of the duplicate joint such as :torso in dualarm-ik
* bug fix for coordinates::difference-rotation :rotation-axis (:xx :yy :zz)
* revert joint-angle-limit-nspace at revision 37613 ;; please check /home/nozawa/prog/euslib/demo/ik/ik-test.l
* rename geo::face-ear-clipping to geo::face-to-triangle-make-simple, add :insode check with edge entersection, this produces better result, but not perfect.
* support jacobian-based :inverse-kinematics in hrp2legs-joint ;; if :use-toes '(t t), use jacobian-based :inverse-kinematics for :rleg and :lleg ;; if :use-toes '(nil nil), use :kajita-ik for :rleg and :lleg ;; fix :link-list of hrp2-leg's :inverse-kinematics (only use leg's link-list)
* rearange codes using let and remove unused variables
* adapt irteus codes to update in which collisoin-avoidance-link-pair is replaced to collision-avoidance-links at revision 32377 ;; add *-collision-avoidance-links to robot-model's slot similar to *-end-coords or *-root-link and some methods using *-collision-avoidance-links ;; update convert-irtmodel to use *collision-avoidance-links
* add weight, centroid and inertia-tensor to converted irt eus model
* add find-extreams finction: find the elements of datum which maximizes key function
* fix for rotation-axis :xm :ym :zm, change defference-rotation method in coordinates and calc-dif-with-axis function
* i convert the most of all source code by nkf -w. if you want to know about details, you need to check the emails from ueda, subject are the source codes of eus to utf8. i have compared python, gauche, nkf, jchardetect, qkc and nkf is the best way. nkf can detect euc-jp, sjis and iso-2022 and convert to utf8, but cannot latin-1. so i didnt convert i2c and debian project
* add libnr
* change compile order
* split nr code from irtc.c to nr.c
* special value for :waist-p
* add :return-with-port keyword to connect-server-until-success
* add :max-port keyword to connect-server-until-success
* move from jsk.l to irtutil.l, eus-server, connect-server-until-sucess
* define need-thread in irtutil.l
* :make-current before gl::draw-globjects in irtviewer
* fix > to >= ;; original paper is if d|dH/dt| >= 0
* float_t->eusfloat_t, integer_t->eusinteger_t
* fix for 64bit eus float->float_t
* :collision-avoidance fix if sentence
* remove unused mode :ik-failed and replace unused return-value ret -> t in :move-joints-avoidance and :move-joints
* add default value for thre, rthre, rotation-axis and translation-axis
* use :calc-union-link-list in :inverse-kinematics
* :calc-vel-from-dif-pos-rot -> calc-vel-from-dif/calc-vel-from-rot
* fix :collision-avoidance-link-pair-from-link-list
* change :  q = f(d) qca + {1-f(d)} J# b + N W y
* fix :collision-avoidance-link-pair-from-link-list
* fix global variables
* draw min-distance collision-pair in red
* add col-dav when twe collision-link-pair has same min-distance
* update debug message
* use [deg] unit for debug message
* add debug message
* add calc-union-link-list
* add debug message
* fix typo, check with cvs -r 1.14 1.14 irtgeo.l
* fix to write mapjoin's package
* fix count of row in :calc-jacobian-from-link-list ;; in previous revision r1.111, link-list which length is over 3 doesn't work
* add utility macro with-difference-position-and-rotation and with-difference-positions-and-rotations
* remove fix-targets support...
* remove look-at-target from inverse-kinematics-loop and add :look-at codes to :inverse-kinematics for compatibility
* IMPORTANT commit. change arguments of inverse-kinematics-loop from coordinates to position and rotation velicity
* make :draw-collision-debug-view method. because debug drawing in :inverse-kinematics-loop is too long and ugly.
* bug fix face-to-triangle 'bad face' error if triangle assigned
* add keyword argument to :calc-jacobian-from-link-list in :inverse-kinematics-loop ;; translation-axis, rotation-axis and move-target
* update face-to-triangle, body-to-triagles
* use copy-object for weight in :move-joints-avoidance
* weight does not have side effect on :inverse-kinematics, but :inverse-kinmatics-loop changes weight
* revert to 1.101
* add face-ear-clpping, face-to-triangle supports face with hole
* add mapjoin
* fix bug in calcation of weight ;; weight was overwrited in :move-joints-avoidance
* remove wmat <- unused in :move-joints-avoidance
* joint-angle-limit-nspace, for wasit-y joint, neutral-angle is always 0
* draw collision-avoidance-result in inverse-kinematics-loop for better display
* fix :no-clear, no-flush rules
* add angle-speed-collision-blending to debug-view message
* add debug in :collision-avoidance
* add pgsql
* set :name using gensym, for (find-viewer) in send irtviewer :create
* make-irtviewer :title works
* update test4
* update test3, this should be mode sophisticated?
* fix test2, do not use map just for dolist, move-target and link-list used in :inverse-kinematics is the purpose of this sample, so these elements must be easily understand
* fix :collision-avoidance-link-pair-from-link-list, if two links have same parent, then never collide???
* add :irt in *features* in irtext.l
* change exit to error for continuing debug of NaN in matrix2quaternion of irtc.c
* previous commit is mistake
* if :debug-view has :no-flush, does not flush in draw-objects of inverse-kinematics-loop
* add :calc-torque to cascaded-link (nozawa)
* add dynamics parameters to bodyset-link class and create new methods for dynamics calculation (:inverse-dynamics, :forward-all-kinematics...)
* add \*g-vec\* for dynamics calculation (ref. euslib/jsk/preview.l and euslib/rbrain/dynamics.l)
* fix bug in :collision-avoidance
* add test4 for using jsk/rbrain library
* fix bug in :collision-avoidance(nozawa)
* fix sample program :: test1 ... add argument use-torso, use-leg :: test2 ... remove rbrain depended code and adapt to dual-arm IK :: test3 ... use multi link-list IK
* add debug-view to :no-clear in :inverse-kinematics-loop and fix default argument of thre and rthre
* need-clear need-flush
* don't load by euscomp
* add
* replace ik method, :move-joints -> :inverse-kinematics-loop
* replace ik method, :move-joints -> :inverse-kinematics-loop
* adapt thre and rthre to list or atom in :inverse-kinematics and :inverse-kinematics-loop
* fix calculation of col in :calc-jacobian-from-link-list
* use args and ik-args for :calc-vel-from-dif-pos-rot
* add angle-speed-limit
* commit for dual-arm ik : main update are as follows : list of move-target,target-coords ... can be used in inverse-kinematics or so. :move-joints-avoidance requires jacobian
* integrate vel-pos and vel-rot -> vel and add :calc-vel-from-dif-pos-rot
* use args and ik-args in :inverse-kinematics-loop
* move-target does not used in move-joint-avoidance
* add default value to stop and loop in :inversekinematics-loop
* add rotation-axis to ik-args and inverse-kinematics-loop and fix "check solved or not"
* move-avoidance-* taes local difference
* fix differnece-position/rotation to calculate local difference
* do not display error when pair-list is null
* fix
* fix previsous commit
* add :inverse-kinmatics-loop
* fix collision-avoidance-links for sample-robot
* add :inverse-kinematics-loop
* add :debug to :collision-avoidance-link-pair-from-link-list
* fix typo :inverse-kenematics-args -> :inverse-kinematics-args
* add :inverse-kinematics-args
* fix: if avoid-collision-distnace 0, do not call :collision-avoidance
* print x when :debug
* if avoid-collision-distnace 0, do not call :collision-avoidance
* fix calculation of dif-pos (in :difference-position)
* update :difference-translation
* fix child-link when child-reverse (in :calc-jacobian-from-link-list)
* fix previous commit / support :translation-axis for dif-pos
* support :translation-axis for dif-pos
* change variable r in joint-angle-limit-weight : global -> local (nozawa)
* support :obstacles
* update ik methods, add references
* fix joint-angle-limit-nspace/joint-angle-limit-weight
* fix:add del-joint
* add del-joint
* update axis of wheel/shere/6dof
* fix :collision-avoidance, use nearest joints for move-joint-avoidance
* mat-tmp-cc, mat-tmp-cr nolonger used in pseudo-inverse
* fix pointer generated by makeint must use intval not >>2
* fix pseudo-inverse2
* modify forward-message-to/to-all 's warning in case to is nil while only in *debug*
* support rotation-axis :xx :yy :zz
* add pseudo-inverse2, use array-dimensions
* set analysis-level in inverse-kinematics
* change polygon -> poly for thread safe
* fix a typo
* modify :move-joints-avoidance
* add ia32 mode for x86_64 linux
* add :draw-objects to geo::viewer for (pickview :separate t)
* chage debug-view in move-joints for (pickview :separate t)
* support :obstacle and :collision-avoidance keyword for :collision-avoidance-link-pair-from-link-list
* remove collision-avoidance-link-pair, add :collision-aovidance-link-pair-from-link-list
* fix typo
* rearrange, la, nul-col, dav-col to null-space-joint-limit, null-space-collision-avoidance, angle-speed-collision-avoidance
* re-arraange collision-avoidance related codes :collision-avoidance, collisoin-avoidance-args :collision-avoidance-calc-distance
* support :null-space '(....)
* fix sphere/6dof-joint to zyx-angle
* change 6d-joint -> 6dof->joint
* fix joint-angle-limit-nspace not to return inf
* support  :no-message in debug-view
* support :no-message mode
* set weight=0 when min-angle == max-angle, with avoid-weight-gain
* add sphere and 6d joint
* support avoid-weight-gain and avoid-null-gain for omniwheel/wheel-joints
* set weight (wmax) even if  avoid-weight-gain == 0
* restore when fillbody ik failed
* support fullbody ik
* change slot variable name from centroid to acentroid
* add centroid
* add :weight and :intertia-tensor
* add permutation and combination
* #f() -> (float-vector) in wheel-joint, because it invekes SEGV
* fixed typos in move-joints-avoidance
* add LIBIRTEUSX to clean
* add wheel/omni-joint
* fix typo
* remove :box from bodyset
* add :box to bodyset
* fix sample-hand
* add default-coords to slot of joint class
* add default-coords to bodyset-link, use default-coords in :joint-angle of linear-joint and rotational-joint
* add argument(args) to :collision-avoidance-link-pair
* fix debug-view angle: display format error
* fix for new cygwin/X
* integrate calc-dif-with-{translation,rotation}-axis to calc-dif-with-axis
* do not calculate weight when avoid-weight-gain and avoid-nspace-gain
* add :angle-speed keyword to :move-joints
* update (:collision-avoidance, (:move-joints-avoidance
* fix wmat, umat, support when min/max-angle is *inf*
* speed up sr-inverse-org
* fix :calc-inverse-jacobian when degenerated link-list,  fix when p-limit/r-limit is nil
* lu-decompose2 accepts LU-DECOMPOSE2 mat [result] [tmp-vector]
* fix typo on copyright
* fix file encoding
* add :analysis-level to bodyset-link
* check event-type and window-id for skipping x event, use :expose for each item
* move defmethod polygon :reset-normal to jskgeo.l
* declare analysis-level in bodyset-link
* fix matrix-log, use atan not atan2
* add sensor information
* fix to work when no viewer created, again
* error handling when normalize-vector #f(0 0 0), again
* fix when no pickview/irtviewer is created
* error handling when normalize-vector #f(0 0 0) -> 0, add VNORMALIZE in irtc.c remove defun normalize-vector from irtmath.l
* fix when avoid-collision-gain 0
* support irteus.exe
* add send-message
* update sr-inverse
* use assoc instead of hash-table
* fix loading sequence
* add irteus
* add trunk, tags and branches
* Contributors: Kei Okada, Kentaro Wada, Kohei Kimura, Noriaki Takasugi, Shunichi Nozawa, Takashi Ogura, Yohei Kakiuchi, Haseru Chen, Jyunya Fujimoto, Hiroyuki Mikita, Masayuki Inaba, Mitsuharu Kojima, Kotaro Nagahama, Manabu Saito, Tsukasa Ito, Ryohai Ueda, Yuto Mori
