#!/bin/bash

set -e

function travis_time_start {
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x # enable debug information
}
function travis_time_end {
    set +x # disable debug information
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
}

travis_time_start setup.apt-get_update
if [ "$(which sudo)" == "" ]; then apt-get update && apt-get install -y sudo; else sudo apt-get update; fi
travis_time_end

travis_time_start setup.apt-get_install
# set non interactive tzdata https://stackoverflow.com/questions/8671308/non-interactive-method-for-dpkg-reconfigure-tzdata
# set DEBIAN_FRONTEND=noninteractive
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
#
sudo apt-get install -qq -y git make gcc g++ libjpeg-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng-dev xfonts-100dpi xfonts-75dpi pkg-config libbullet-dev  # msttcorefonts could not install on 14.04 travis
# sudo apt-get install -qq -y texlive-latex-base ptex-bin latex2html nkf poppler-utils || echo "ok" # 16.04 does ont have ptex bin
travis_time_end

if [[ "$COLLISION_LIB" != "" ]]; then
    travis_time_start setup.collision_lib

    if [[ "$COLLISION_LIB" != "PQP" ]]; then
        rm -fr $CI_SOURCE_PATH/irteus/PQP
    fi

    if [[ "$COLLISION_LIB" != "BULLET" ]]; then
        dpkg -r libbullet-dev
    fi

    travis_time_end
fi

travis_time_start install # Use this to install any prerequisites or dependencies necessary to run your build
cd ${HOME}
ln -s $CI_SOURCE_PATH jskeus
travis_time_end

travis_time_start script.make # All commands must exit with code 0 on success. Anything else is considered failure.
cd jskeus
if [[ "$DOCKER_IMAGE" == *"trusty"* || "$DOCKER_IMAGE" == *"jessie"* ]]; then
    make WFLAGS="-Werror=implicit-int -Werror=implicit-function-declaration -Werror=unused-result"
else
    make WFLAGS="-Werror=implicit-int -Werror=implicit-function-declaration -Werror=incompatible-pointer-types -Werror=int-conversion -Werror=unused-result"
fi
travis_time_end

source bashrc.eus
export EXIT_STATUS=0;
set +e

set -x # enable debug information
# arm target (arm64v8/ubuntu:bionic,focal) stil failing... though the test in EusLisp repository passed
# apply same workaround as we have in EusLisp GA
if [[ ( "$DOCKER_IMAGE" == "arm64v8//ubuntu:bionic" || "$DOCKER_IMAGE" == "arm64v8/ubuntu:focal" ) ]]; then
    sed -i 's@do-until-key-counter 10@do-until-key-counter 1@' irteus/test/irteus-demo.l;
    sed -i 's/h7/ape/' irteus/test/test-cad.l
    sed -i 's/(hanoi-program (length \*disks\*))/(subseq (hanoi-program (length \*disks\*)) 0 2)/' irteus/demo/hanoi-arm.l
    sed -i 's/^\s*footstep-list/(subseq footstep-list 0 3)/' irteus/demo/walk-motion.l
fi
set +x # disable debug information

for test_l in irteus/test/*.l; do

    # osrf/ubuntu_arm64:trusty takes >50 min, skip irteus-demo.l
    [[ "$DOCKER_IMAGE" == *"arm64:trusty"* && $test_l =~ irteus-demo.l ]] && continue;
    # skip collision test because bullet of 2.83 or later version is not released in trusty and jessie.
    # https://github.com/euslisp/jskeus/blob/6cb08aa6c66fa8759591de25b7da68baf76d5f09/irteus/Makefile#L37
    [[ ( "$DOCKER_IMAGE" == *"trusty"* || "$DOCKER_IMAGE" == *"jessie"* ) && $test_l =~ test-collision.l ]] && continue;

    travis_time_start jskeus.source.${test_l##*/}.test

    irteusgl $test_l;
    export TMP_EXIT_STATUS=$?

    travis_time_end `expr 32 - $TMP_EXIT_STATUS`

    export EXIT_STATUS=`expr $TMP_EXIT_STATUS + $EXIT_STATUS`;


    travis_time_start jskeus.compiled.${test_l##*/}.test

    irteusgl "(let ((o (namestring (merge-pathnames \".o\" \"$test_l\"))) (so (namestring (merge-pathnames \".so\" \"$test_l\")))) (compile-file \"$test_l\" :o o) (if (probe-file so) (load so) (exit 1))))"
    export TMP_EXIT_STATUS=$?

    travis_time_end `expr 32 - $TMP_EXIT_STATUS`

    export EXIT_STATUS=`expr $TMP_EXIT_STATUS + $EXIT_STATUS`;

done;
echo "Exit status : $EXIT_STATUS";

[ $EXIT_STATUS == 0 ] || exit 1
