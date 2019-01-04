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
sudo apt-get install -qq -y git make gcc g++ libjpeg-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng-dev xfonts-100dpi xfonts-75dpi  # msttcorefonts could not install on 14.04 travis
# sudo apt-get install -qq -y texlive-latex-base ptex-bin latex2html nkf poppler-utils || echo "ok" # 16.04 does ont have ptex bin
travis_time_end

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

for test_l in irteus/test/*.l; do

    # osrf/ubuntu_arm64:trusty takes >50 min, skip irteus-demo.l
    [[ "$DOCKER_IMAGE" == *"arm64:trusty"* && $test_l =~ irteus-demo.l ]] && continue;

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
