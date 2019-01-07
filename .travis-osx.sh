#!/bin/bash

set -e

travis_time_start() {
    set +x
    TRAVIS_START_TIME=$(gdate +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | gtr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}

travis_time_end() {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(gdate +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME) / 1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

setup_brew_test() {
    travis_time_start brew.install
    brew install euslisp/jskeus/jskeus --HEAD
    travis_time_end

    travis_time_start brew.test
    brew test jskeus
    travis_time_end
}

setup_make() {
    travis_time_start brew.install-deps
    brew list make &>/dev/null || brew install make
    brew list jpeg &>/dev/null || brew install jpeg
    brew list libpng &>/dev/null || brew install libpng
    brew list mesalib-glw &>/dev/null || brew install mesalib-glw
    brew list wget &>/dev/null || brew install wget
    brew list poppler &>/dev/null || brew install poppler
    travis_time_end

    # travis_time_start install.x11
    # wget http://xquartz.macosforge.org/downloads/SL/XQuartz-2.7.6.dmg
    # hdiutil attach XQuartz-2.7.6.dmg
    # sudo installer -pkg /Volumes/XQuartz-2.7.6/XQuartz.pkg -target /
    # travis_time_end
    export LIBGL_ALLOW_SOFTWARE=1

    travis_time_start script.make
    make -j 2
    travis_time_end

    travis_time_start script.test
    source bashrc.eus
    export DISPLAY=
    export EXIT_STATUS=0; for test_l in irteus/test/*.l; do irteusgl $test_l; export EXIT_STATUS=`expr $? + $EXIT_STATUS`; done;echo "Exit status : $EXIT_STATUS"; [ $EXIT_STATUS == 0 ] || exit 1
    travis_time_end
}
if [ "$TRAVIS_PULL_REQUEST" = "false" -a "$TRAVIS_BRANCH" = "master" ]; then
    setup_brew_test
else
    setup_make
fi

exit 0
