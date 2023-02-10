#!/usr/bin/bash

# We assume that the directory structure takes the form
# <colcon workspace>/src/TRAILBot
REPO_DIR=$(git rev-parse --show-toplevel)
WORKSPACE_DIR=$REPO_DIR/../..

BUILD_BASE=$WORKSPACE_DIR/build
INSTALL_BASE=$WORKSPACE_DIR/install
LOG_BASE=$WORKSPACE_DIR/log
BASE_FLAGS="--build-base $BUILD_BASE --install-base $INSTALL_BASE"

COLCON_CMD="colcon --log-base $LOG_BASE"

EXTRA_BUILD_FLAGS="--symlink-install"
TEST_FLAGS="--return-code-on-test-failure"

BUILD_TYPE="Release"

usage () {
    echo "Usage: $0 [-h/--help] [-t/--test] [-d/--delete] [--type <type>] <packages>"
    echo
    echo "Build specified colcon packages and optionally run tests."
    echo
    echo "Options:"
    echo " -h, --help          show this help message and exit"
    echo " -t, --test          run tests with 'colcon test' (defaults to false)"
    echo " -d, --delete        with -t, deletes all old test artifacts before rerunning (defaults to false)"
    echo " -v, --verbose       print verbose test results (defaults to false)"
    echo " --type <type>       use a specific CMake build type (defaults to Release)"
    exit 2
}

# Canonicalize arguments so that people can use them in any order.
# https://stackoverflow.com/a/7948533
OPTS=$(getopt -o htdv \
              --long help,test,delete,verbose,type: \
              -n "build.sh" -- "$@")

# If getopt returned a nonzero exit code, the user entered an invalid
# option, so we print usage and exit.
if [[ $? -ne 0 ]]; then
    usage
fi

eval set -- "$OPTS"

unset RUN_TESTS
unset DELETE_TESTS
unset VERBOSE

# Figure out which options have actually been selected
while true; do
    case "$1" in
        -h | --help ) usage ;;
        -t | --test ) RUN_TESTS=1; shift ;;
        -d | --delete ) DELETE_TESTS=1; shift ;;
        -v | --verbose ) VERBOSE="--verbose"; shift ;;
        --type ) BUILD_TYPE="$2"; shift 2 ;;
        -- ) shift; break ;;
        * ) usage ;;
    esac
done

# Remaining arguments are used to specify packages to build
unset PACKAGES_SELECTED
if [[ $# -gt 0 ]]; then
    PACKAGES_SELECTED="--packages-select $@"
fi

TYPE_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE"
BUILD_FLAGS="$EXTRA_BUILD_FLAGS $TYPE_FLAG"

# We always run the build before trying to run tests
$COLCON_CMD build $BASE_FLAGS $BUILD_FLAGS $PACKAGES_SELECTED

# Exit the script if the build failed. Since 'colcon build' doesn't
# return a nonzero exit code on individual package failure, we need to
# go and check the log file manually; we do so by grepping for lines
# that contain the text `'returncode': 2`, which indicates the package
# failed. The [1-9][0-9]* in the regex matches any positive integer;
# in practice, it seems like 2 is the standard error code, but we might
# as well be robust.
LOG_FILE="$LOG_BASE/latest_build/events.log"
PATTERN="'returncode': [1-9][0-9]*"
NUM_FAILURES=$(grep -c "$PATTERN" $LOG_FILE)  # The -c flag counts occurences
if [[ NUM_FAILURES -gt 0 ]]; then
    echo "At least one package failed; exiting build script"
    exit 2
fi

# Run tests as the very last step
if [[ RUN_TESTS -eq 1 ]]; then
    if [[ DELETE_TESTS -eq 1 ]]; then
        $COLCON_CMD test-result --test-result-base $BUILD_BASE --delete-y
    fi
    $COLCON_CMD test $BASE_FLAGS $TEST_FLAGS $PACKAGES_SELECTED
    if [[ $? -ne 0 ]]; then
        # Print test results if any of them failed
        $COLCON_CMD test-result --test-result-base $BUILD_BASE $VERBOSE
        if [[ DELETE_TESTS -ne 1 ]]; then
            echo "[WARNING] Old test results were not deleted before running tests. Some failures" \
                 "may be artifacts from previous runs. Run with -d to ensure all results are from" \
                 "the most recent run."
        fi
        exit 2
    fi
fi
