#!/bin/sh
set -e

[ "$DEBUG" = "" ] || set -x

error() {
    echo "error: $*" >&2
    exit 1
}

cleanup() {
    [ "$tmpfile" = "" ] || rm -f $tmpfile
}

trap "cleanup; exit 1" 1 2 15
trap "cleanup" 0

. ./source_sh4.sh

tmpfile=`mktemp /tmp/cctestXXXXXX`
QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
RUN_TIMEOUT=${RUN_TIMEOUT:-1200}
TIMEOUT=${TIMEOUT-"/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout $RUN_TIMEOUT"}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $TIMEOUT $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}

# Clean profiling files if present
find . -name '*.gcda' -exec rm {} \;

CCTEST=${CCTEST:-./obj/test/debug/cctest}
$RUN_PREFIX $CCTEST --list >$tmpfile || error "cannot list the tests: cctest --list failed"
CCTEST_OPTS=${CCTEST_OPTS-"-debug_code"}

echo "Listing the available tests..."
if [ $# != 0 ]; then
    echo "Filtering tests with regexp: $1"
    tests=$(cat $tmpfile | grep '<$' | grep "$1" | sed 's/<//g')
else
    tests=$(cat $tmpfile | grep '<$' | sed 's/<//g')
fi

total=0
failed=0

echo "Cleaning profiling files..."
find . -name '*.gcda' -exec rm {} \;

rm -f run_test.log
echo "Running the tests..."
for name in $tests
do
  echo "=== $name ===" | tee -a run_test.log
  res=0
  $RUN_PREFIX $CCTEST $CCTEST_OPTS $name >>run_test.log 2>&1 || res=1
  if [ $res != 0 ]
  then
    echo "...FAILED" | tee -a run_test.log
    failed=$((failed+1))
  fi
  total=$((total+1))
done

echo "========================="
echo "Failed test: $failed/$total"
echo "========================="
