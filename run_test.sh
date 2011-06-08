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

trap "cleanup" 0 1 2 15

tmpfile=`mktemp /tmp/cctestXXXXXX`

RUN_PREFIX=${RUN_PREFIX:-"env QEMU_ASSUME_KERNEL=2.6.30 /sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 20 /home/compwork/guillon/qemu-stm/build-x86_64/sh4-linux-user/qemu-sh4 -distro -L /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target  -x $PWD -cwd $PWD"}

CCTEST="$RUN_PREFIX ./obj/test/debug/cctest"
$CCTEST --list >$tmpfile 2>&1 || error "cannot list the tests: cctest --list failed"

echo "Listing the available tests..."
if [ $# != 0 ]; then
    echo "Filtering tests with regexp: $1"
    tests=$(cat $tmpfile | grep '<$' | grep "$1" | sed 's/<//g')
else
    tests=$(cat $tmpfile | grep '<$' | sed 's/<//g')
fi

total=0
failed=0

rm -f run_test.log
echo "Running the tests..."
for name in $tests
do
  echo "=== $name ===" | tee -a run_test.log
  res=0
  $CCTEST $name >>run_test.log 2>&1 || res=1
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
