#!/bin/sh
set -e
RUN_PREFIX=${RUN_PREFIX:-"env QEMU_ASSUME_KERNEL=2.6.30 /sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 20 /home/compwork/guillon/qemu-stm/build-x86_64/sh4-linux-user/qemu-sh4 -distro -L /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target  -x $PWD -cwd $PWD"}

CCTEST="$RUN_PREFIX ./obj/test/debug/cctest"

echo "Listing the test..."
tests=$($CCTEST --list 2>&1)

if [ $? != 0 ]
then
  echo "ERROR: Unable to list the tests"
  exit 1
fi

tests=$(echo $tests | grep '<' | sed 's/<//g')
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
