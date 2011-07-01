#!/bin/sh
QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
TIMEOUT=${TIMEOUT-"/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 600"}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $TIMEOUT $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}


SHELL="$RUN_PREFIX ./shell_g"

total=0
failed=0

rm -f run_mjsunit.log
echo "Running the tests..."
for file in $(ls test/mjsunit/*.js)
do
  echo "=== $file ===" | tee -a run_mjsunit.log
  res=0
  $SHELL test/mjsunit/mjsunit.js $file >>run_mjsunit.log 2>&1 || res=1
  if [ $res != 0 ]
  then
    echo "...FAILED" | tee -a run_mjsunit.log
    failed=$((failed+1))
  fi
  total=$((total+1))
done

echo "========================="
echo "Failed test: $failed/$total"
echo "========================="
