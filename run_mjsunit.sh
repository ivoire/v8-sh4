#!/bin/sh
set -e

[ "$DEBUG" = "" ] || set -x

. ./source_sh4.sh

QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
TIMEOUT=${TIMEOUT-"/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 600"}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $TIMEOUT $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}

V8_SHELL=${V8_SHELL-"./shell_g"}
V8_SHELL="$RUN_PREFIX $V8_SHELL"

total=0
failed=0

files=${1:-"$(ls test/mjsunit/*.js | grep -v 'debug-')"}

rm -f run_mjsunit.log
echo "Running the tests..."
for file in $files
do
  res=0
  # Get the flags to give to the shell
  flags=$(grep 'Flags:' $file | sed 's/.*Flags: //')
  # Do not run the test that required debugging information (not implemented)
  need_debug=$(echo $flags | grep 'expose-debug-as' | wc -l)
  if [ $need_debug = 0 ]
  then
    echo "=== $file ===" | tee -a run_mjsunit.log
    $V8_SHELL $flags test/mjsunit/mjsunit.js $file >>run_mjsunit.log 2>&1 || res=1
    if [ $res != 0 ]
    then
      echo "...FAILED" | tee -a run_mjsunit.log
      failed=$((failed+1))
    fi
    total=$((total+1))
  fi
done

echo "========================="
echo "Failed test: $failed/$total"
echo "========================="
