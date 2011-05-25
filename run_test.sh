#!/bin/sh
CCTEST="env QEMU_ASSUME_KERNEL=2.6.30 /sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 20 /home/compwork/guillon/qemu-stm/build-x86_64/sh4-linux-user/qemu-sh4 -distro -L /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target  -x $PWD -cwd $PWD ./obj/test/debug/cctest"

echo "Listing the test..."
tests=$($CCTEST --list 2>&1 | grep '<' | sed 's/<$//')
total=0
failed=0

echo "Running the tests..."
for name in $tests
do
  echo "=== $name ==="
  $CCTEST $name > /dev/null 2>&1
  if [ $? != 0 ]
  then
    echo "...FAILED"
    failed=$((failed+1))
  fi
  total=$((total+1))
done

echo "========================="
echo "Failed test: $failed/$total"
echo "========================="
