#!/bin/sh
set -e

[ "$DEBUG" = "" ] || set -x

. ../source_sh4.sh
mode=${mode:-release}
SHELL=shell_g
[ "$mode" = release ] && SHELL=shell

rm -f run_benchmarks.log
cp -f ../$SHELL ./

QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
RUN_TIMEOUT=${RUN_TIMEOUT:-1200}
TIMEOUT=${TIMEOUT-"/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout $RUN_TIMEOUT"}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $TIMEOUT $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}


$RUN_PREFIX ./$SHELL run.js > run_benchmarks.log
