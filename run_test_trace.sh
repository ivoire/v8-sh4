#!/bin/sh
set -e

[ "$DEBUG" = "" ] || set -x

error() {
    echo "error: $*" >&2
    exit 1
}

cleanup() {
    true
}

trap "cleanup; exit 1" 1 2 15
trap "cleanup" 0

. ./source_sh4.sh

QEMU=${QEMU:-/home/compwork/guillon/qemu-stm/build-x86_64/sh4-linux-user/qemu-sh4}
TARGET_ROOT=${TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
QEMU_PLUGIN_LIB=${QEMU_PLUGIN_LIB:-/home/compwork/guillon/pycache/contrib/qemu-plugin-profile.so}
START_FUNC=${START_FUNC-_ZN2v88internalL6InvokeEbNS0_6HandleINS0_10JSFunctionEEENS1_INS0_6ObjectEEEiPPPS4_Pb}
RUN_PREFIX=${RUN_PREFIX:-"env QEMU_ASSUME_KERNEL=2.6.30 $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}
RUN_PROFILE_PREFIX=${RUN_PROFILE_PREFIX:-"env QEMU_ASSUME_KERNEL=2.6.30 PLUGIN_SYMTAB_FILE=codegen.sym PLUGIN_TRACE=1 ${START_FUNC:+PLUGIN_TRACE_START=$START_FUNC} $QEMU -distro -tb-plugin-lib $QEMU_PLUGIN_LIB -L $TARGET_ROOT -x $PWD -cwd $PWD"}

CCTEST=${CCTEST:-./obj/test/debug/cctest}
CCTEST_OPTS=${CCTEST_OPTS:-"-debug_code -enable_slow_asserts"}
CCTEST_LOG_OPTS="$CCTEST_OPTS -print_code -print_code_stubs -print_builtin_code -code_comments"

# Clean profiling files if present
find . -name '*.gcda' -exec rm {} \;

rm -f cctest.trace codegen.trace codegen.sym 

$RUN_PREFIX $CCTEST $CCTEST_LOG_OPTS $@ >codegen.trace 2>&1 || true

[ "$NO_PROFILE" = "" ] || exit

tools/comments2symtab.pl <codegen.trace >codegen.sym
$RUN_PROFILE_PREFIX $CCTEST $CCTEST_OPTS $@
