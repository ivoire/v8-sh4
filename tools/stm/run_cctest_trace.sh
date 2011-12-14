#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x

test=${1:?}

pdir=`dirname $0`
arch=${arch:-""} # if not defined, assume build in native mode
site=${site:-default}
mode=${mode:-debug}

[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh

TIMEOUT=${TIMEOUT:-timeout}
RUN_TIMEOUT=${RUN_TIMEOUT:-1200}
TIMEOUT_OPTS=${TIMEOUT_OPTS:-"-s KILL $RUN_TIMEOUT"}
if [ "$QEMU" != "" ]; then
    if [ "$PROOT" = "" ]; then
	QEMU_OPTS="-distro -L $TARGET_ROOT -x $PWD -x /tmp -cwd $PWD"
	RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS $QEMU $QEMU_OPTS"}
    else
	QEMU_PROOT_OPTS=`echo $QEMU | sed 's/ /,/g'`
	PROOT_OPTS="-W -Q $QEMU_PROOT_OPTS $TARGET_ROOT"
	RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS $PROOT $PROOT_OPTS"}
    fi
else
RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS"}
fi

echo "Cleaning profiling files..."
find . -name '*.gcda' -exec rm {} \;

CCTEST=${CCTEST:-./obj/test/${mode}/cctest}
CCTEST_OPTS=${CCTEST_OPTS-"-debug_code $XCCTEST_OPTS"}
CCTEST_LOG_OPTS=${CCTEST_LOG_OPTS-"-print_code -print_code_stubs -print_builtin_code -code_comments $XCCTEST_LOG_OPTS"}

rm -f codegen_${mode}.trace run_cctest_trace_${mode}.log
echo "Running the test..."
echo "=== $@ ===" | tee -a run_cctest_trace_${mode}.log
res=0
$RUN_PREFIX $CCTEST $CCTEST_OPTS $CCTEST_LOG_OPTS $@ >codegen_${mode}.trace 2>&1 || res=1
if [ $res != 0 ]; then
    echo "...FAILED" | tee -a run_cctest_trace_${mode}.log
fi
