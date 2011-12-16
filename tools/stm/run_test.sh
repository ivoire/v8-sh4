#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`
arch=${arch:-""} # if not defined, assume build in native mode
site=${site:-default}
mode=${mode:-release}
[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh
jobs=${jobs:-4}
profile_gcov=${profile_gcov:-off}
library=${library:-shared}
[ "$library" = shared ] && export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
# For gcov we need to run in sequencial mode
[ "$profile_gcov" = on ] && jobs=1 && find . -name '*.gcda' -exec rm {} \;

if [ "$QEMU" != "" ]; then
    if [ "$PROOT" = "" ]; then
	QEMU_OPTS="-distro -L $TARGET_ROOT -x $PWD -x /tmp -cwd $PWD"
	RUN_PREFIX=${RUN_PREFIX:-"$QEMU $QEMU_OPTS"}
    else
	QEMU_PROOT_OPTS=`echo $QEMU | sed 's/ /,/g'`
	PROOT_OPTS="-W -Q $QEMU_PROOT_OPTS $TARGET_ROOT"
	RUN_PREFIX=${RUN_PREFIX:-"$PROOT $PROOT_OPTS"}
    fi
else
    RUN_PREFIX="${RUN_PREFIX}"
fi

# Clean profiling files if present
find . -name '*.gcda' -exec rm {} \;

rm -f run_test_${mode}.log
tools/test.py --no-build ${arch:+--arch=${arch}} --mode ${mode} --nocrankshaft --run-prefix "${RUN_PREFIX}" -j ${jobs} --report --progress mono ${1+"$@"} 2>&1 | tee run_test_${mode}.log
exit ${PIPESTATUS[0]}
