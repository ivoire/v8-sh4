#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`
arch=${arch:-""} # if not defined, assume build in native mode
site=${site:-default}
mode=${mode:-release}

tmpfile=`mktemp /tmp/cctestXXXXXX`

[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh
library=${library:-shared}
[ "$library" = shared ] && export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
profile_gcov=${profile_gcov:-off}
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

export LANG=
export TZ=:Europe/London
echo Running $RUN_PREFIX ${1+"$@"}
exec $RUN_PREFIX ${1+"$@"}
