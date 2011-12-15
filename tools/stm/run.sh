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

QEMU_OPTS=${QEMU:+"-distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}
RUN_PREFIX=${RUN_PREFIX:-"$QEMU $QEMU_OPTS"}

echo Running $RUN_PREFIX ${1+"$@"}
exec $RUN_PREFIX ${1+"$@"}