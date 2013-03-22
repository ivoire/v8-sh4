#!/usr/bin/env bash
# Copyright 2011 the V8 project authors. All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#     * Neither the name of Google Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`
arch=${arch:-""} # if not defined, assume build in native mode
site=${site:-default}
mode=${mode:-release}
simulator=${simulator:-off}

[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh -a "$simulator" = off ] && . ${pdir}/source_${arch}.sh
jobs=${jobs:-4}
profile_gcov=${profile_gcov:-off}
library=${library:-shared}
[ "$library" = shared ] && export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
# For gcov we need to run in sequencial mode
[ "$profile_gcov" = on ] && jobs=1 && find . -name '*.gcda' -exec rm {} \;

if [ "$QEMU" != "" -a "$simulator" = off ]; then
    if [ "$PROOT" = "" ]; then
	QEMU_OPTS="-distro -L $TARGET_ROOT -x $PWD -x /tmp -cwd $PWD"
	RUN_PREFIX=${RUN_PREFIX:-"$QEMU $QEMU_OPTS"}
    else
	PROOT_OPTS="-b $PWD -Q \"$QEMU\" $TARGET_ROOT"
	RUN_PREFIX=${RUN_PREFIX:-"$PROOT $PROOT_OPTS"}
    fi
else
    RUN_PREFIX="${RUN_PREFIX}"
fi

# Clean profiling files if present
[ "$profile_gcov" = on ] && find . -name '*.gcda' -exec rm {} \;

rm -f run_test_${mode}.log
export LANG=
export TZ=Europe/London
tools/test.py --no-build ${arch:+--arch=${arch}} --mode ${mode} --nocrankshaft --run-prefix "${RUN_PREFIX}" ${V8_EXTRA_ARGS:+--special-command "@ $V8_EXTRA_ARGS"} -j ${jobs} --report --progress mono ${1+"$@"} 2>&1 | tee run_test_${mode}.log
exit ${PIPESTATUS[0]}
