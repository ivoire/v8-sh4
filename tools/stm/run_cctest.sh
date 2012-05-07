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

error() {
    echo "error: $*" >&2
    exit 1
}

cleanup() {
    [ "$tmpfile" = "" ] || rm -f $tmpfile
}

trap "cleanup; exit 1" 1 2 15
trap "cleanup" 0

tmpfile=`mktemp /tmp/cctestXXXXXX`

[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh
profile_gcov=${profile_gcov:-off}

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

CCTEST=${CCTEST:-./obj/test/${mode}/cctest}
$RUN_PREFIX $CCTEST --list >$tmpfile || error "cannot list the tests: cctest --list failed"
CCTEST_OPTS=${CCTEST_OPTS-"-debug_code $XCCTEST_OPTS"}

[ "$profile_gcov" = on ] && find . -name '*.gcda' -exec rm {} \;

echo "Listing the available tests..."
if [ $# != 0 ]; then
    echo "Filtering tests with regexp: $1"
    tests=$(cat $tmpfile | grep '<$' | grep "$1" | sed 's/<//g')
else
    tests=$(cat $tmpfile | grep '<$' | sed 's/<//g')
fi

total=0
failed=0

[ "$profile_gcov" = on ] && echo "Cleaning profiling files..." && find . -name '*.gcda' -exec rm {} \;

rm -f run_cctest_${mode}.log
echo "Running the tests..."
for name in $tests
do
  echo "=== $name ===" | tee -a run_cctest_${mode}.log
  res=0
  $RUN_PREFIX $CCTEST $CCTEST_OPTS $name >>run_cctest_${mode}.log 2>&1 || res=1
  if [ $res != 0 ]
  then
    echo "...FAILED" | tee -a run_cctest_${mode}.log
    failed=$((failed+1))
  fi
  total=$((total+1))
done

echo "=========================" | tee -a run_cctest_${mode}.log
echo "Failed test: $failed/$total" | tee -a run_cctest_${mode}.log
echo "=========================" | tee -a run_cctest_${mode}.log
