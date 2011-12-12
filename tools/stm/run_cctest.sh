#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`
arch=${arch:-""} # if not defined, assume build in native mode
site=${site:-default}
mode=${mode:-debug}

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

TIMEOUT=${TIMEOUT:-timeout}
RUN_TIMEOUT=${RUN_TIMEOUT:-1200}
TIMEOUT_OPTS=${TIMEOUT_OPTS:-"-s KILL $RUN_TIMEOUT"}
if [ "$QEMU" != "" ]; then
    if [ "$PROOT" = "" ]; then
	QEMU_OPTS="-distro -L $TARGET_ROOT -x $PWD -cwd $PWD"
	RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS $QEMU $QEMU_OPTS"}
    else
	QEMU_PROOT_OPTS=`echo $QEMU | sed 's/ /,/g'`
	PROOT_OPTS="-W -Q $QEMU_PROOT_OPTS $TARGET_ROOT"
	RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS $PROOT $PROOT_OPTS"}
    fi
else
RUN_PREFIX=${RUN_PREFIX:-"$TIMEOUT $TIMEOUT_OPTS"}
fi

# Clean profiling files if present
find . -name '*.gcda' -exec rm {} \;

CCTEST=${CCTEST:-./obj/test/${mode}/cctest}
$RUN_PREFIX $CCTEST --list >$tmpfile || error "cannot list the tests: cctest --list failed"
CCTEST_OPTS=${CCTEST_OPTS-"-debug_code $XCCTEST_OPTS"}

echo "Listing the available tests..."
if [ $# != 0 ]; then
    echo "Filtering tests with regexp: $1"
    tests=$(cat $tmpfile | grep '<$' | grep "$1" | sed 's/<//g')
else
    tests=$(cat $tmpfile | grep '<$' | sed 's/<//g')
fi

total=0
failed=0

echo "Cleaning profiling files..."
find . -name '*.gcda' -exec rm {} \;

rm -f run_cctest.log
echo "Running the tests..."
for name in $tests
do
  echo "=== $name ===" | tee -a run_cctest.log
  res=0
  $RUN_PREFIX $CCTEST $CCTEST_OPTS $name >>run_cctest.log 2>&1 || res=1
  if [ $res != 0 ]
  then
    echo "...FAILED" | tee -a run_cctest.log
    failed=$((failed+1))
  fi
  total=$((total+1))
done

echo "=========================" | tee -a run_cctest.log
echo "Failed test: $failed/$total" | tee -a run_cctest.log
echo "=========================" | tee -a run_cctest.log
