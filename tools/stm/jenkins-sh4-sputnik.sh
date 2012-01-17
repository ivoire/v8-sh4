#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x 

dir=`dirname $0`

# clean generated files
rm -f *.log *.xml *.tgz

# configuration
arch=${arch:-sh4}
regexp=${regexp:-interpreted}
mode=${mode:-release}
site=${site:-stgnb}
tests=sputnik

# build and run
env site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/build.sh

# Fetch the test suite
(cd test/sputnik && rm -rf sputnicktests && tar xzf /home/compwork/duraffort/pub/sputniktests-r97.tar.gz)

rm -f /dev/shm/sem.qemu-user.acicecmg
# Use 272k stack size for mjsunit in release mode, seems ok with QEMU/sh4
# Note that with a 256kb only stack string-indexof-1 fails.
env XCCTEST_OPTS="-stack-size 272" suite=sputnik site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/run_test.sh --progress=verbose --timeout-scale=10 ${tests} || true

# Generate JUnit test output
wget -O v8test_to_junit.pl -q ${HUDSON_URL}/job/aci-utils-master/lastStableBuild/artifact/utils/v8test_to_junit.pl
perl v8test_to_junit.pl ${tests}-${mode}-${arch} <run_test_${mode}.log >junit-${tests}-${mode}-${arch}.xml
