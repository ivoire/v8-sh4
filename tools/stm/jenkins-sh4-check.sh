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
tests=${tests:-cctest}

# build and run
env site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/build.sh

rm -f /dev/shm/sem.qemu-user.acicecmg
env site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/run_test.sh --progress=verbose --timeout-scale=10 ${tests} || true

# Generate JUnit test output
wget -O v8test_to_junit.pl -q ${HUDSON_URL}/job/aci-utils-master/lastStableBuild/artifact/utils/v8test_to_junit.pl
perl v8test_to_junit.pl ${tests}-${mode}-${arch} <run_test_${mode}.log >junit-${tests}-${mode}-${arch}.xml

