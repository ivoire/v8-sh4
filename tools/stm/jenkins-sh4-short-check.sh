#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x 

dir=`dirname $0`

# clean generated files
rm -f *.log *.xml *.tgz

# configuration
arch=${arch:-sh4}
regexp=${regexp:-interpreted}
mode=${mode:-debug}
site=${site:-stgnb}

# build and run in profile mode
env site=$site arch=$arch mode=$mode regexp=$regexp profile_gcov=on $dir/build.sh

rm -f /dev/shm/sem.qemu-user.acicecmg
env site=$site arch=$arch mode=$mode regexp=$regexp profile_gcov=on $dir/run_test.sh --progress=verbose --timeout-scale=10 cctest/*${arch} || true

# Generate JUnit test output
wget -O v8test_to_junit.pl -q ${HUDSON_URL}/job/aci-utils-master/lastStableBuild/artifact/utils/v8test_to_junit.pl
perl v8test_to_junit.pl cctest-${mode}-${arch} <run_test_${mode}.log >junit-cctest-${mode}-${arch}.xml

# Archive coverage information
find . -wholename "./obj/${mode}/${arch}/*.gcda" | grep -v opcodes | grep -v disasm | xargs tar cvzf profile-${mode}-${arch}.tgz
find . -wholename "./obj/${mode}/${arch}/*.gcda" | grep -v lithium | grep -v opcodes | grep -v disasm | grep -v debug- | grep -v deoptimizer | xargs tar cvzf profile-${mode}-${arch}-min.tgz


# Force cross compiler path for correct gcov version.
# TODO setup in site configuration
PATH=/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/${arch}/${arch}-linux/bin:$PATH
which gcov
mkdir -p artifacts/cov/
rm -f artifacts/cov/coverage*.xml

# Do coverage of minimal set
find . -name '*.gcda' -exec rm {} \;
tar xvzf profile-${mode}-${arch}-min.tgz
python /home/compwork/duraffort/projects/gcovr --xml -r . > artifacts/cov/coverage-${mode}-${arch}-min.xml

# Do coverage of full set
find . -name '*.gcda' -exec rm {} \;
tar xvzf profile-${mode}-${arch}.tgz
python /home/compwork/duraffort/projects/gcovr --xml -r . > artifacts/cov/coverage-${mode}-${arch}.xml
