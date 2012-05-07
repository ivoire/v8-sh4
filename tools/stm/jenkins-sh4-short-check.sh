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

dir=`dirname $0`

# clean generated files
rm -f *.log *.xml *.tgz

# configuration
arch=${arch:-sh4}
regexp=${regexp:-native}
mode=${mode:-debug}
site=${site:-stgnb}

# build and run in profile mode
env site=$site arch=$arch mode=$mode regexp=$regexp profile_gcov=on $dir/build.sh

rm -f /dev/shm/sem.qemu-user.acicecmg
env site=$site arch=$arch mode=$mode regexp=$regexp profile_gcov=on $dir/run_test.sh --progress=verbose --timeout-scale=15 cctest/*${arch} || true

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
