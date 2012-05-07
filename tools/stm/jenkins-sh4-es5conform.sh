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
mode=${mode:-release}
site=${site:-stgnb}
tests=es5conform

# build and run
env site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/build.sh

# Fetch the test suite
(cd test/es5conform && rm -rf data && tar xzf /home/compwork/duraffort/pub/es5conform.71525.tar.gz)

rm -f /dev/shm/sem.qemu-user.acicecmg
# Use 272k stack size for mjsunit in release mode, seems ok with QEMU/sh4
# Note that with a 256kb only stack string-indexof-1 fails.
env XCCTEST_OPTS="-stack-size 272" site=$site arch=$arch mode=$mode regexp=$regexp tools/stm/run_test.sh --progress=verbose --timeout-scale=15 ${tests} || true

# Generate JUnit test output
wget -O v8test_to_junit.pl -q ${HUDSON_URL}/job/aci-utils-master/lastStableBuild/artifact/utils/v8test_to_junit.pl
perl v8test_to_junit.pl ${tests}-${mode}-${arch} <run_test_${mode}.log >junit-${tests}-${mode}-${arch}.xml
