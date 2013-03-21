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
srcdir=${srcdir:-.}

# Default architecture: native one
arch=${arch:-""}
site=${site:-default}

# Load the global flags (local and site)
[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
# Load default flags for this arch
[ -f ${pdir}/source_flags_${arch}.sh ] && . ${pdir}/source_flags_${arch}.sh

# Load the CCFLAGS and other arch specific only if not using the simulator
simulator=${simulator:-""}
[ -f ${pdir}/source_${arch}.sh  -a "$simulator" = "" ] && . ${pdir}/source_${arch}.sh

# If acivated the simulator should be set to the arch
[ "${simulator}" = on ] && simulator="simulator=${arch}"

# Define build flags
mode=${mode:-release}
snapshot=${snapshot:-off}
regexp=${regexp:-native}
profilingsupport=${profilingsupport:-off}
debuggersupport=${debuggersupport:-on}
backtracesupport=${backtracesupport:-off}
library=${library:-shared}
armeabi=${armeabi:-soft}
vfp3=${vfp3:-on}
logging=${logging:-off}
prof=${prof:-off}
tests=${tests:-""}


# Enable the GPL disassembler in debug mode if the user does not disable it
# explicitly
[ -z "$gpl_disassembler" -a "$mode" = debug ] && gpl_disassembler=on
gpl_disassembler=${gpl_disassembler:-off}

jobs=${jobs:-4}
profile_gcov=${profile_gcov:-off}
if [ "$profile_gcov" = on -a "$mode" = release ]; then
    echo "error: cannot compile for both mode=release and profile_gcov=on. Use mode=debug for coverage." >&2
    exit 1
fi
[ "$library" = shared ] && export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
[ "$profile_gcov" = on ] && export CXXFLAGS="-fprofile-arcs -ftest-coverage -fno-inline -fno-default-inline -fno-inline-functions -fno-early-inlining" && export LIBS="gcov"

PROOT_ENV=''
[ "$snapshot" = on -a \( "$arch" = sh4 -o "$arch" = arm \) -a "$simulator" = "" ] && PROOT_ENV="$PROOT_FOR_SNAPSHOTS -Q $QEMU_FOR_SNAPSHOTS -b $PWD -b /usr/bin/env $TARGET_ROOT env PATH=/host-rootfs/usr/bin:$PATH "


$PROOT_ENV scons -Y ${srcdir} ${arch:+arch=${arch}} snapshot=${snapshot} mode=${mode} regexp=${regexp} profilingsupport=${profilingsupport} debuggersupport=${debuggersupport} backtracesupport=${backtracesupport} library=${library} armeabi=${armeabi} vfp3=${vfp3} logging=${logging} prof=${prof} gpl_disassembler=${gpl_disassembler} ${simulator} -j ${jobs}
$PROOT_ENV scons -Y ${srcdir} ${arch:+arch=${arch}} snapshot=${snapshot} mode=${mode} regexp=${regexp} profilingsupport=${profilingsupport} debuggersupport=${debuggersupport} backtracesupport=${backtracesupport} library=${library} armeabi=${armeabi} vfp3=${vfp3} logging=${logging} prof=${prof} gpl_disassembler=${gpl_disassembler} ${simulator} -j ${jobs} sample=shell
${srcdir}/tools/test.py -v ${arch:+--arch=${arch}} --build-only --mode=${mode} -S snapshot=${snapshot} -S regexp=${regexp} -S profilingsupport=${profilingsupport} -S debuggersupport=${debuggersupport} -S backtracesupport=${backtracesupport} -S library=${library} -S armeabi=${armeabi} -S vfp3=${vfp3} -S logging=${logging} -S prof=${prof} -S gpl_disassembler=${gpl_disassembler} -S ${simulator} -j ${jobs} ${tests}
