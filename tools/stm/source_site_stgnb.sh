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

#
# Site host tools specifications for STM GNB site
#
export PATH=/home/compwork/projects/scons/`/sw/st/gnu_compil/gnu/scripts/guess-os`/bin:$PATH
export TIMEOUT=${TIMEOUT:-/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout}

# For now proot/qemu on STLinux2.4 does not work correctly 
#export SH4_TOOL_PREFIX=${SH4_TOOL_PREFIX:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/sh4/bin/sh4-linux}
#export SH4_TARGET_ROOT=${SH4_TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/sh4/target}
#export SH4_PROOT=${SH4_PROOT:-/home/compwork/guillon/proot/build-x86_64/devimage/bin/proot}
#export SH4_QEMU=${SH4_QEMU:-"/home/compwork/guillon/qemu-stm/build-next-x86_64/devimage/bin/qemu-sh4"}

# Still use lagacy qemu on STLinux-2.3
export SH4_TOOL_PREFIX=${SH4_TOOL_PREFIX:-/sw/st/gnu_compil/comp/st40/st40-linux/4.6.2-100/bin/sh4-linux}
export SH4_TARGET_ROOT=${SH4_TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
export SH4_QEMU=${SH4_QEMU:-"/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4 -R"}
export QEMU_ASSUME_KERNEL=2.6.30

export ARM_TOOL_PREFIX=${ARM_TOOL_PREFIX:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/armv7/bin/armv7-linux}
export ARM_TARGET_ROOT=${ARM_TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/armv7/target}
export ARM_QEMU=${ARM_QEMU:-"/home/compwork/guillon/qemu-stm/build-next-x86_64/devimage/bin/qemu-arm -cpu cortex-a9"}
export armeabi=${armeabi:-hard}
export ARM_PROOT=${ARM_PROOT:-/home/compwork/guillon/proot/build-x86_64/devimage/bin/proot}
