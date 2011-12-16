#
# Site host tools specifications for STM GNB site
#
export PATH=/home/compwork/projects/scons/`/sw/st/gnu_compil/gnu/scripts/guess-os`/bin:$PATH
export TIMEOUT=${TIMEOUT:-/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout}
export SH4_TOOL_PREFIX=${SH4_TOOL_PREFIX:-/sw/st/gnu_compil/comp/st40/st40-linux/4.5.3-90/bin/sh4-linux}
export SH4_TARGET_ROOT=${SH4_TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
export SH4_QEMU=${SH4_QEMU:-"/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4 -R"}
export QEMU_ASSUME_KERNEL=2.6.30

export ARM_TOOL_PREFIX=${ARM_TOOL_PREFIX:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/armv7/bin/armv7-linux}
export ARM_TARGET_ROOT=${ARM_TARGET_ROOT:-/home/compwork/projects/stlinux/opt/STM/STLinux-2.4/devkit/armv7/target}
export ARM_QEMU=${ARM_QEMU:-"/home/compwork/guillon/qemu-stm/build-next-x86_64/devimage/bin/qemu-arm -cpu cortex-a9"}
export armeabi=${armeabi:-hard}
export ARM_PROOT=${ARM_PROOT:-/home/compwork/guillon/proot/build-x86_64/devimage/bin/proot}
