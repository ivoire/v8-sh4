#
# Site host tools specifications for STM GNB site
#
export TIMEOUT=${TIMEOUT:-/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout}
export SH4_TOOL_PREFIX=${SH4_TOOL_PREFIX:-/sw/st/gnu_compil/comp/st40/st40-linux/4.5.2-76/bin/sh4-linux}
export SH4_TARGET_ROOT=${SH4_TARGET_ROOT:-/sw/st/gnu_compil/comp/st40/st40-linux/4.5.2-76/target}
export SH4_QEMU=${SH4_QEMU:-"/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4 -R"}
export QEMU_ASSUME_KERNEL=2.6.30
