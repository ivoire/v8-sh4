#
# Site host tools specifications for a default STM linux installation
#
export SH4_TOOL_PREFIX=${SH4_TOOL_PREFIX:-/opt/STM/STLinux-2.4/devkit/sh4/bin/sh4-linux}
export SH4_TARGET_ROOT=${SH4_TARGET_ROOT:-/opt/STM/STLinux-2.4/devkit/sh4/target}
export SH4_QEMU=${SH4_QEMU:-"/opt/STM/STLinux-2.4/devkit/sh4/bin/qemu-sh4 -R"}

export ARM_TOOL_PREFIX=${ARM_TOOL_PREFIX:-/opt/STM/STLinux-2.4/devkit/armv7/bin/armv7-linux}
export ARM_TARGET_ROOT=${ARM_TARGET_ROOT:-/opt/STM/STLinux-2.4/devkit/armv7/target}
export ARM_QEMU=${ARM_QEMU:-"/opt/STM/STLinux-2.4/devkit/armv7/bin/qemu-arm -R -cpu cortex-a9"}
export armeabi=${armeabi:-hard}
