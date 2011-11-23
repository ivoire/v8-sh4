#!/bin/sh
#PATH=/opt/qemu-stm/bin/:/opt/qemu-stm/bin/:/usr/local/bin:/usr/bin:/bin:/usr/games:/usr/lib/java/bin:/usr/lib/kde4/libexec LD_LIBRARY_PATH=/opt/qemu-stm/lib /sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/proot -W -Q qemu-sh4 /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target/ ./shell_g
QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}
${RUN_PREFIX} ./shell_g
