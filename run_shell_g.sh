#!/bin/sh
#PATH=/opt/qemu-stm/bin/:/opt/qemu-stm/bin/:/usr/local/bin:/usr/bin:/bin:/usr/games:/usr/lib/java/bin:/usr/lib/kde4/libexec LD_LIBRARY_PATH=/opt/qemu-stm/lib /sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/proot -W -Q qemu-sh4 /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target/ ./shell_g
/home/compwork/projects/proot/release/v0.6/x86_64/proot -W -Q /home/compwork/projects/qemu/release/r0/x86_64/qemu-sh4 /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target/ ./shell_g
