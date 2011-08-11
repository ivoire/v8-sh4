#!/bin/sh

SPUTNIK_ZIP='/home/compwork/duraffort/pub/sputniktests-v1.zip'
SHELL='shell_g'
SHELL_ENV='/home/compwork/projects/proot/release/v0.6/x86_64/proot -W -Q /home/compwork/projects/qemu/release/r0/x86_64/qemu-sh4 /home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target/'

# Get the test suite
rm -rf AUTHORS lib LICENSE tests tools
unzip $SPUTNIK_ZIP

# Copy the shell to run
cp -f ../../$SHELL ./

# Run the test suite
python tools/sputnik.py --command "$SHELL_ENV ./$SHELL"
