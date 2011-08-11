#!/bin/sh

SPUTNIK_ZIP='/home/compwork/duraffort/pub/sputniktests-v1.zip'

QEMU=${QEMU-/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4}
TIMEOUT=${TIMEOUT-"/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout 600"}
TARGET_ROOT=${TARGET_ROOT-/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target}
RUN_PREFIX=${RUN_PREFIX-"env QEMU_ASSUME_KERNEL=2.6.30 $TIMEOUT $QEMU -distro -L $TARGET_ROOT -x $PWD -cwd $PWD"}

SHELL="$RUN_PREFIX ./shell_g"

# Get the test suite
echo "=== Fetching an inflating the test suite ==="
rm -rf AUTHORS lib LICENSE tests tools
unzip -q $SPUTNIK_ZIP

# Copy the shell to run
echo "=== Copying the shell ==="
cp -f ../../shell_g ./

# Run the test suite
echo "=== Running the test suite ==="
python tools/sputnik.py --command "$SHELL"
