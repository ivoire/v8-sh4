#
# arm cross build specification
#

# Cross compilation tools
export TOOL_PREFIX=${ARM_TOOL_PREFIX}
export CXX=$TOOL_PREFIX-g++
export AR=$TOOL_PREFIX-ar
export RANLIB=$TOOL_PREFIX-ranlib
export CC=$TOOL_PREFIX-cc
export LD=$TOOL_PREFIX-ld
export CCFLAGS="$CCFLAGS"

# Target file system
export TARGET_ROOT=${ARM_TARGET_ROOT}

# Emulation tools
export QEMU=${ARM_QEMU}
export PROOT=${ARM_PROOT}
export XCCTEST_OPTS="-stack_size=128 $XCCTEST_OPTS" # for QEMU we use a 128kb stack size (default is 512)

