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

