#
# sh4 cross build specification
#

# Cross compilation tools
export TOOL_PREFIX=${SH4_TOOL_PREFIX}
export CXX=$TOOL_PREFIX-g++
export AR=$TOOL_PREFIX-ar
export RANLIB=$TOOL_PREFIX-ranlib
export CC=$TOOL_PREFIX-cc
export LD=$TOOL_PREFIX-ld
export CCFLAGS="$CCFLAGS"

# Target file system
export TARGET_ROOT=${SH4_TARGET_ROOT}

# Emulation tools
export QEMU=${SH4_QEMU}
export XCCTEST_OPTS="-stack_size=256" # for QEMU we use a 256kb stack size (default is 512)

# Default v8 settings for sh4
debuggersupport=off
