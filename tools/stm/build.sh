#!/bin/sh

set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`

# Default architecture: native one
arch=${arch:-""}
site=${site:-default}
jobs=${jobs-1}

# Load the global flags (local and site)
[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
# Load default flags for this arch
[ -f ${pdir}/source_flags_${arch}.sh ] && . ${pdir}/source_flags_${arch}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh

# Define default build flags
mode=${mode:-release}
snapshot=${snapshot:-on}		# Default: on
regexp=${regexp:-native}		# Default: native
debuggersupport=${debuggersupport:-on}	# Default: on
backtrace=${backtrace:-on}		# Default: on
library=${library:-shared}		# Default: static
profile_gcov=${profile_gcov:-off}

# gcov profiling only works in debug mode
if [ "$profile_gcov" = on -a "$mode" = release ]; then
    echo "error: cannot compile for both mode=release and profile_gcov=on. Use mode=debug for coverage." >&2
    exit 1
fi
# Add the right flags
if [ "$profile_gcov" = on ]; then
  export CFLAGS="--coverage"
  export CXXFLAGS="--coverage"
  export LDFLAGS="--coverage"
fi

set -x
make ${arch}.${mode} snapshot=${snapshot} regexp=${regexp} debuggersupport=${debuggersupport} backtrace=${backtrace} library=${library} ${jobs+-j$jobs} "$@"

#  sh4.debug snapshot=off regexp=interpreted debuggersupport=off -j4
