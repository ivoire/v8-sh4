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
snapshot=${snapshot:-off}
regexp=${regexp:-native}
profilingsupport=${profilingsupport:-off}
debuggersupport=${debuggersupport:-on}
backtrace=${backtrace:-off}
library=${library:-shared}
armeabi=${armeabi:-soft}
vfp3=${vfp3:-on}
logging=${logging:-off}
prof=${prof:-off}
tests=${tests:-""}
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
make ${arch}.${mode} snapshot=${snapshot} regexp=${regexp} profilingsupport=${profilingsupport} debuggersupport=${debuggersupport} backtrace=${backtrace} library=${library} armeabi=${armeabi} vfp3=${vfp3} logging=${logging} prof=${prof} ${jobs+-j$jobs} "$@"

#  sh4.debug snapshot=off regexp=interpreted debuggersupport=off -j4
