#!/usr/bin/env bash
set -e

[ "$DEBUG" = "" ] || set -x

pdir=`dirname $0`
arch=${arch:-""} # if not defined, build in native mode
site=${site:-default}
mode=${mode:-release}
[ -f ${pdir}/source_local.sh ] && . ${pdir}/source_local.sh
[ -f ${pdir}/source_site_${site}.sh ] && . ${pdir}/source_site_${site}.sh
[ -f ${pdir}/source_${arch}.sh ] && . ${pdir}/source_${arch}.sh
regexp=${regexp:-interpreted}
profilingsupport=${profilingsupport:-off}
debuggersupport=${debuggersupport:-on}
library=${library:-shared}
armeabi=${armeabi:-soft}
tests=${tests:-""}
vfp3=${vfp3:-on}
jobs=${jobs:-4}

[ "$library" = shared ] && LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH

scons ${arch:+arch=${arch}} mode=${mode} regexp=${regexp} profilingsupport=${profilingsupport} debuggersupport=${debuggersupport} library=${library} armeabi=${armeabi} vfp3=${vfp3} -j ${jobs}
scons ${arch:+arch=${arch}} mode=${mode} regexp=${regexp} profilingsupport=${profilingsupport} debuggersupport=${debuggersupport} library=${library} armeabi=${armeabi} vfp3=${vfp3} -j ${jobs} sample=shell
tools/test.py -v ${arch:+--arch=${arch}} --build-only --mode=${mode} -S regexp=${regexp} -S profilingsupport=${profilingsupport} -S debuggersupport=${debuggersupport}  -S library=${library} -S armeabi=${armeabi} -S vfp3=${vfp3} -j ${jobs} ${tests}
