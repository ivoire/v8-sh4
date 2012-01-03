#!/bin/sh
set -e
dir=`dirname $0`
version=`$dir/git_describe.sh`
git archive --format tar --prefix v8-${version}/ HEAD | gzip - > v8-${version}.tar.gz
