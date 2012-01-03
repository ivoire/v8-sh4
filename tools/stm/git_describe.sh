#!/bin/sh
exec git describe --tags --dirty --abbrev=8 --always
