#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# update PATH to include kconfiglib scripts
export PATH=${DIR}:${PATH}

# PX4_NUTTX_APPS_DIR is set when PX4 builds against an external NuttX tree
# (the apps repository does not sit next to it as "apps").
APPS_DIR="${PX4_NUTTX_APPS_DIR:-../apps}"

make --no-print-directory --silent CONFIG_APPS_DIR="${APPS_DIR}" olddefconfig
