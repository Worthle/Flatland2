#!/bin/bash
set -e
if [[ "${FLATLAND_RENDERER:-cpu}" == nvidia ]]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    unset LIBGL_ALWAYS_SOFTWARE
else
    export LIBGL_ALWAYS_SOFTWARE=1
    unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME
fi
source /opt/ros/humble/setup.bash
source /flatland_ws/install/setup.bash
exec "$@"
