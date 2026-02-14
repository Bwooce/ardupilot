#!/bin/bash
# Minimal wrapper for build agent to ensure proper ESP-IDF environment
# This script ensures each waf command runs with the correct IDF environment

set -e

# Allow override: IDF_PATH_OVERRIDE=/opt/espressif/esp-idf-v6.0 ./agent_build_wrapper.sh rover -j 16
if [ -n "$IDF_PATH_OVERRIDE" ]; then
    export IDF_PATH="$IDF_PATH_OVERRIDE"
else
    unset IDF_PATH
fi

# Get the action and remaining arguments
ACTION="$1"
shift

# Execute the action with proper environment
case "$ACTION" in
    configure|build|rover|upload|clean|distclean)
        # Source ESP-IDF and run waf command in a subshell
        (
            if [ -n "$IDF_PATH" ]; then
                source "$IDF_PATH/export.sh" >/dev/null 2>&1
            else
                source modules/esp_idf/export.sh >/dev/null 2>&1
            fi
            # Report which IDF we're building with
            echo "ESP-IDF: ${IDF_PATH} ($(python3 -c "import re; v=open('${IDF_PATH}/components/esp_common/include/esp_idf_version.h').read(); print(f'v{re.search(r\"MAJOR\s+(\d+)\",v).group(1)}.{re.search(r\"MINOR\s+(\d+)\",v).group(1)}.{re.search(r\"PATCH\s+(\d+)\",v).group(1)}')" 2>/dev/null || echo "version unknown"))"
            ./waf "$ACTION" "$@"
        )
        ;;
    *)
        echo "Usage: $0 {configure|build|rover|upload|clean|distclean} [args...]"
        echo "Set IDF_PATH_OVERRIDE to use a different ESP-IDF installation"
        exit 1
        ;;
esac
