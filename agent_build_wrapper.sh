#!/bin/bash
# Minimal wrapper for build agent to ensure proper ESP-IDF environment
# This script ensures each waf command runs with the correct IDF environment

set -e

# Allow override: IDF_PATH_OVERRIDE=/path/to/idf ./agent_build_wrapper.sh rover -j 16
if [ -n "$IDF_PATH_OVERRIDE" ]; then
    export IDF_PATH="$IDF_PATH_OVERRIDE"
elif [ -z "$IDF_PATH" ]; then
    # Default to IDF 6.x if installed, otherwise submodule
    IDF6=$(ls -d /opt/espressif/esp-idf-v6* 2>/dev/null | sort -rV | head -1)
    if [ -n "$IDF6" ]; then
        export IDF_PATH="$IDF6"
    fi
fi

# Get the action and remaining arguments
ACTION="$1"
shift

# Execute the action with proper environment
case "$ACTION" in
    configure|build|rover|upload|clean|distclean)
        # Source ESP-IDF and run waf command in a subshell
        (
            # IDF_PATH should be set by now (from override, env, or auto-detect above)
            if [ -z "$IDF_PATH" ]; then
                export IDF_PATH="$(pwd)/modules/esp_idf"
            fi
            source "$IDF_PATH/export.sh" >/dev/null 2>&1
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
