#!/bin/bash
# Minimal wrapper for build agent to ensure proper ESP-IDF environment
# This script ensures each waf command runs with the correct IDF environment

set -e

# Unset any existing IDF_PATH to avoid conflicts
unset IDF_PATH

# Get the action and remaining arguments
ACTION="$1"
shift

# Execute the action with proper environment
case "$ACTION" in
    configure|build|rover|upload|clean|distclean)
        # Source ESP-IDF and run waf command in a subshell
        (
            source modules/esp_idf/export.sh >/dev/null 2>&1
            ./waf "$ACTION" "$@"
        )
        ;;
    *)
        echo "Usage: $0 {configure|build|rover|upload|clean|distclean} [args...]"
        exit 1
        ;;
esac