#!/bin/bash



test_command() {
    local cmd="$1"
    local expected_exit_code="$2"
    local expected_output="$3"

    local output
    output=$($cmd 2>&1)
    local exit_code=$?

    if [[ "${exit_code}" -ne "${expected_exit_code}" ]]; then
        echo "Exit code test:"
        echo "  Expected - ${expected_exit_code}"
        echo "  Actual - ${exit_code}"
        return 1
    fi

    if [[ "$output" != *"$expected_output"* ]]; then
        echo "Terminal output test:"
        echo "  Expected - ${$expected_output}"
        echo "  Actual - ${$output}"
        return 1
    fi

    return 0
}

set -oux pipefail

APP="/buildroot/build-ros1-app-Release/reprojection.reprojection_app"

test_command "$APP --config nonexistent.toml" 1 "Error parsing file 'nonexistent.toml' - File could not be opened for reading"