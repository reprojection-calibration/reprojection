#!/bin/bash

# NOTE(Jack): We do not enable debug output with -x because it makes the terminal test output impossible to interpret.
set -eou pipefail

test_command() {
    local cmd="$1"
    local expected_exit_code="$2"
    local expected_output="$3"

    # NOTE(Jack): Setting -e means that any non-zero exit code kills the program. But we need to be able to test failed
    # executions where the exit code is non-zero. Therefore for the part where we run the app we disable the -e flag and
    # re-enable it right after the app is done.
    set +e
    local output
    output=$($cmd 2>&1)
    local exit_code=$?
    set -e


    if [[ "${exit_code}" -ne "${expected_exit_code}" ]]; then
        echo "Exit code test:"
        echo "  Expected - ${expected_exit_code}"
        echo "  Actual - ${exit_code}"
        return 1
    fi

    if [[ "$output" != *"$expected_output"* ]]; then
        echo "Terminal output test:"
        echo "  Expected - ${expected_output}"
        echo "  Actual - ${output}"
        return 1
    fi

    return 0
}


test_command "${APP}" 1 "Missing --config flag"
test_command "${APP} --config nonexistent.toml" 1 "Error parsing file 'nonexistent.toml' - File could not be opened for reading"