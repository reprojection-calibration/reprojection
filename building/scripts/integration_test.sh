#!/bin/bash

# NOTE(Jack): We do not enable debug output with -x because it makes the terminal test output impossible to interpret.
set -eou pipefail

test_command() {
    local cmd="${1}"
    local expected_exit_code="${2}"
    local expected_output="${3}"

    # NOTE(Jack): Setting -e means that any non-zero exit code kills the program. But we need to be able to test failed
    # executions where the exit code is non-zero. Therefore for the part where we run the app we disable the -e flag and
    # re-enable it right after the app is done.
    set +e
    local output
    output=$($cmd 2>&1)
    local exit_code=${?}
    set -e

    echo "Command under test: ${cmd}"
    if [[ "${exit_code}" -ne "${expected_exit_code}" ]]; then
        echo "Exit code test:"
        echo "  Expected - ${expected_exit_code}"
        echo "  Actual - ${exit_code}"

        echo "Failed"
        return 1
    fi

    if [[ "${output}" != "$expected_output" ]]; then
        echo "Terminal output test:"
        echo "  Expected - ${expected_output}"
        echo "  Actual - ${output}"

        echo "Failed"
        return 1
    fi

    echo "Test passed with exit code: '${exit_code}' and terminal output: '${output}'"
    return 0
}

test_command "${APP} --config nonexistent.toml" \
    1 \
    "Missing --data flag"

test_command "${APP} --data nonexistent.data" \
    1 \
    "Missing --config flag"

# Running the program with a invalid config file (i.e. nonexistent, incomplete, invalid etc.) is an error.
test_command "${APP} --config nonexistent.toml --data nonexistent.data" \
    1 \
    "Error parsing file 'nonexistent.toml' - File could not be opened for reading on line (0)"

test_command "${APP} --config /temporary/code/test_data/minimum_config.toml --data nonexistent.data" \
    0 \
    ""