#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null 2>&1 && pwd )"

# detect system info
if [[ "$(uname)" == "Darwin" ]]; then
    # macOS
    echo "Detected macOS, running install_deps_macos.sh"
    ${SCRIPT_DIR}/install_deps_macos.sh
elif [[ "$(uname)" == "Linux" ]]; then
    # Ubuntu
    echo "Detected Ubuntu, running install_deps_ubuntu.sh"
    ${SCRIPT_DIR}/install_deps_ubuntu.sh
else
    echo "Unsupported operating system"
    exit 1
fi