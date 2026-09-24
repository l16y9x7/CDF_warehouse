#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
"${SCRIPT_DIR}/stop-mocks.sh"
"${SCRIPT_DIR}/start-mocks.sh"
