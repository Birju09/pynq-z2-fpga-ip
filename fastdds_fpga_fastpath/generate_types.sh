#!/bin/bash
# =============================================================================
# generate_types.sh
# Optional script to generate Fast DDS IDL type support using fastddsgen
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IDL_DIR="${SCRIPT_DIR}/idl"

if ! command -v fastddsgen &> /dev/null; then
    echo "[-] fastddsgen not found in PATH."
    echo "    Note: Handcrafted portable Fast DDS 2.x and 3.x type support"
    echo "    is already pre-generated in ${IDL_DIR}."
    exit 0
fi

echo "[+] Generating Fast DDS type support from ${IDL_DIR}/FastSample.idl..."
cd "${IDL_DIR}"
fastddsgen -replace -typeobject FastSample.idl

echo "[+] Generation complete."
