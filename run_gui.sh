#!/usr/bin/env bash
# Launch the crankcam GUI, creating/reusing a local venv and installing
# the python/ package into it as needed. No manual venv steps required.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi

"$VENV_DIR/bin/pip" install -q --upgrade pip
"$VENV_DIR/bin/pip" install -q -e "$SCRIPT_DIR/python"

exec "$VENV_DIR/bin/crankcam-gui" "$@"
