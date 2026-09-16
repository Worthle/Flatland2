#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 - "$repo_dir" "${CLANG_FORMAT:-clang-format-3.8}" <<'PYTHON'
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
formatter = sys.argv[2]
failed = []
for package in ('flatland_server', 'flatland_plugins', 'flatland_viz'):
    for path in sorted((root / package).rglob('*')):
        if path.suffix not in ('.cpp', '.h', '.hpp', '.c') or 'thirdparty' in path.parts:
            continue
        formatted = subprocess.check_output([formatter, '--style=file', str(path)])
        if formatted != path.read_bytes():
            failed.append(str(path.relative_to(root)))
if failed:
    sys.exit('Formatting required:\n' + '\n'.join(failed))
print('C++ formatting checked.')
PYTHON
