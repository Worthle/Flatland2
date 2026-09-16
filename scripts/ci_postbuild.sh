#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 - "$repo_dir" "${1:-build}" "${CLANG_TIDY:-clang-tidy-14}" <<'PYTHON'
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import os
import re
import subprocess
import sys

root = Path(sys.argv[1]).resolve()
build = Path(sys.argv[2]).resolve()
tidy = sys.argv[3]
tasks = []
seen = set()
for package in ('flatland_server', 'flatland_plugins', 'flatland_viz'):
    database = build / package / 'compile_commands.json'
    if not database.is_file():
        sys.exit(f'Missing {database}; build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON')
    for entry in json.loads(database.read_text()):
        source = Path(entry['file']).resolve()
        if not source.is_relative_to(root / package) or 'thirdparty' in source.parts or source in seen:
            continue
        seen.add(source)
        tasks.append((database.parent, source))
if not tasks:
    sys.exit('No project compilation commands found.')

def check(task):
    directory, source = task
    result = subprocess.run([tidy, '-p', str(directory), '--warnings-as-errors=*',
        '--header-filter=' + re.escape(str(root)) + '/(flatland_server|flatland_plugins|flatland_viz)/(include|src|test)/',
        str(source)], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # Older clang-tidy releases can report compiler errors with exit status zero.
    failed = result.returncode != 0 or re.search(r'\berror:', result.stdout) is not None
    return source, failed, result.stdout

failed = False
with ThreadPoolExecutor(max_workers=int(os.environ.get('LINT_JOBS', '2'))) as pool:
    for source, error, output in pool.map(check, tasks):
        if error:
            print(output, flush=True)
            failed = True
        else:
            print(f'Checked {source.relative_to(root)}', flush=True)
sys.exit(1 if failed else 0)
PYTHON
