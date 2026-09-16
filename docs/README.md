# Plugin and visualization documentation

```bash
python3 -m venv .work/docs-venv
.work/docs-venv/bin/pip install -r docs/requirements.txt
.work/docs-venv/bin/sphinx-build -b html -n -W docs .work/docs-html
```

Open `.work/docs-html/index.html` in a browser.
