# Build the documentation

From the repository root, using Python 3.10 or newer:

```bash
python3 -m venv .work/docs-venv
.work/docs-venv/bin/pip install -r docs/requirements.txt
.work/docs-venv/bin/sphinx-build -b html -n -W --keep-going docs .work/docs-html
```

Open `.work/docs-html/index.html` in a browser. The build needs no ROS runtime.
Generated files and the virtual environment stay in the ignored `.work/` folder.

The plugin catalog in `included_plugins/plugin_catalog.rst` compares the current
plugin registry with the pinned upstream Humble revision. Each plugin reference
belongs in `included_plugins/` and must be listed in `index.rst`. Keep configuration
examples aligned with the public source's parameter readers and ROS interfaces;
use generic names and bundled assets for examples.
