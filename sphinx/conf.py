"""Sphinx configuration for the texnitis_nav_core Python API reference.

Implementation lands in milestone M7. The current scaffold only sets the
minimum required values so ``sphinx-build`` succeeds once the package becomes
importable.
"""

from __future__ import annotations

import sys
from pathlib import Path

project = "texnitis_nav_core"
author = "Nyanziba"
release = "0.0.1"

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root / "python"))

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"
html_static_path: list[str] = []

autodoc_default_options = {
    "members": True,
    "undoc-members": False,
    "show-inheritance": True,
}

myst_enable_extensions = ["colon_fence"]
