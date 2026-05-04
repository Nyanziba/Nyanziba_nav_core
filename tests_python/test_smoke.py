"""Minimum smoke test for the Python package scaffold.

The native ``_core`` extension only exists once the M7 pybind11 commit
lands, so we can't import it here yet. We just verify the namespace
package resolves and exposes ``__version__``.
"""

import texnitis_nav_core


def test_version_is_string():
    """Sanity check: the package advertises a version string."""
    assert isinstance(texnitis_nav_core.__version__, str)
    assert texnitis_nav_core.__version__
