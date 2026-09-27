"""Make the legacy ``controller`` package (raspberry/) importable for the tests."""

import sys
from pathlib import Path

_REPO_RASPBERRY = Path(__file__).resolve().parents[4] / "raspberry"
if (_REPO_RASPBERRY / "controller").is_dir() and str(_REPO_RASPBERRY) not in sys.path:
    sys.path.insert(0, str(_REPO_RASPBERRY))
