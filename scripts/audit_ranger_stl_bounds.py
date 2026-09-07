#!/usr/bin/env python3
# ################################
# Python: Compatibility wrapper — forwards to audit_ranger_geometry.py
# ################################
"""Deprecated entrypoint; use scripts/audit_ranger_geometry.py."""

from __future__ import annotations

import sys
from pathlib import Path

_GEOMETRY = Path(__file__).resolve().parent / "audit_ranger_geometry.py"


def main(argv=None) -> int:
    import runpy

    sys.argv = [str(_GEOMETRY)] + list(sys.argv[1:] if argv is None else argv)
    runpy.run_path(str(_GEOMETRY), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
