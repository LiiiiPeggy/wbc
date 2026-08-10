#!/usr/bin/env python3
# ################################
# Python: rewrite package:// mesh URIs to file:// begin
# ################################
"""Expand package:// mesh paths in a URDF so RViz/resource_retriever
does not call rospack (avoids 'too many positional options' spam)."""
from __future__ import print_function
import os
import re
import sys

import rospkg


def main():
    if len(sys.argv) < 2:
        sys.stderr.write("usage: urdf_package_to_file.py <urdf_path>\n")
        return 1
    path = os.path.abspath(sys.argv[1])
    with open(path, "r") as f:
        text = f.read()
    rp = rospkg.RosPack()
    cache = {}
    # urdf lives in <pkg>/urdf/... — fallback when rospack cannot find pkg
    urdf_pkg_fallback = os.path.dirname(os.path.dirname(path))

    def repl(m):
        pkg, rel = m.group(1), m.group(2)
        if pkg not in cache:
            try:
                cache[pkg] = rp.get_path(pkg)
            except rospkg.ResourceNotFound:
                cache[pkg] = urdf_pkg_fallback
        return 'filename="file://%s/%s"' % (cache[pkg], rel)

    out = re.sub(
        r'filename="package://([^/]+)/([^"]+)"',
        repl,
        text,
    )
    sys.stdout.write(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
# ################################
# Python: rewrite package:// mesh URIs to file:// end
# ################################
