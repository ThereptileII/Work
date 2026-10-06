#!/usr/bin/env python3
"""Emit the source-derived product identity for native Windows resources."""
from pathlib import Path
from product_version import read_product_version, windows_product_version

version = read_product_version(Path(__file__).resolve().parents[1] / 'src/application/Version.h')
print(version)
print(','.join(map(str, windows_product_version(version))))
