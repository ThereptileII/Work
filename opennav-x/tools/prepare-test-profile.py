#!/usr/bin/env python3
"""Create a NEW isolated, disconnected OpenCPN visual-test profile only."""
import argparse
from pathlib import Path
import re

parser = argparse.ArgumentParser()
parser.add_argument("--build", type=Path, required=True)
parser.add_argument("--profile", type=Path, required=True)
args = parser.parse_args()
config = (args.build / "include/config.h").read_text()
version = re.search(r'#define VERSION_FULL "([^"]+)"', config).group(1)
date = re.search(r'#define VERSION_DATE "([^"]+)"', config).group(1)
# Refuse to overwrite a real profile or a previous run's data.
args.profile.mkdir(parents=True, exist_ok=False)
(args.profile / "OPENNAV_TEST_PROFILE").write_text("Disposable disconnected UI test.\n")
(args.profile / "opencpn.conf").write_text(
    "[Settings]\n"
    f"ConfigVersionString=Version {version} Build {date}\n"
    "NavMessageShown=1\n"
    "ShowStatusBar=1\n"
    "ShowMenuBar=1\n"
    "[Settings/GlobalState]\n"
    "FrameWinX=1280\nFrameWinY=800\nFrameWinPosX=0\nFrameWinPosY=0\n"
    "FrameMax=0\n"
)
print(args.profile.resolve())
