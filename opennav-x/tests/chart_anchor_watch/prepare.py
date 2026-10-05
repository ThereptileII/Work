#!/usr/bin/env python3
"""Extract actual pinned/patched ring painters for an offline renderer fixture."""
import argparse
from pathlib import Path


def function(text, signature):
    start = text.index(signature)
    body = text.index('{', start)
    depth = 1
    end = body + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


parser = argparse.ArgumentParser()
parser.add_argument('--prepared', type=Path, required=True)
parser.add_argument('--pinned', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
patched = (args.prepared / 'gui/src/chcanv.cpp').read_text(encoding='utf-8')
original = (args.pinned / 'gui/src/chcanv.cpp').read_text(encoding='utf-8')
radius = 'double ChartCanvas::GetAnchorWatchRadiusPixels('
if function(patched, radius) != function(original, radius):
    raise RuntimeError('Anchor radius/projection changed')
signature = 'void ChartCanvas::DrawAnchorWatchPoints('
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(function(patched, signature) + '\n' +
    function(original, signature).replace('::DrawAnchorWatchPoints(', '::DrawOriginalAnchorWatchPoints(', 1) + '\n' +
    function((args.prepared / 'gui/src/waypointman_gui.cpp').read_text(encoding='utf-8'),
             'bool WayPointmanGui::IsPinnedAnchor(') + '\nnamespace opennav::ui {\n' +
    function((Path(__file__).resolve().parents[2] / 'src/ui/Controls.cpp').read_text(encoding='utf-8'),
             'wxColour Colour(') + '\n}\n', encoding='utf-8', newline='\n')
