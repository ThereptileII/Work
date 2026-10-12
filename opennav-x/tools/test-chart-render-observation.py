#!/usr/bin/env python3
"""Compile real core/private dispatch with inert terminal boundaries, offline.

Prepared inputs must come from the pinned source recipes. This checks copied
observation semantics and actual callsite wiring, not native canvas/GPU output.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('loader', ROOT / 'tools/verify-anchor-loader.py')
loader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loader)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_wiring(source, private):
    cpp = (source / 'libs/s52plib/src/s52plib.cpp').read_text()
    raster = loader.function(cpp, 'bool s52plib::RenderRasterSymbol(')
    assert raster.index('m_presentationRasterSubmitted = false;') < raster.index('double scale_factor')
    assert 'glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);\n#ifdef' in raster
    assert 'm_presentationRasterSubmitted = true;' in raster
    assert raster.count('m_presentationRasterSubmitted =\n') == 2
    chart = (source / ('src/eSENCChart.cpp' if private else 'gui/src/s57chart.cpp')).read_text()
    signature = ('ListOfPI_S57Obj *eSENCChart::GetObjRuleListAtLatLon(' if private else
                 'ListOfObjRazRules *s57chart::GetObjRuleListAtLatLon(')
    if private:
        chart_query = chart[chart.index('//ListOfObjRazRules *s57chart::GetObjRuleListAtLatLon'):]
    else:
        chart_query = chart
    query = loader.function(chart_query, signature)
    assert 'm_presentationObservation.BeginQuery(' in query
    assert ('m_presentationObservation.Select(cobj, pObj);' if private else
            'm_presentationObservation.Select(selected_rules[i]->obj, selected_rules[i]->obj);') in query
    description = loader.function(chart, ('wxString eSENCChart::CreateObjDescriptions(' if private else
                                         'wxString s57chart::CreateObjDescriptions('))
    assert 'm_presentationObservation.Describe(' in description
    assert 'if (ps52plib && ps52plib->PresentationCaLightsEnabled())' in description
    return cpp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--private-source', type=Path)
    mode.add_argument('--core-only', action='store_true',
                      help='Explicitly test core only; no private-adapter claim.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    receipts = []
    sources = [('core', args.source, 'OPENNAV_X', False)]
    if not args.core_only:
        sources.append(('private', args.private_source, 'SKAGER_OCHARTS_ADAPTER', True))
    for name, source, guard, private in sources:
        cpp = check_wiring(source, private)
        methods = ''.join(loader.function(cpp, signature) for signature in (
            'int s52plib::RenderSY(', 'void s52plib::RenderPresentationCaLightPoint('))
        for enabled in (True, False):
            destination = args.output / (name if enabled else name + '-standard')
            destination.mkdir(exist_ok=True)
            (destination / 'actual-render-observation.inc').write_text(methods)
            executable = destination / 'test'
            command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                       '-I' + str(ROOT / 'src'), '-I' + str(destination)]
            if enabled:
                command += ['-D' + guard, '-DOBSERVATION_ENABLED']
            command += [str(ROOT / 'tests/chart_render_observation_test.cpp'), '-o', str(executable)]
            subprocess.run(command, check=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
            print(destination.name + ': ' + result.stdout.strip(), flush=True)
            receipts.append({'name': destination.name, 'command': command,
                             'dispatchSha256': hashlib.sha256(methods.encode()).hexdigest(),
                             'sourceSha256': sha(source / 'libs/s52plib/src/s52plib.cpp'),
                             'stdout': result.stdout, 'exit': result.returncode})
    (args.output / 'receipt.json').write_text(json.dumps({
        'helperSha256': sha(ROOT / 'src/integration/ChartRenderObservation.h'),
        'fixtureSha256': sha(ROOT / 'tests/chart_render_observation_test.cpp'),
        'mode': 'core-only' if args.core_only else 'paired-core-private',
        'runs': receipts,
        'limits': 'Real dispatch with inert alias/projection/painter seams; no native canvas, GPU readback, private DLL or boat execution.'
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
