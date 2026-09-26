"""Verify compiled runtime archives cannot supply synthetic generators.

The separately linked test archive is a positive control, so a broken symbol
reader cannot falsely pass this gate. No application or hardware is launched.
"""
import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', required=True)
    parser.add_argument('--msvc', action='store_true')
    parser.add_argument('--fixtures', required=True)
    parser.add_argument('runtime', nargs='+')
    args = parser.parse_args()
    generators = ('DemoFixture', 'DemoAis', 'DemoSource', 'SimulatorFixture',
                  'PreviewEnergyModel', 'SimulatedAutopilot')

    def symbols(path):
        options = ['/dump', '/nologo', '/linkermember:1'] if args.msvc else [
            '-C', '-g', '--defined-only']
        result = subprocess.run([args.tool, *options, path], check=True,
                                capture_output=True, text=True, errors='replace',
                                timeout=30)
        if not result.stdout.strip():
            raise AssertionError(f'No symbol output for {path}')
        return result.stdout

    fixture_symbols = symbols(args.fixtures)
    for name in generators:
        if name not in fixture_symbols:
            raise AssertionError(f'Positive control missing generator {name}')
    for archive in args.runtime:
        defined = symbols(archive)
        for name in generators:
            if name in defined:
                raise AssertionError(f'Runtime archive {archive} defines {name}')
    print('All six generators present only in dedicated fixture archive; '
          'Vessel Data, SmartNav and adapter runtime definitions absent')


if __name__ == '__main__':
    main()
