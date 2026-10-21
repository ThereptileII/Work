#!/usr/bin/env python3
"""Bounded, offline workerd smoke for the actual API proxy module.

Install outside the repository: npm install --prefix /tmp/skager-workerd-smoke
--no-audit --no-fund --save-exact workerd@1.20261013.1
Run with --workerd /tmp/skager-workerd-smoke/node_modules/.bin/workerd.
The runtime has only a local upstream service and no Internet capability.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

VERSION = '1.20261013.1'
DATE = '2026-10-13'
GENERIC = b'{"error":"The service is temporarily unavailable. Please try again."}'
SOURCE = Path(__file__).resolve().parents[1] / 'edge/cloudflare-worker.mjs'

ENTRY = '''import actual from './worker.mjs';
export default { fetch(request, env) {
 const path = new URL(request.url).pathname;
 if (path === '/__fixture_count') return env.UPSTREAM.fetch('https://fixture.invalid/count');
 return actual.fetch(new Request('https://beta.skager.app' + path, request));
}};
'''
UPSTREAM = '''let calls = 0;
export default { fetch(request) {
 const url = new URL(request.url);
 if (url.hostname === 'fixture.invalid' && url.pathname === '/count') return Response.json({calls});
 ++calls;
 if (url.href !== 'https://kelwgubkuvditalsbqpr.supabase.co/functions/v1/skager-web/api/auth/session')
  return new Response('unexpected upstream target', {status:500});
 const status = Number(/fixture_status=(\\d+)/.exec(request.headers.get('Cookie') || '')?.[1] || '200');
 const headers = new Headers({'Content-Type':'application/json'});
 if (status !== 200) {
  headers.set('Location', 'https://must-not-follow.invalid/synthetic-private');
  headers.set('Set-Cookie', 'private=synthetic-private');
  return new Response('synthetic-private', {status, headers});
 }
 headers.append('Set-Cookie', '__Host-skager_csrf=' + 'a'.repeat(64) + '; Secure; SameSite=Lax; Path=/');
 headers.append('Set-Cookie', '__Host-skager_session=fixture; Secure; HttpOnly; Path=/');
 return new Response(JSON.stringify({account:null, csrfToken:'a'.repeat(64)}), {headers});
}};
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workerd', type=Path, required=True)
    args = parser.parse_args()
    binary = args.workerd.resolve(strict=True)
    package = json.loads((binary.parent.parent / 'package.json').read_text())
    assert package['name'] == 'workerd' and package['version'] == VERSION, 'exact official workerd package required'
    assert subprocess.check_output([binary, '--version'], text=True, timeout=5).strip() == f'workerd {DATE}'
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='skager-worker-runtime-') as temporary:
        root = Path(temporary)
        (root / 'worker.mjs').write_bytes(SOURCE.read_bytes())
        (root / 'generated-assets.mjs').write_text('export default Object.freeze({});\n')
        (root / 'entry.mjs').write_text(ENTRY)
        (root / 'upstream.mjs').write_text(UPSTREAM)
        config = '''using Workerd = import "/workerd/workerd.capnp";
const config :Workerd.Config = (
 services = [
  (name = "main", worker = (modules = [
   (name = "entry.mjs", esModule = embed "entry.mjs"),
   (name = "worker.mjs", esModule = embed "worker.mjs"),
   (name = "generated-assets.mjs", esModule = embed "generated-assets.mjs")
  ], compatibilityDate = "DATE", globalOutbound = "upstream",
   bindings = [(name = "UPSTREAM", service = "upstream")])),
  (name = "upstream", worker = (modules = [(name = "upstream.mjs", esModule = embed "upstream.mjs")], compatibilityDate = "DATE")),
  (name = "internet", network = (allow = []))
 ],
 sockets = [(name = "http", address = "127.0.0.1:PORT", http = (), service = "main")]
);
'''.replace('DATE', DATE).replace('PORT', str(port))
        (root / 'config.capnp').write_text(config)
        def get(path, status=200):
            request = urllib.request.Request(f'http://127.0.0.1:{port}{path}', headers={
                'Cookie': f'fixture_status={status}; private=synthetic-private',
            })
            try:
                return urllib.request.urlopen(request, timeout=3)
            except urllib.error.HTTPError as response:
                return response
        with (root / 'runtime.log').open('w+') as log:
            process = subprocess.Popen([binary, 'serve', root / 'config.capnp'], stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 5
                while True:
                    try:
                        with get('/__fixture_count') as response:
                            assert json.load(response) == {'calls': 0}
                        break
                    except urllib.error.URLError:
                        assert process.poll() is None and time.monotonic() < deadline, 'runtime did not start'
                        time.sleep(.05)
                for count, status in enumerate([200, 301, 302, 303, 307, 308], 1):
                    with get('/api/auth/session', status) as response:
                        body = response.read(4096)
                        assert response.headers['Cache-Control'] == 'no-store'
                        assert response.headers['X-Content-Type-Options'] == 'nosniff'
                        assert "connect-src 'self'" in response.headers['Content-Security-Policy']
                        if status == 200:
                            assert response.status == 200
                            assert json.loads(body) == {'account': None, 'csrfToken': 'a' * 64}
                            cookies = response.headers.get_all('Set-Cookie')
                            assert len(cookies) == 2 and all('Secure;' in value for value in cookies)
                            assert 'SameSite=Lax' in cookies[0] and 'HttpOnly' in cookies[1]
                        else:
                            assert response.status == 503 and body == GENERIC
                            assert response.headers.get('Location') is None
                            assert response.headers.get('Set-Cookie') is None
                            assert 'synthetic-private' not in str(response.headers)
                    with get('/__fixture_count') as response:
                        assert json.load(response) == {'calls': count}, 'redirect followed or unexpected extra request'
                log.flush()
                log.seek(0)
                assert log.read().splitlines() == ['skager-worker:upstream-request'] * 5
            finally:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
    print(json.dumps({'workerd': VERSION, 'compatibilityDate': DATE, 'groups': 6,
                      'passed': 6, 'network': 'local service only; internet denied',
                      'workerSha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                      'runtimeSha256': hashlib.sha256(binary.read_bytes()).hexdigest()}))


if __name__ == '__main__':
    main()
