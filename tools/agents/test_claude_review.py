"""Focused boundary tests; no network, Claude calls, repository edits or boat access."""
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

loader = importlib.machinery.SourceFileLoader('claude_review', str(Path(__file__).with_name('claude-review')))
spec = importlib.util.spec_from_loader(loader.name, loader)
r = importlib.util.module_from_spec(spec); loader.exec_module(r)


class ReviewBoundary(unittest.TestCase):
    def test_secret_and_escape_scopes_refused(self):
        for path in ('../x', '/etc/passwd', '.env', '.env.production', '.git/config',
                     'nested/key.pem', 'evidence/local/boat.json', '.claude/settings.json', 'src/a\n.py'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                r.safe_relative(path)

    def test_symlink_files_and_directories_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); (root/'real').mkdir(); (root/'real/a').write_text('data')
            (root/'link').symlink_to(root/'real', target_is_directory=True)
            (root/'file').symlink_to(root/'real/a')
            for name in ('link/a', 'file'):
                with self.assertRaises(ValueError): r.collect(root, [name])

    def test_link_swap_during_open_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'a').write_text('source');(root/'secret').write_text('private')
            real_open=r.os.open
            def swapped(path,*args,**kwargs):
                if str(path)=='a':
                    (root/'a').unlink();(root/'a').symlink_to(root/'secret')
                return real_open(path,*args,**kwargs)
            with patch.object(r.os,'open',side_effect=swapped), self.assertRaises(OSError):
                r.read_regular(root,'a')

    def test_directories_do_not_sweep_untracked_private_files(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); subprocess.run(['git','init','-q',d],check=True)
            (root/'src').mkdir(); (root/'src/a.py').write_text('ok')
            (root/'src/private.txt').write_text('not selected')
            subprocess.run(['git','-C',d,'add','src/a.py'],check=True)
            self.assertEqual(r.collect(root,['src']), {'src/a.py':b'ok'})

    def test_binary_and_oversized_files_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'x'; p.write_bytes(b'a\0b')
            with self.assertRaises(ValueError): r.collect(Path(d),['x'])
            p.write_bytes(b'x'*(2*1024**2+1))
            with self.assertRaises(ValueError): r.collect(Path(d),['x'])

    def test_hardlinks_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); (p/'private').write_text('private')
            r.os.link(p/'private',p/'source')
            with self.assertRaises(ValueError): r.read_regular(p,'source')

    def test_git_scopes_are_literal_and_ignore_inherited_git_environment(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); subprocess.run(['git','init','-q',d],check=True)
            for name in ('[src]', 's'):
                (root/name).mkdir(); (root/name/'a.py').write_text(name)
            subprocess.run(['git','-C',d,'add','.'],check=True)
            with patch.dict(r.os.environ,{'GIT_DIR':'/does/not/exist'}):
                self.assertEqual(r.collect(root,['[src]']),{'[src]/a.py':b'[src]'})

    def test_terminal_output_has_no_control_or_bidi_characters(self):
        self.assertEqual(r.terminal_text('ok\x1b\x07\u202e\n'), 'ok\n')

    def test_frozen_capture_does_not_follow_later_edits(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'x';p.write_text('before'); captured=r.collect(Path(d),['x'])
            p.write_text('after');self.assertEqual(captured['x'],b'before')

    def test_real_os_boundary_denies_source_writes_and_hides_host(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); snapshot=root/'snapshot';snapshot.mkdir();(snapshot/'sample').write_text('frozen')
            home=root/'home';(home/'.claude').mkdir(parents=True)
            secret=root/'host-secret';secret.write_text('must not be visible')
            cred=root/'credential';cred.write_text('{}')
            cmd=r.sandbox(snapshot,home,Path('/usr/bin/python3'),cred)+['--','/usr/bin/python3','-c',
                'import os,pathlib\n'
                'try: pathlib.Path("/review/sample").write_text("bad")\n'
                'except OSError: pass\n'
                'else: raise AssertionError("snapshot writable")\n'
                f'assert not pathlib.Path({str(secret)!r}).exists()\n'
                'assert not pathlib.Path("/home/standard").exists()\n'
                'assert not os.environ.get("SSH_AUTH_SOCK")\n'
                'assert not os.environ.get("ANTHROPIC_BASE_URL")\n'
                'assert pathlib.Path("/review/sample").read_text()=="frozen"\n']
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual((snapshot/'sample').read_text(),'frozen')

    def test_subagents_have_no_shell_or_write_tools(self):
        cmd=r.claude_args('security',r.DEFAULT_MODEL,1)
        agents=json.loads(cmd[cmd.index('--agents')+1])
        self.assertEqual(set(agents),set(r.KINDS))
        for agent in agents.values():
            self.assertEqual(set(agent['tools']),{'Read','Glob','Grep'})
            self.assertEqual(agent['permissionMode'],'dontAsk')
            self.assertEqual(agent['model'],'inherit')
        self.assertIn('--restricted',cmd); self.assertNotIn('--dangerously-skip-permissions',cmd)
        self.assertIn('Read(//review/**)',cmd[cmd.index('--allowedTools')+1])
        self.assertIn('Read(//home/**)',cmd[cmd.index('--disallowedTools')+1])

    def test_pinned_model_observed_in_parent_and_child(self):
        for model in r.MODELS:
            events=[{'type':'assistant','message':{'model':model}},
                    {'type':'assistant','parent_tool_use_id':'child','message':{'model':model}},
                    {'type':'system','message':'notice'},
                    {'type':'assistant','message':{'model':'<synthetic>'}}]
            self.assertEqual(r.verified_models(events,model),[model])

    def test_model_fallback_or_missing_model_is_not_accepted(self):
        for events in ([],[{'type':'assistant','message':{'model':'claude-sonnet-5-5'}}],
                       [{'type':'assistant','message':{'model':'claude-opus-5'}},
                        {'type':'assistant','message':{}}],
                       [{'type':'assistant','message':{'model':'claude-opus-5'}},
                        {'type':'assistant','parent_tool_use_id':'child','message':{'model':'claude-opus-5-5'}}]):
            with self.subTest(events=events), self.assertRaises(RuntimeError):
                r.verified_models(events,'claude-opus-5')

    def test_timeout_is_a_failure_with_partial_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            with self.assertRaisesRegex(RuntimeError,'timed out'):
                r.execute(['/usr/bin/python3','-c','import time;print("started",flush=True);time.sleep(30)'],
                          '',p/'events',p/'errors',0.1)
            self.assertEqual((p/'events').read_text(),'started\n')


if __name__=='__main__': unittest.main()
