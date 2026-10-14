"""Operator artifact tests with inert existing binder fixtures; no real approval."""
import base64
from copy import deepcopy
from datetime import timedelta
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


def module(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), Path(__file__).with_name(name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


producer = module('produce-installed-updater-approval')
fixtures = module('test-installed-updater-approval')


class ProducerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.HandoffTests()
        self.fixture.setUp()
        self.records = {'request.json': self.fixture.request_bytes, 'approval.json': self.fixture.approval_bytes,
                        'approval-pins.json': fixtures.encode(dict(schema=1, purpose=producer.transport.PINS_PURPOSE,
                            requestSha256=producer.binding.sha256(self.fixture.request_bytes),
                            approvalSha256=producer.binding.sha256(self.fixture.approval_bytes), pins=self.fixture.pins))}
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output = self.root / 'originals'

    def retain(self, **changes):
        args = dict(records=self.records, output=self.output, run_id='700', run_attempt='1', now=self.fixture.now)
        args.update(changes)
        return producer.retain(**args)

    def test_originals_retained_exactly_and_binder_accepts_without_rewriting(self):
        name = self.retain()
        self.assertEqual(name, producer.transport.artifact_name('700', '1', producer.binding.sha256(self.fixture.request_bytes)))
        self.assertEqual({p.name: p.read_bytes() for p in self.output.iterdir()}, self.records)
        inputs = self.fixture.inputs()
        pins = json.loads((self.output / 'approval-pins.json').read_bytes())
        inputs.update(request_bytes=(self.output / 'request.json').read_bytes(),
                      approval_bytes=(self.output / 'approval.json').read_bytes(),
                      approved_request_sha256=pins['requestSha256'],
                      approved_approval_sha256=pins['approvalSha256'], approved_pins=pins['pins'])
        result = producer.binding.bind_approval(**inputs)
        self.assertEqual(result.original_approval_bytes, self.fixture.approval_bytes)

    def test_no_creation_or_overwrite_on_invalid_or_missing_original(self):
        for changed in ({k: v for k, v in self.records.items() if k != 'approval.json'},
                        dict(self.records, extra=b'not accepted'),
                        dict(self.records, **{'approval.json': b'{}\n'})):
            with self.subTest(changed=changed.keys()), self.assertRaises(ValueError):
                self.retain(records=changed)
            self.assertFalse(self.output.exists())
        self.output.mkdir()
        marker = self.output / 'keep'
        marker.write_bytes(b'original')
        with self.assertRaises(FileExistsError):
            self.retain()
        self.assertEqual(marker.read_bytes(), b'original')

    def test_mutated_pins_context_challenge_and_approval_refuse_before_write(self):
        for member, path, value in (
            ('approval-pins.json', ('requestSha256',), '0' * 64),
            ('approval-pins.json', ('approvalSha256',), '0' * 64),
            ('approval-pins.json', ('pins', 'brokerSha256'), '0' * 64),
            ('approval.json', ('runner', 'jobId'), '999'),
            ('approval.json', ('challenge',), '0' * 64),
            ('approval.json', ('purpose',), 'self-approved'),
        ):
            changed = deepcopy(self.records)
            value_object = json.loads(changed[member])
            target = value_object
            for part in path[:-1]:
                target = target[part]
            target[path[-1]] = value
            changed[member] = fixtures.encode(value_object)
            # Keep the outer approval hash consistent, testing the changed
            # semantic boundary instead of only a stale byte checksum.
            if member == 'approval.json':
                pins = json.loads(changed['approval-pins.json'])
                pins['approvalSha256'] = producer.binding.sha256(changed[member])
                changed['approval-pins.json'] = fixtures.encode(pins)
            with self.subTest(member=member, path=path), self.assertRaises(ValueError):
                self.retain(records=changed)
            self.assertFalse(self.output.exists())

    def test_expired_future_and_same_run_refuse(self):
        for args in ({'now': self.fixture.now + timedelta(minutes=20)},
                     {'now': self.fixture.now - timedelta(minutes=2)},
                     {'run_id': self.fixture.runner['runId']}, {'run_attempt': '0'}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.retain(**args)
            self.assertFalse(self.output.exists())

    def test_strict_canonical_bounded_base64_and_records(self):
        raw = self.fixture.request_bytes
        text = base64.b64encode(raw).decode('ascii')
        self.assertEqual(producer.decode(text), raw)
        for value in ('', text + '\n', 'not base64!', 'A' * 30000):
            with self.subTest(value_length=len(value)), self.assertRaises(ValueError):
                producer.decode(value)
        for value in (raw[:-1], raw + b'\n', b'{"schema":1,"schema":1}\n', b'NaN\n'):
            with self.subTest(value_length=len(value)), self.assertRaises(ValueError):
                self.retain(records=dict(self.records, **{'request.json': value}))
            self.assertFalse(self.output.exists())

    def test_symlink_directory_refuses(self):
        self.output.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.retain()
        self.assertFalse((self.root / 'approval.json').exists())

    def test_cli_wrong_context_has_generic_error_and_no_data_output(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(producer.sys, 'argv', ['producer']), patch.dict(producer.os.environ, {}, clear=True), \
                patch('sys.stdout', stdout), patch('sys.stderr', stderr):
            self.assertEqual(producer.main(), 1)
        self.assertEqual(stdout.getvalue(), '')
        self.assertEqual(stderr.getvalue(), producer.FAILURE + '\n')

    def test_cli_writes_only_validated_originals_and_upload_output(self):
        env = dict(GITHUB_ACTIONS='true', GITHUB_EVENT_NAME='workflow_dispatch',
                   GITHUB_REPOSITORY=producer.binding.REPOSITORY, GITHUB_JOB='approval',
                   GITHUB_WORKFLOW_REF=producer.binding.REPOSITORY + '/' + producer.transport.WORKFLOW + '@refs/heads/reviewed',
                   GITHUB_RUN_ID='700', GITHUB_RUN_ATTEMPT='1', RUNNER_TEMP=str(self.root),
                   GITHUB_OUTPUT=str(self.root / 'outputs'))
        env.update({name: base64.b64encode(self.records[filename]).decode('ascii') for filename, name in producer.INPUTS.items()})
        with patch.object(producer.sys, 'argv', ['producer']), patch.dict(producer.os.environ, env, clear=True), \
                patch.object(producer, 'datetime') as clock, patch('sys.stdout', io.StringIO()):
            clock.now.return_value = self.fixture.now
            self.assertEqual(producer.main(), 0)
        directory = self.root / 'installed-b0-approval-700-attempt1'
        self.assertEqual({p.name: p.read_bytes() for p in directory.iterdir()}, self.records)
        self.assertEqual(len((self.root / 'outputs').read_text().splitlines()), 2)


if __name__ == '__main__':
    unittest.main()
