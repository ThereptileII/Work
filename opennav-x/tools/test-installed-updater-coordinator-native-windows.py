"""Actual Windows adapter/Go bridge fixture only; never approval or installation.

Invoked by TestNativeQualificationChallengePythonAdapterStreaming with its own
compiled test executable and disposable protected parent. No SCM/profile/product
operation is available. The proposed consumption hashes are inert fixture values.
"""
import json
import os
from pathlib import Path
import secrets
import sys

import installed_updater_coordinator as coordinator


def fixture_record(environment, challenge):
    coordinator.require(environment.get('GITHUB_ACTIONS') == 'true' and
                        environment.get('GITHUB_REPOSITORY') == coordinator.binding.REPOSITORY)
    record = dict(schema=1, purpose='installed-updater-b0-consumption',
        repository=environment['GITHUB_REPOSITORY'], headSha=environment['GITHUB_SHA'],
        runId=environment['GITHUB_RUN_ID'], runAttempt=environment['GITHUB_RUN_ATTEMPT'],
        jobId='1', jobName='Native adapter inert fixture', challenge=challenge,
        requestSha256='1' * 64, approvalSha256='2' * 64)
    coordinator.binding._hex(record['headSha'], 40)
    coordinator.binding._hex(challenge)
    for name in ('runId', 'runAttempt'):
        coordinator.binding._decimal(record[name])
    return record


def run_fixture(executable, sha256, parent, evidence, sid):
    coordinator.require(os.name == 'nt')
    record = fixture_record(os.environ, secrets.token_hex(32))
    executable, parent, evidence = map(Path, (executable, parent, evidence))
    coordinator.require(evidence.is_dir() and parent.is_dir() and not list(parent.iterdir()))
    # This is the genuine existing actual elevated token observer, not a fake
    # image-pin callback. The Go test supplied its independently read actual SID.
    owner = coordinator.journey.admission.BeforeEnrollment({'request': {'clientSid': sid}})
    consumers = []
    try:
        def start(name):
            consumer = coordinator.NativeConsumption(executable, sha256, parent, evidence / name)
            consumers.append(consumer)
            consumer.pin(owner)
            return consumer

        eof = start('ready-eof.jsonl')
        coordinator.require(eof.process.poll() is None)
        eof.close()
        coordinator.require(eof.process.returncode == 1 and not list(parent.iterdir()))

        admitted = start('consumed.jsonl')
        # Actual native ancestry must deny replacement while READY, independently
        # of the Python pre/post path observations.
        displaced = parent.with_name(parent.name + '-must-not-move')
        try:
            parent.rename(displaced)
        except OSError:
            pass
        else:
            raise ValueError('Native READY parent was replaceable')
        coordinator.require(parent.is_dir() and not displaced.exists())
        admitted.consume(record)
        admitted.close()
        durable = parent / ('.installed-b0-consumed-' + record['challenge'] + '.json')
        coordinator.require(durable.read_bytes() == coordinator.binding._encode(record))
        original = durable.stat()

        replay = start('replay-refused.jsonl')
        try:
            replay.consume(record)
        except ValueError:
            pass
        else:
            raise ValueError('Native restarted consumer admitted replay')
        finally:
            replay.close()
        coordinator.require(replay.process.returncode == 1)
        current = durable.stat()
        coordinator.require((original.st_dev, original.st_ino) == (current.st_dev, current.st_ino) and
                            durable.read_bytes() == coordinator.binding._encode(record))
        return dict(schema=1, state='native-adapter-streaming-passed', readyEOFNoEffect=True,
                    protectedParentRetained=True, exactConsumedBytes=True,
                    restartedReplayRefused=True, productInstallAttempted=False)
    finally:
        try:
            for consumer in reversed(consumers):
                consumer.close()
        finally:
            owner.close()


def main(arguments):
    try:
        coordinator.require(len(arguments) == 5)
        result = run_fixture(*arguments)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (Exception, KeyboardInterrupt):
        print('Native adapter streaming refused; preserve fixture evidence.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
