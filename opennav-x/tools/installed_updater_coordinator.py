"""SCRUM-311 live composition: independent approval, native consume, one journey.

Caller supplies authenticated original pending context and operator handoff. This
module never produces approval, discovers a permissive producer, publishes,
changes connectivity, retries or clears consumed state. Native consumption is a
required executable dependency, not an injected authority grant.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import subprocess
import os
import time
from pathlib import Path

import installed_updater_approval as binding
import installed_updater_approval_http as http
import installed_updater_approval_transport as transport
import installed_updater_approval_rendezvous as rendezvous
import installed_updater_journey as journey

FAILURE = 'Installed updater coordinator refused; preserve original state and evidence.'


def require(value):
    if not value:
        raise ValueError(FAILURE)


def consumption_record(approval):
    require(type(approval) is binding.BoundApproval)
    r = approval.request.runner
    return dict(schema=1, purpose='installed-updater-b0-consumption',
                repository=r.repository, headSha=r.head_sha, runId=r.run_id,
                runAttempt=r.run_attempt, jobId=r.job_id, jobName=r.job_name,
                challenge=approval.request.challenge,
                requestSha256=approval.request.request_sha256,
                approvalSha256=approval.approval_sha256)


def _rebind(approval, arguments, now):
    return binding.bind_approval(**arguments, approval_bytes=approval.original_approval_bytes,
        approved_request_sha256=approval.request.request_sha256,
        approved_approval_sha256=approval.approval_sha256,
        approved_pins=dict(approval.request.pins), now=now)


def _originals(approval, initial, phases):
    journey._same_inputs(initial, phases)
    journey.admission.unchanged(initial)
    journey.phase.unchanged(phases)
    for path, expected in ((initial['retained'][0][0], approval.original_declaration_bytes),
                           (initial['requestPath'], approval.original_client_request_bytes),
                           (initial['b0']['paths']['policy'], approval.original_release_policy_bytes)):
        with path.open('rb') as stream:
            require(stream.read(len(expected) + 1) == expected)
    journey.admission.unchanged(initial)
    journey.phase.unchanged(phases)
    require(str(initial['setup']) == approval.setup.path and
            initial['setupSha256'] == approval.setup.sha256 and
            initial['setup'].stat().st_size == approval.setup.size)
    pins = dict(approval.request.pins)
    require(initial['retained'][0][2] == pins['declarationSha256'] and
            initial['brokerPolicy']['provisioner']['sha256'] == pins['provisionerSha256'] and
            initial['brokerPolicy']['broker']['sha256'] == pins['brokerSha256'])


def _identity(identity):
    binding._keys(identity, ('volumeSerial', 'fileID', 'creationTime', 'descriptorSha256'))
    for number in ('volumeSerial', 'creationTime'):
        binding._decimal(identity[number])
        require(int(identity[number]) <= (1 << 64) - 1)
    binding._hex(identity['fileID'], 32)
    require(identity['fileID'] != '0' * 32)
    binding._hex(identity['descriptorSha256'])


def _original_reader(stream):
    # ReOpenFile takes the retained original HANDLE, never its pathname, and
    # gives this observer its own cursor while native stdout continues writing.
    require(os.name == 'nt')
    import ctypes
    import msvcrt
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    reopen = kernel.ReOpenFile
    reopen.argtypes = (wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD, wintypes.DWORD)
    reopen.restype = wintypes.HANDLE
    handle = reopen(msvcrt.get_osfhandle(stream.fileno()), 0x80000000, 3, 0)
    require(handle not in (None, ctypes.c_void_p(-1).value))
    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    except BaseException:
        kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
        kernel.CloseHandle.restype = wintypes.BOOL
        require(kernel.CloseHandle(handle))
        raise
    try:
        return os.fdopen(descriptor, 'rb', buffering=0)
    except BaseException:
        os.close(descriptor)
        raise


class NativeConsumption:
    """Two frames from one original harness child retaining the native parent."""
    def __init__(self, executable, executable_sha256, parent, receipt):
        self.executable, self.parent, self.receipt = map(Path, (executable, parent, receipt))
        binding._hex(executable_sha256)
        self.sha256 = executable_sha256
        require(all(p.is_absolute() for p in (self.executable, self.parent, self.receipt)))
        require(self.receipt != self.parent and self.parent not in self.receipt.parents)
        self.attempted, self.pinned = False, False
        self.process = self.output = self.reader = None
        self.closed = False

    def _frame(self):
        deadline, raw = time.monotonic() + 60, bytearray()
        while True:
            require(os.fstat(self.reader.fileno()).st_size <= 8192)
            part = self.reader.read(1)
            if part:
                raw.extend(part)
                require(len(raw) <= 4096)
                if part == b'\n':
                    return binding._json(bytes(raw), 4096)
            else:
                require(self.process.poll() is None and time.monotonic() < deadline)
                time.sleep(0.05)

    def pin(self, owner):
        require(not self.pinned and not self.attempted and not self.closed)
        journey.admission.updater._plain(self.parent)
        require(self.parent.is_dir())
        self.parent_identity = journey.phase.stamp(self.parent)[:2]
        owner.pin_image(self.executable, self.sha256)
        journey.admission.updater._plain(self.receipt.parent)
        require(not self.receipt.is_symlink())
        self.output = self.receipt.open('x+b', buffering=0)
        self.reader = _original_reader(self.output)
        left, right = os.fstat(self.reader.fileno()), os.fstat(self.output.fileno())
        require((left.st_dev, left.st_ino) == (right.st_dev, right.st_ino))
        self.process = subprocess.Popen([str(self.executable),
            '--consume-installed-b0-challenge=' + str(self.parent)],
            stdin=subprocess.PIPE, stdout=self.output, stderr=subprocess.DEVNULL)
        ready = self._frame()
        binding._keys(ready, ('schema', 'state', 'parent'))
        require(type(ready['schema']) is int and ready['schema'] == 1 and ready['state'] == 'ready')
        _identity(ready['parent'])
        self.parent_native = ready['parent']
        require(not self.reader.read(1))
        self.check()
        self.pinned = True

    def check(self):
        require(not self.closed and self.process is not None and self.process.poll() is None)
        journey.admission.updater._plain(self.parent)
        require(journey.phase.stamp(self.parent)[:2] == self.parent_identity)

    def consume(self, record):
        require(self.pinned and not self.attempted)
        self.attempted = True
        self.check()
        raw = binding._encode(record)
        require(0 < len(raw) <= 4096)
        self.process.stdin.write(raw)
        self.process.stdin.close()
        # The native helper retains its original deny-delete ancestry from READY
        # through the effect. A deadline is failure even if late exit succeeds.
        require(self.process.wait(timeout=60) == 0)
        receipt = self._frame()
        binding._keys(receipt, ('schema', 'state', 'recordSha256', 'parent', 'record'))
        require(type(receipt['schema']) is int and receipt['schema'] == 1 and
                receipt['state'] == 'consumed' and
                receipt['recordSha256'] == hashlib.sha256(raw).hexdigest())
        _identity(receipt['parent'])
        _identity(receipt['record'])
        require(receipt['parent'] == self.parent_native and
                receipt['parent']['volumeSerial'] == receipt['record']['volumeSerial'] and
                receipt['parent']['fileID'] != receipt['record']['fileID'] and not self.reader.read(1))
        journey.admission.updater._plain(self.parent)
        require(journey.phase.stamp(self.parent)[:2] == self.parent_identity)

    def close(self):
        if self.closed:
            return
        self.closed = True
        pending = None
        try:
            if self.process is not None:
                try:
                    if not self.process.stdin.closed:
                        self.process.stdin.close()  # EOF before record grants nothing.
                except BaseException as error:
                    pending = error
                # A failed stdin close or repeated Ctrl-C cannot skip the
                # original child drain and release its image/parent custody.
                while True:
                    try:
                        self.process.wait()
                        break
                    except BaseException as error:
                        pending = error
                        # An unexpected wait error is not an exit receipt.
                        # Keep all original owners until wait/poll actually
                        # corroborates exit; preserve the original failure.
                        try:
                            if self.process.poll() is not None:
                                break
                        except BaseException:
                            pass
                        time.sleep(0.05)
        finally:
            try:
                if self.reader is not None:
                    self.reader.close()
            finally:
                if self.output is not None:
                    self.output.close()
        if pending is not None:
            raise pending


class Coordinator:
    """One object, one attempt, including failed waits and ambiguous native effects.

    Instantiation does not authenticate caller inputs. receive_approval and the
    immediate native/original checks below establish the required bindings.
    """
    def __init__(self, *, source, bind_arguments, initial, phases, output, handoff,
                 consumer, reads, cancelled=lambda: False,
                 utcnow=lambda: datetime.now(timezone.utc)):
        require(type(source) in (transport.ApprovalSource, rendezvous.TrustedStatusLocator) and callable(handoff) and
                type(consumer) is NativeConsumption and type(reads) is http.GitHubReads)
        self.source, self.arguments = source, deepcopy(bind_arguments)
        self.initial, self.phases = deepcopy(initial), deepcopy(phases)
        self.output, self.handoff = output, handoff
        self.consumer, self.reads = consumer, reads
        self.cancelled, self.utcnow = cancelled, utcnow
        self.attempted = False

    def run(self):
        require(not self.attempted)
        self.attempted = True
        owner = None
        deadline = time.monotonic() + 3600
        request = binding._request(self.arguments['request_bytes'])
        def remaining():
            value = min(deadline - time.monotonic(),
                        (request.expires_at - binding._clock(self.utcnow())).total_seconds())
            require(not self.cancelled() and request.created_at <= self.utcnow() and value > 0)
            return value
        resolved = None
        try:
            require(not self.cancelled())
            # Keep the actual account and original deny-write executable owner
            # through approval wait, consumption and the complete installed run.
            owner = journey.admission.BeforeEnrollment(self.initial)
            require(owner.sid == self.initial['request']['clientSid'])
            profile = owner.profile()
            self.consumer.pin(owner)
            source = self.source
            if type(source) is rendezvous.TrustedStatusLocator:
                resolved = rendezvous.resolve_source(locator=source, request=request,
                    read_api=self.reads.read_api, timeout_seconds=remaining(),
                    cancelled=self.cancelled, utcnow=self.utcnow, check=self.consumer.check)
                source = resolved.source
            approval = transport.receive_approval(source=source, timeout_seconds=remaining(),
                read_api=self.reads.read_api, download_archive=self.reads.download_archive,
                bind_arguments=self.arguments, cancelled=self.cancelled, utcnow=self.utcnow)
            self.consumer.check()
            _originals(approval, self.initial, self.phases)
            require(owner.actual_sid() == owner.sid and
                    journey.phase.profile_preserved(profile, owner.profile()) and not self.cancelled())
            approval = _rebind(approval, self.arguments, self.utcnow())
            if resolved is not None:
                rendezvous.resolve_source(locator=self.source, request=request,
                    read_api=self.reads.read_api, timeout_seconds=remaining(),
                    cancelled=self.cancelled, utcnow=self.utcnow, check=self.consumer.check,
                    expected=resolved)
            remaining()
            # A paginated status recheck may consume time from the shorter
            # independent approval lifetime as well as the request deadline.
            approval = _rebind(approval, self.arguments, self.utcnow())
            self.consumer.check()
            self.consumer.consume(consumption_record(approval))
            # Expiration/cancellation after consumption permanently refuses this
            # attempt. Never turn an uncertain native effect into another call.
            require(not self.cancelled())
            approval = _rebind(approval, self.arguments, self.utcnow())
            _originals(approval, self.initial, self.phases)
            require(owner.actual_sid() == owner.sid and
                    journey.phase.profile_preserved(profile, owner.profile()))
            pins = dict(approval.request.pins)
            return journey.run_joined(self.initial, self.phases,
                dict(request=pins['clientRequestSha256'], policy=pins['bootstrapPolicySha256'],
                     provisioner=pins['provisionerSha256'], broker=pins['brokerSha256'],
                     timestamp=pins['timestampSha256']), self.output, handoff=self.handoff)
        finally:
            try:
                self.consumer.close()
            finally:
                if owner is not None:
                    owner.close()


def run_live(*, source, bind_arguments, initial, phases, output, handoff,
             read_only_token, consumer_executable, consumer_sha256,
             qualification_parent, consumption_receipt, cancelled=lambda: False):
    """Execute one already-authorized live pending-job composition.

    source and bind_arguments must come from independently authenticated operator
    and pending-job provenance. No producer identity or approval pin is inferred
    from the applicant's files. Authentication of the original candidate/kit and
    preparation of the two maintained driver declarations precede this call.
    """
    return Coordinator(source=source, bind_arguments=bind_arguments,
        initial=initial, phases=phases, output=output, handoff=handoff,
        consumer=NativeConsumption(consumer_executable, consumer_sha256,
                                   qualification_parent, consumption_receipt),
        reads=http.GitHubReads(read_only_token), cancelled=cancelled).run()
