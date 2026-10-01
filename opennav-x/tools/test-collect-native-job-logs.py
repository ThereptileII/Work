#!/usr/bin/env python3
"""Offline streaming, identity and credential refusal tests for native logs."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
from email.message import Message
from urllib.error import HTTPError


SPEC = importlib.util.spec_from_file_location("collector", Path(__file__).with_name("collect-native-job-logs.py"))
collector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(collector)


class Response:
    def __init__(self, body=b"", *, status=200, length=None, headers=None, partial_at=None):
        self.body = body
        self.offset = 0
        self.status = status
        self.partial_at = partial_at
        self.headers = Message()
        if length is not None:
            self.headers["Content-Length"] = str(length)
        for name, value in (headers or {}).items():
            self.headers[name] = value
        self.largest_read = 0

    def read(self, size=-1):
        assert 0 < size <= collector.CHUNK_BYTES
        self.largest_read = max(size, self.largest_read)
        stop = len(self.body) if self.partial_at is None else min(len(self.body), self.partial_at)
        data = self.body[self.offset:min(stop, self.offset + size)]
        self.offset += len(data)
        return data

    def getcode(self):
        return self.status

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class FakeOpener:
    def __init__(self, *, run_id=17, job_id=18, head=None, job_head=None,
                 job_run=None, job_status="completed", body=b"first\nlast\n",
                 log_length=None, log_status=200, redirect_host="productionresultssa0.blob.core.windows.net",
                 partial_at=None, log_headers=None, direct_log=False, omit_length=False,
                 repository=None, log_http_error=None):
        self.run_id = run_id
        self.job_id = job_id
        self.head = head or collector.HEAD_SHA
        self.job_head = job_head or collector.HEAD_SHA
        self.job_run = job_run if job_run is not None else run_id
        self.job_status = job_status
        self.body = body
        self.log_length = len(body) if log_length is None else log_length
        self.log_status = log_status
        self.redirect_host = redirect_host
        self.partial_at = partial_at
        self.log_headers = log_headers or {}
        self.direct_log = direct_log
        self.omit_length = omit_length
        self.repository = repository or collector.REPOSITORY
        self.log_http_error = log_http_error
        self.requests = []
        self.log_response = None
        self.signed_url = f"https://{redirect_host}/logs?sig=PRIVATE-SIGNED-TOKEN"

    def open(self, request, timeout=None):
        self.requests.append(request)
        url = request.full_url
        if url.endswith(f"/actions/runs/{self.run_id}"):
            value = {"id": self.run_id, "head_sha": self.head, "run_attempt": 1,
                     "repository": {"full_name": self.repository}}
            data = json.dumps(value).encode()
            return Response(data, length=len(data))
        if url.endswith(f"/actions/jobs/{self.job_id}"):
            value = {"id": self.job_id, "run_id": self.job_run, "head_sha": self.job_head,
                     "status": self.job_status, "completed_at": "2026-10-01T00:00:00Z"
                     if self.job_status == "completed" else None, "conclusion": "cancelled"}
            data = json.dumps(value).encode()
            return Response(data, length=len(data))
        if url.endswith(f"/actions/jobs/{self.job_id}/logs"):
            if self.log_http_error:
                raise HTTPError(url, self.log_http_error, "unavailable", Message(), None)
            if self.direct_log:
                return self._log()
            headers = Message()
            headers["Location"] = self.signed_url
            raise HTTPError(url, 302, "found", headers, None)
        if url == self.signed_url:
            return self._log()
        raise AssertionError("unexpected request route")

    def _log(self):
        self.log_response = Response(self.body, status=self.log_status,
                                     length=None if self.omit_length else self.log_length,
                                     partial_at=self.partial_at, headers=self.log_headers)
        return self.log_response


class CollectorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "logs"

    def collect(self, opener, *, token="token", jobs=((17, 18),)):
        return collector.collect(self.output, token, opener=opener, jobs=jobs)

    def test_streams_complete_log_with_digest_and_small_excerpts(self):
        body = b"start\n" + b"x" * (3 * collector.CHUNK_BYTES) + b"\nend\n"
        opener = FakeOpener(body=body)
        result = self.collect(opener)
        self.assertEqual(result["collected"], 1)
        self.assertEqual(result["bytesCollected"], len(body))
        log = result["jobs"][0]["log"]
        self.assertEqual(log["sha256"], hashlib.sha256(body).hexdigest())
        self.assertEqual((self.output / "job-18/job.log").stat().st_size, len(body))
        self.assertLessEqual(opener.log_response.largest_read, collector.CHUNK_BYTES)
        self.assertLessEqual(len(log["head"].encode()), collector.EXCERPT_BYTES)
        self.assertLessEqual(len(log["tail"].encode()), collector.EXCERPT_BYTES)
        self.assertFalse((self.output / "job-18/job.part").exists())

    def test_identity_and_live_job_refuse_before_log_download(self):
        for opener in (FakeOpener(head="0" * 40), FakeOpener(job_head="0" * 40),
                       FakeOpener(job_run=99), FakeOpener(job_status="in_progress"),
                       FakeOpener(repository="attacker/other")):
            with self.subTest(opener=opener.__dict__):
                result = self.collect(opener)
                self.assertEqual(result["collected"], 0)
                self.assertFalse((self.output / "job-18/job.log").exists())

    def test_signed_redirect_has_no_token_and_bad_host_refuses(self):
        opener = FakeOpener()
        self.collect(opener, token="PRIVATE-BEARER")
        signed = [r for r in opener.requests if r.full_url == opener.signed_url]
        self.assertEqual(len(signed), 1)
        self.assertIsNone(signed[0].get_header("Authorization"))
        self.assertTrue(all(r.get_header("Authorization") == "Bearer PRIVATE-BEARER"
                            for r in opener.requests if r.full_url.startswith(collector.API)))
        evidence = "".join(p.read_text() for p in self.output.rglob("*.json"))
        self.assertNotIn("PRIVATE-BEARER", evidence)
        self.assertNotIn("PRIVATE-SIGNED-TOKEN", evidence)
        bad = FakeOpener(redirect_host="attacker.invalid")
        result = collector.collect(self.output / "bad", "token", opener=bad, jobs=((17, 18),))
        self.assertEqual(result["collected"], 0)
        self.assertIn("redirect host", result["jobs"][0]["error"])

    def test_partial_missing_length_and_oversize_refuse_without_raw_file(self):
        cases = (
            FakeOpener(body=b"short", log_length=10),
            FakeOpener(body=b"partial", log_status=206),
            FakeOpener(body=b"data", omit_length=True),
            FakeOpener(body=b"data", log_headers={"Content-Range": "bytes 0-3/8"}),
            FakeOpener(body=b"x" * 20),
        )
        old_limit = collector.MAX_TOTAL_LOG_BYTES
        try:
            collector.MAX_TOTAL_LOG_BYTES = 10
            for index, opener in enumerate(cases):
                with self.subTest(index=index):
                    output = self.output / str(index)
                    result = collector.collect(output, "token", opener=opener, jobs=((17, 18),))
                    self.assertEqual(result["collected"], 0)
                    self.assertFalse((output / "job-18/job.log").exists())
                    self.assertFalse((output / "job-18/job.part").exists())
        finally:
            collector.MAX_TOTAL_LOG_BYTES = old_limit

    def test_secret_or_signed_url_in_log_refuses_without_artifact(self):
        for index, body in enumerate((b"prefix token suffix", b"https://example.test/log?sig=PRIVATE")):
            with self.subTest(index=index):
                output = self.output / str(index)
                result = collector.collect(output, "token", opener=FakeOpener(body=body), jobs=((17, 18),))
                self.assertEqual(result["collected"], 0)
                self.assertFalse((output / "job-18/job.log").exists())
                self.assertNotIn("PRIVATE", (output / "summary.json").read_text())

    def test_secret_and_signed_query_split_across_tiny_reads_are_refused(self):
        class TinyReadOpener(FakeOpener):
            def _log(self):
                response = super()._log()
                original_read = response.read
                response.read = lambda size: original_read(min(size, 1))
                return response
        for index, (body, token) in enumerate((
            (b"before LONG-PRIVATE-TOKEN after", "LONG-PRIVATE-TOKEN"),
            (b"https://example.test/object?sig=PRIVATE", "unrelated-token"),
        )):
            with self.subTest(index=index):
                output = self.output / str(index)
                result = collector.collect(output, token, opener=TinyReadOpener(body=body),
                                           jobs=((17, 18),))
                self.assertEqual(result["collected"], 0)
                self.assertFalse((output / "job-18/job.log").exists())
                self.assertFalse((output / "job-18/job.part").exists())

    def test_token_longer_than_detection_window_is_refused_before_network(self):
        opener = FakeOpener()
        with self.assertRaisesRegex(collector.CollectionError, "invalid"):
            self.collect(opener, token="x" * (collector.EXCERPT_BYTES + 1))
        self.assertEqual(opener.requests, [])

    def test_completed_job_with_unavailable_log_is_not_collected(self):
        result = self.collect(FakeOpener(log_http_error=404))
        self.assertEqual(result["collected"], 0)
        self.assertIn("unavailable", result["jobs"][0]["error"])

    def test_overall_deadline_refuses_without_request(self):
        opener = FakeOpener()
        result = collector.collect(self.output, "token", opener=opener,
                                   jobs=((17, 18),), deadline_seconds=0)
        self.assertEqual(result["collected"], 0)
        self.assertEqual(opener.requests, [])

    def test_slow_body_read_hits_hard_deadline_and_removes_partial_file(self):
        class SlowOpener(FakeOpener):
            def _log(self):
                response = super()._log()
                original_read = response.read
                def slow_read(size):
                    time.sleep(0.2)
                    return original_read(size)
                response.read = slow_read
                return response
        started = time.monotonic()
        result = collector.collect(self.output, "token", opener=SlowOpener(),
                                   jobs=((17, 18),), deadline_seconds=0.05)
        self.assertEqual(result["collected"], 0)
        self.assertLess(time.monotonic() - started, 0.15)
        self.assertFalse((self.output / "job-18/job.part").exists())
        self.assertFalse((self.output / "job-18/job.log").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
