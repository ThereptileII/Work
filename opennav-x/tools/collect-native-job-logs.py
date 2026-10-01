#!/usr/bin/env python3
"""Stream exact frozen GitHub Actions job logs into bounded local evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener


REPOSITORY = "ThereptileII/Work"
HEAD_SHA = "b48bf4a8f12f98c459d79aa08805141ac49e0306"
JOBS = ((36901029053, 110499936978),)
API = "https://api.github.com"
MAX_API_BYTES = 2 * 1024 * 1024
MAX_TOTAL_LOG_BYTES = 2 * 1024 * 1024 * 1024
EXCERPT_BYTES = 16 * 1024
CHUNK_BYTES = 1024 * 1024
OVERALL_SECONDS = 8 * 60
REQUEST_SECONDS = 30
REDIRECT_LIMIT = 4
SENSITIVE = re.compile(rb"(?i)(?:[?&](?:sig|signature|x-amz-signature|token|access_token)=|authorization\s*:\s*bearer\s+)")
URL = re.compile(r"https://[^\s<>'\"]+")


class CollectionError(Exception):
    """A sanitized refusal safe to retain in the evidence summary."""


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _deadline_timeout(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise CollectionError("overall collection deadline exceeded")
    return min(REQUEST_SECONDS, remaining)


def _read_api(response) -> bytes:
    length = response.headers.get("Content-Length")
    if length is not None:
        if not length.isdecimal() or int(length) > MAX_API_BYTES:
            raise CollectionError("GitHub API response length is invalid or excessive")
    data = bytearray()
    read = getattr(response, "read1", response.read)
    while True:
        block = read(min(CHUNK_BYTES, MAX_API_BYTES + 1 - len(data)))
        if not block:
            break
        data.extend(block)
        if len(data) > MAX_API_BYTES:
            raise CollectionError("GitHub API response exceeds byte limit")
    if length is not None and len(data) != int(length):
        raise CollectionError("GitHub API response is incomplete")
    return bytes(data)


def _api_json(opener, route: str, token: str, deadline: float) -> dict:
    request = Request(API + route, headers={
        "Accept": "application/vnd.github+json",
        "Authorization": "Bearer " + token,
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "OpenNavX-native-log-evidence",
    })
    try:
        with opener.open(request, timeout=_deadline_timeout(deadline)) as response:
            if response.getcode() != 200:
                raise CollectionError("GitHub API did not return a complete response")
            value = json.loads(_read_api(response))
    except HTTPError as exc:
        status = exc.code
        exc.close()
        raise CollectionError("GitHub API request failed (HTTP %d)" % status) from None
    except (URLError, TimeoutError, OSError):
        raise CollectionError("GitHub API request failed (network error)") from None
    except (ValueError, UnicodeError, RecursionError):
        raise CollectionError("GitHub API returned invalid JSON") from None
    if not isinstance(value, dict):
        raise CollectionError("GitHub API returned an unexpected JSON shape")
    return value


def _allowed_download_url(value: str) -> bool:
    try:
        parsed = urlsplit(value)
        host = parsed.hostname
        port = parsed.port
    except ValueError:
        return False
    if parsed.scheme != "https" or parsed.username or parsed.password or port not in (None, 443) or not host:
        return False
    host = host.lower().rstrip(".")
    return (host == "github.com" or host.endswith(".github.com") or
            host.endswith(".githubusercontent.com") or
            re.fullmatch(r"productionresultssa[0-9]+\.blob\.core\.windows\.net", host) is not None)


def _open_log(opener, job_id: int, token: str, deadline: float):
    route = f"{API}/repos/{REPOSITORY}/actions/jobs/{job_id}/logs"
    request = Request(route, headers={
        "Accept": "application/vnd.github+json",
        "Authorization": "Bearer " + token,
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "OpenNavX-native-log-evidence",
    })
    try:
        response = opener.open(request, timeout=_deadline_timeout(deadline))
    except HTTPError as exc:
        if exc.code not in (301, 302, 303, 307, 308):
            code = exc.code
            exc.close()
            raise CollectionError("job log unavailable (HTTP %d)" % code) from None
        response = exc
    except (URLError, TimeoutError, OSError):
        raise CollectionError("job log unavailable (network error)") from None
    for _ in range(REDIRECT_LIMIT + 1):
        status = response.getcode()
        if status == 200:
            return response
        if status == 206:
            response.close()
            raise CollectionError("job log server returned a partial response")
        if status not in (301, 302, 303, 307, 308):
            response.close()
            raise CollectionError("job log server returned an unsupported response")
        location = response.headers.get("Location")
        response.close()
        if not location or not _allowed_download_url(location):
            raise CollectionError("job log redirect host or scheme is not allowed")
        # A fresh request uses the signed URL only as its target; API credentials
        # are never forwarded to the download host.
        request = Request(location, headers={
            "Accept-Encoding": "identity", "User-Agent": "OpenNavX-native-log-evidence"})
        try:
            response = opener.open(request, timeout=_deadline_timeout(deadline))
        except HTTPError as exc:
            if exc.code in (301, 302, 303, 307, 308):
                response = exc
            else:
                code = exc.code
                exc.close()
                raise CollectionError("job log download failed (HTTP %d)" % code) from None
        except (URLError, TimeoutError, OSError):
            raise CollectionError("job log download failed (network error)") from None
    response.close()
    raise CollectionError("job log redirect limit exceeded")


def _excerpt(data: bytes, token: str) -> str:
    value = data.decode("utf-8", errors="replace").replace(token, "[redacted]")
    return URL.sub(lambda match: match.group(0).split("?", 1)[0] + "?[redacted]"
                   if "?" in match.group(0) else match.group(0), value)


def _stream_log(response, path: Path, token: str, remaining_bytes: int,
                deadline: float) -> dict:
    length = response.headers.get("Content-Length")
    if (response.headers.get("Content-Range") is not None or
            response.headers.get("Content-Encoding", "identity").lower() != "identity" or
            length is None or not length.isdecimal() or int(length) <= 0):
        raise CollectionError("job log has no verifiable complete length")
    expected = int(length)
    if expected > remaining_bytes:
        raise CollectionError("job log exceeds the remaining 2 GiB collection budget")
    digest = hashlib.sha256()
    count = 0
    head = b""
    tail = b""
    overlap = b""
    temporary = path.with_suffix(".part")
    try:
        with temporary.open("xb") as output:
            read = getattr(response, "read1", response.read)
            while True:
                _deadline_timeout(deadline)
                block = read(min(CHUNK_BYTES, expected + 1 - count))
                if not block:
                    break
                count += len(block)
                if count > expected or count > remaining_bytes:
                    raise CollectionError("job log exceeds its declared or total byte limit")
                window = overlap + block
                if token.encode() in window or SENSITIVE.search(window):
                    raise CollectionError("job log contains sensitive credential material")
                overlap = window[-EXCERPT_BYTES:]
                digest.update(block)
                output.write(block)
                if len(head) < EXCERPT_BYTES:
                    head += block[:EXCERPT_BYTES - len(head)]
                tail = (tail + block)[-EXCERPT_BYTES:]
        if count != expected:
            raise CollectionError("job log ended before its declared length")
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return {"path": path.name, "bytes": count, "sha256": digest.hexdigest(),
            "head": _excerpt(head, token), "tail": _excerpt(tail, token)}


def collect(output: Path, token: str, *, opener=None, jobs=JOBS,
            deadline_seconds=OVERALL_SECONDS) -> dict:
    if not token or "\r" in token or "\n" in token or len(token.encode()) > EXCERPT_BYTES:
        raise CollectionError("GITHUB_TOKEN is missing or invalid")
    if opener is None:
        opener = build_opener(NoRedirect())
    deadline = time.monotonic() + deadline_seconds
    if not hasattr(signal, "setitimer"):
        raise CollectionError("hard collection deadline requires a POSIX host")
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    results = []
    used = 0
    prior_handler = signal.getsignal(signal.SIGALRM)
    def expired(_signum, _frame):
        raise CollectionError("overall collection deadline exceeded")
    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, deadline_seconds if deadline_seconds > 0 else 0)
    try:
        for run_id, job_id in jobs:
            record = {"runId": run_id, "jobId": job_id, "headSha": HEAD_SHA, "status": "failed"}
            try:
                if time.monotonic() >= deadline:
                    raise CollectionError("overall collection deadline exceeded")
                run = _api_json(opener, f"/repos/{REPOSITORY}/actions/runs/{run_id}", token, deadline)
                repository = run.get("repository")
                if (run.get("id") != run_id or run.get("head_sha") != HEAD_SHA or
                        not isinstance(repository, dict) or repository.get("full_name") != REPOSITORY):
                    raise CollectionError("workflow run identity mismatch")
                job = _api_json(opener, f"/repos/{REPOSITORY}/actions/jobs/{job_id}", token, deadline)
                if (job.get("id") != job_id or job.get("run_id") != run_id or
                        job.get("head_sha") != HEAD_SHA):
                    raise CollectionError("job run or commit identity mismatch")
                if job.get("status") != "completed" or not job.get("completed_at"):
                    raise CollectionError("job is not complete; terminal logs unavailable")
                if (type(run.get("run_attempt")) is not int or run["run_attempt"] < 1 or
                        not isinstance(job.get("conclusion"), str) or
                        job["conclusion"] not in {
                            "success", "failure", "cancelled", "skipped", "timed_out",
                            "action_required", "neutral", "stale"} or
                        not isinstance(job["completed_at"], str) or
                        re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z", job["completed_at"]) is None):
                    raise CollectionError("job completion metadata is invalid")
                directory = output / f"job-{job_id}"
                directory.mkdir(exist_ok=True)
                path = directory / "job.log"
                if path.exists() or path.with_suffix(".part").exists():
                    raise CollectionError("job output already exists; refusing overwrite")
                with _open_log(opener, job_id, token, deadline) as response:
                    log = _stream_log(response, path, token, MAX_TOTAL_LOG_BYTES - used, deadline)
                used += log["bytes"]
                record.update({"status": "collected", "runAttempt": run.get("run_attempt"),
                               "jobConclusion": job.get("conclusion"),
                               "jobCompletedAt": job.get("completed_at"), "log": log})
            except CollectionError as exc:
                record["error"] = str(exc)
            except Exception:
                record["error"] = "collection failed (invalid or unavailable evidence)"
            (output / f"job-{job_id}.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                                                    encoding="utf-8")
            results.append(record)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, prior_handler)
    summary = {"schemaVersion": 1, "repository": REPOSITORY, "headSha": HEAD_SHA,
               "jobs": results, "collected": sum(row["status"] == "collected" for row in results),
               "bytesCollected": used, "maxBytes": MAX_TOTAL_LOG_BYTES}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                         encoding="utf-8")
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        summary = collect(args.output, os.environ.get("GITHUB_TOKEN", ""))
    except CollectionError as exc:
        print("Native log collection unavailable: " + str(exc), file=sys.stderr)
        return 1
    for item in summary["jobs"]:
        print("run=%d job=%d %s" % (item["runId"], item["jobId"], item["status"]))
    return 0 if summary["collected"] == len(JOBS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
