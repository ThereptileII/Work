"""SCRUM-311 offline content inspection only; never a launch/approval token.

Paths are supplied explicitly by a trusted caller; paths inside evidence are
compared but never followed. A success does not authenticate that caller, prove
live commissioning, or retain custody beyond this inspection.
"""
from contextlib import ExitStack
from datetime import datetime
import hashlib
import json
import ntpath
import os
from pathlib import Path
import re
import stat

JSON_LIMIT = 4 * 1024 * 1024
PROFILE_LIMIT = 4 * 1024 * 1024
EXE_LIMIT = 128 * 1024 * 1024
MAX_DEPTH = 24
MAX_NODES = 100000
OBJECT_HASH = "sha256-json-sortkeys-utf8-v1"
RECORDS = ("prepared", "applied", "inventory", "plan", "target", "state", "ownership", "active")
RAW = ("baseline", "inputProfile", "profile", "executable")
OWNER = "OpenNavX.ReadOnlyCommissioning.1"


class Refused(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise Refused(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def object_digest(value):
    """Separate content digest; NOT the PowerShell receipt serialization hash."""
    return digest(json.dumps(value, sort_keys=True, ensure_ascii=False,
                             separators=(",", ":"), allow_nan=False).encode("utf-8"))


def closed(value, fields):
    require(type(value) is dict and set(value) == set(fields), "Closed input schema differs")


def hex_id(value, length=64):
    require(type(value) is str and re.fullmatch("[0-9a-f]{%d}" % length, value)
            and set(value) != {"0"}, "Invalid exact identity")


def strict_json(data):
    require(0 < len(data) <= JSON_LIMIT, "JSON byte bound")

    def pairs(items):
        result, seen = {}, set()
        for key, value in items:
            require(key and key.casefold() not in seen, "Duplicate/ambiguous JSON key")
            seen.add(key.casefold())
            result[key] = value
        return result

    def no_number(_):
        raise Refused("Only finite integer JSON numbers are supported")

    try:
        value = json.loads(data.decode("utf-8"), object_pairs_hook=pairs,
                           parse_float=no_number, parse_constant=no_number)
        pending, nodes = [(value, 0)], 0
        while pending:
            item, depth = pending.pop()
            nodes += 1
            require(depth <= MAX_DEPTH and nodes <= MAX_NODES, "JSON complexity bound")
            if isinstance(item, (dict, list)):
                pending.extend((v, depth + 1) for v in (item.values() if isinstance(item, dict) else item))
            if isinstance(item, str):
                item.encode("utf-8", errors="strict")
            if type(item) is int:
                require(-(2**63) <= item < 2**63, "JSON integer bound")
        require(type(value) is dict, "JSON object required")
        return value
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise Refused("Invalid bounded UTF-8 JSON") from error


def source_path(value):
    require(type(value) is str and len(value) <= 240 and
            re.match(r"^[A-Za-z]:\\", value) and "/" not in value and
            ntpath.normpath(value) == value, "Canonical local Windows source path required")
    for part in value[3:].split("\\"):
        require(part and not part.endswith((".", " ")) and
                not re.search(r'[\x00-\x1f:<>"|?*]', part) and
                not re.fullmatch(r"(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?", part),
                "Ambiguous Windows source path")
    return value


def identity(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_nlink,
            info.st_size, info.st_mtime_ns, info.st_ctime_ns)


class Snapshot:
    """Hold and recheck a regular-file snapshot, without any mutation API."""
    def __init__(self, descriptor, limit, stack):
        closed(descriptor, ("path", "sourcePath", "sha256"))
        source_path(descriptor["sourcePath"])
        hex_id(descriptor["sha256"])
        name = descriptor["path"]
        require(type(name) is str and len(name) <= 4096 and os.path.isabs(name) and
                os.path.normpath(name) == name, "Canonical absolute snapshot path required")
        self.path, self.limit = Path(name), limit
        self.ancestors = self.check_ancestors()
        before = self.path.lstat()
        require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1 and
                not getattr(before, "st_file_attributes", 0) & 0x400 and
                0 < before.st_size <= limit, "Artifact type/link/size refused")
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0) | getattr(os, "O_BINARY", 0)
        fd = os.open(self.path, flags)
        stack.callback(os.close, fd)
        self.fd, self.initial = fd, identity(before)
        require(identity(os.fstat(fd)) == self.initial, "Artifact replaced during open")
        self.sha256 = descriptor["sha256"]
        self.data = self.read()
        self.recheck()

    def check_ancestors(self):
        result = []
        for path in reversed(self.path.parents):
            info = path.lstat()
            require(stat.S_ISDIR(info.st_mode) and not stat.S_ISLNK(info.st_mode) and
                    not getattr(info, "st_file_attributes", 0) & 0x400, "Linked ancestor refused")
            result.append((str(path), info.st_dev, info.st_ino))
        return result

    def read(self):
        os.lseek(self.fd, 0, os.SEEK_SET)
        chunks, size = [], 0
        while True:
            data = os.read(self.fd, min(1024 * 1024, self.limit + 1 - size))
            if not data:
                break
            size += len(data)
            require(size <= self.limit, "Artifact grew beyond bound")
            chunks.append(data)
        result = b"".join(chunks)
        require(digest(result) == self.sha256, "Artifact bytes differ from expected digest")
        return result

    def recheck(self):
        require(self.check_ancestors() == self.ancestors and
                identity(self.path.lstat()) == self.initial and
                identity(os.fstat(self.fd)) == self.initial, "Artifact replaced or changed during inspection")
        self.read()
        require(identity(self.path.lstat()) == self.initial and
                identity(os.fstat(self.fd)) == self.initial and self.check_ancestors() == self.ancestors,
                "Artifact changed during final read")


def utc_seconds(text):
    require(type(text) is str and re.fullmatch(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(?:\.\d{1,7})?(?:Z|\+00:00)", text),
            "Explicit UTC review time required")
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00")).timestamp()
    except ValueError as error:
        raise Refused("Invalid UTC review time") from error


def _validate(expectation_json, now):
    """Inspect explicit copies of existing artifacts; returns no launch authority."""
    e = strict_json(expectation_json)
    closed(e, ("schema", "session", "identity", "createdUnix", "expiresUnix", "reviewedUnix",
               "objectHashAlgorithm", "contextContentSha256", "treesContentSha256",
               "environmentContentSha256", "artifacts"))
    require(type(e["schema"]) is int and e["schema"] == 1 and e["objectHashAlgorithm"] == OBJECT_HASH,
            "Unknown artifact expectation format")
    hex_id(e["session"])
    closed(e["identity"], ("generation", "commit", "packageSha256", "executableSha256"))
    for key, size in (("generation", 32), ("commit", 40), ("packageSha256", 64), ("executableSha256", 64)):
        hex_id(e["identity"][key], size)
    for key in ("contextContentSha256", "treesContentSha256", "environmentContentSha256"):
        hex_id(e[key])
    require(all(type(v) is int and v >= 0 for v in (now, e["createdUnix"], e["expiresUnix"], e["reviewedUnix"])),
            "Integer session time required")
    require(e["createdUnix"] <= e["reviewedUnix"] <= now < e["expiresUnix"] and
            0 < e["expiresUnix"] - e["createdUnix"] <= 14400, "Stale or invalid session review")
    closed(e["artifacts"], RECORDS + RAW)
    paths = e["artifacts"]
    with ExitStack() as stack:
        snaps = {key: Snapshot(paths[key], EXE_LIMIT if key == "executable" else JSON_LIMIT, stack)
                 for key in RECORDS + RAW}
        require(len({(s.initial[0], s.initial[1]) for s in snaps.values()}) == len(snaps), "Aliased evidence files")
        require(len({paths[k]["sourcePath"].casefold() for k in paths}) == len(paths), "Aliased source paths")
        d = {key: strict_json(snaps[key].data) for key in RECORDS}
        p, a, inv, plan, target, state, own, active = (d[k] for k in RECORDS)
        for obj, owner in ((p, OWNER), (a, OWNER), (inv, OWNER.replace(".1", ".Inventory.1")),
                           (plan, OWNER.replace(".1", ".Plan.1")), (target, "OpenNavX.BoatTarget.1"),
                           (active, OWNER), (state, "OpenNavX.Alpha1.SideBySide.1")):
            require(type(obj.get("schema")) is int and obj["schema"] == 1 and obj.get("owner") == owner,
                    "Artifact schema/owner differs")
        require(own["owner"] == state["owner"], "Ownership owner differs")
        audit, context, candidate = target["readOnlyAudit"], p["context"], e["identity"]
        installation, environment = context["installation"], context["launchEnvironment"]
        require(state["current"] == candidate["generation"] and own["commit"] == candidate["commit"] and
                own["packageSha256"] == candidate["packageSha256"] and
                snaps["executable"].sha256 == candidate["executableSha256"], "Candidate identity differs")
        owned_executable = [item for item in own["managedFiles"] if item["path"] == "app/opencpn.exe"]
        require(len(owned_executable) == 1 and owned_executable[0]["sha256"] == candidate["executableSha256"],
                "Executable absent or changed in ownership content")
        require(installation["commit"] == candidate["commit"] and
                installation["executableSha256"] == candidate["executableSha256"] and
                installation["stateSha256"] == snaps["state"].sha256 and
                installation["ownershipSha256"] == snaps["ownership"].sha256 and
                ntpath.basename(source_path(installation["generation"])) == candidate["generation"] and
                installation["executable"] == paths["executable"]["sourcePath"], "Commissioning candidate differs")
        require(installation["executable"] == ntpath.join(installation["generation"], "app", "opencpn.exe"),
                "Executable outside selected generation")
        require(installation["generation"] == ntpath.join(installation["root"], "generations", candidate["generation"]),
                "Generation outside selected installation")
        require(paths["state"]["sourcePath"] == ntpath.join(source_path(installation["root"]), "state.json") and
                paths["ownership"]["sourcePath"] == ntpath.join(installation["generation"], "ownership.json"),
                "Selected generation source locations differ")
        transaction = ntpath.dirname(paths["prepared"]["sourcePath"])
        require(ntpath.basename(paths["prepared"]["sourcePath"]) == "prepared.json" and
                ntpath.dirname(transaction) == ntpath.join(source_path(context["workspace"]), "runs"), "Transaction location differs")
        for key, name in (("applied", "applied.json"), ("inventory", "inventory.json"), ("plan", "review-plan.json"),
                          ("baseline", "baseline.ini"), ("inputProfile", "input-only.ini")):
            require(paths[key]["sourcePath"] == ntpath.join(transaction, name), "Transaction artifact location differs")
        require(paths["target"]["sourcePath"] == ntpath.join(context["workspace"], "boat-target.json") and
                paths["active"]["sourcePath"] == ntpath.join(context["workspace"], "commissioning-active.json") and
                target["profileDirectory"] == context["profile"] and
                target["stockExecutable"] == context["executable"] == state["stock"]["path"] and
                paths["profile"]["sourcePath"] == ntpath.join(source_path(context["profile"]), "opencpn.ini"),
                "Profile/target source identity differs")
        require(p["status"] == "prepared" and a["status"] == "input-only-prepared" and
                a["applicationLaunched"] is False and a["launchAuditStillRequired"] is True and
                p["applicationLaunched"] is False, "Incomplete commissioning artifacts")
        require(p["planSha256"] == snaps["plan"].sha256 and
                p["inventorySha256"] == plan["inventorySha256"] == snaps["inventory"].sha256 and
                a["recordSha256"] == active["recordSha256"] == audit["commissioning"]["recordSha256"] == snaps["prepared"].sha256 and
                active["record"] == audit["commissioning"]["record"] == paths["prepared"]["sourcePath"] and
                audit["commissioning"]["appliedSha256"] == snaps["applied"].sha256, "Commissioning artifact chain differs")
        require(p["baselineSha256"] == inv["profileSha256"] == snaps["baseline"].sha256 and
                p["inputSha256"] == a["profileSha256"] == audit["profileIniSha256"] ==
                snaps["inputProfile"].sha256 == snaps["profile"].sha256, "Profile content chain differs")
        require(audit["buildCommit"] == candidate["commit"] and
                all(audit[k] is True for k in ("connectionsOutputDisabled", "pluginOutputsReviewed", "noActiveRouteOutput")),
                "Audit candidate/read-only assertions differ")
        for value in (p["createdUtc"], plan["reviewedUtc"], audit["reviewedUtc"]):
            require(e["createdUnix"] <= utc_seconds(value) <= now, "Artifact review outside session")
        require(int(utc_seconds(audit["reviewedUtc"])) == e["reviewedUnix"], "Expected audit review differs")
        require(inv["context"] == context and object_digest(context) == e["contextContentSha256"] and
                object_digest(inv["trees"]) == e["treesContentSha256"] and
                object_digest(environment) == e["environmentContentSha256"], "Context/tree/environment content differs")
        require(type(inv["trees"]) is list and type(context["pluginRoots"]) is list and
                len(inv["trees"]) <= 64 and len(context["pluginRoots"]) <= 64, "Tree inventory bound")
        roots = [source_path(tree["root"]).casefold() for tree in inv["trees"]]
        expected_roots = [source_path(root).casefold() for root in context["pluginRoots"]]
        require(len(set(roots)) == len(roots) == len(set(expected_roots)) == len(expected_roots) and
                set(roots) == set(expected_roots), "Incomplete or duplicate loader-tree inventory")
        closed(environment, ("workingDirectory", "path"))
        require(environment["workingDirectory"] == ntpath.dirname(installation["executable"]), "Working directory differs")
        parts = environment["path"].split(";")
        require(parts and parts[0] == environment["workingDirectory"] and len(parts) <= 16 and
                len(set(x.casefold() for x in parts)) == len(parts), "Environment PATH differs")
        for part in parts:
            source_path(part)
        for snapshot in snaps.values():
            snapshot.recheck()
        return {"status": "content-verified-only", "launchAuthorized": False,
                "session": e["session"], "checkedArtifacts": len(snaps), "nativeCustodyQualified": False}


def validate(expectation_json, now):
    try:
        return _validate(expectation_json, now)
    except Refused:
        raise
    except (KeyError, TypeError, AttributeError, OSError, OverflowError) as error:
        # No partial report is returned for missing fields, inaccessible bytes,
        # unexpected record types, or unsupported filesystem observations.
        raise Refused("Incomplete, unreadable or unsupported artifact evidence") from error
