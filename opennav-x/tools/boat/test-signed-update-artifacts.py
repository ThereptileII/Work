"""Disposable snapshot tests; never reads a boat path or creates a process."""
from contextlib import ExitStack
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("artifacts", Path(__file__).with_name("signed_update_artifacts.py"))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def encoded(value):
    return json.dumps(value, ensure_ascii=False).encode("utf-8")


class Fixture:
    def __init__(self, root, mutate=lambda key, value: None):
        self.root, self.paths, self.docs = root, {}, {}
        self.now = 1800000060
        self.generation = "a" * 32
        self.commit = "b" * 40
        self.base = "C:\\XNav\\runs\\candidate"
        self.install = "C:\\Install"
        self.gen = self.install + "\\generations\\" + self.generation
        self.exe = self.gen + "\\app\\opencpn.exe"
        self.profile = "C:\\ProgramData\\opencpn"
        def save(key, value, source):
            mutate(key, value)
            self.docs[key] = value
            return self.write(key, encoded(value), source)
        save("state", {"schema": 1, "owner": "OpenNavX.Alpha1.SideBySide.1", "current": self.generation,
                       "stock": {"path": "C:\\OpenCPN\\opencpn.exe"}}, self.install + "\\state.json")
        self.write("executable", b"inert fixture executable bytes; never executed", self.exe)
        save("ownership", {"owner": "OpenNavX.Alpha1.SideBySide.1", "commit": self.commit,
                           "packageSha256": "c" * 64, "managedFiles": [{"path": "app/opencpn.exe", "sha256": self.sha("executable")}]},
             self.gen + "\\ownership.json")
        self.write("baseline", b"original profile", self.base + "\\baseline.ini")
        self.write("inputProfile", b"input-only fixture profile", self.base + "\\input-only.ini")
        self.write("profile", b"input-only fixture profile", self.profile + "\\opencpn.ini")
        context = {"installation": {"root": self.install, "generation": self.gen, "commit": self.commit,
                   "executable": self.exe, "stateSha256": self.sha("state"), "ownershipSha256": self.sha("ownership"),
                   "executableSha256": self.sha("executable")}, "workspace": "C:\\XNav", "profile": self.profile,
                   "executable": "C:\\OpenCPN\\opencpn.exe", "pluginRoots": ["C:\\Plugins"],
                   "launchEnvironment": {"workingDirectory": self.gen + "\\app",
                                         "path": self.gen + "\\app;C:\\Windows\\System32;C:\\Windows"}}
        stamp = "2027-01-15T08:00:00.0000000Z"  # Unix 1800000000.
        save("inventory", {"schema": 1, "owner": "OpenNavX.ReadOnlyCommissioning.Inventory.1", "context": copy.deepcopy(context),
                           "profileSha256": self.sha("baseline"), "trees": [{"root": "C:\\Plugins", "exists": True, "entries": []}]},
             self.base + "\\inventory.json")
        save("plan", {"schema": 1, "owner": "OpenNavX.ReadOnlyCommissioning.Plan.1", "reviewedUtc": stamp,
                      "inventorySha256": self.sha("inventory"), "plugins": []}, self.base + "\\review-plan.json")
        save("prepared", {"schema": 1, "owner": m.OWNER, "status": "prepared", "createdUtc": stamp,
                          "context": context, "baselineSha256": self.sha("baseline"), "inputSha256": self.sha("inputProfile"),
                          "inventorySha256": self.sha("inventory"), "planSha256": self.sha("plan"),
                          "applicationLaunched": False, "evidence": [], "quarantine": []}, self.base + "\\prepared.json")
        save("applied", {"schema": 1, "owner": m.OWNER, "status": "input-only-prepared", "recordSha256": self.sha("prepared"),
                         "profileSha256": self.sha("profile"), "applicationLaunched": False, "launchAuditStillRequired": True},
             self.base + "\\applied.json")
        save("active", {"schema": 1, "owner": m.OWNER, "record": self.base + "\\prepared.json", "recordSha256": self.sha("prepared")},
             "C:\\XNav\\commissioning-active.json")
        save("target", {"schema": 1, "owner": "OpenNavX.BoatTarget.1", "stockExecutable": context["executable"],
                        "profileDirectory": self.profile, "readOnlyAudit": {"reviewedUtc": stamp, "buildCommit": self.commit,
                        "profileIniSha256": self.sha("profile"), "connectionsOutputDisabled": True, "pluginOutputsReviewed": True,
                        "noActiveRouteOutput": True, "commissioning": {"record": self.base + "\\prepared.json",
                        "recordSha256": self.sha("prepared"), "appliedSha256": self.sha("applied")}}}, "C:\\XNav\\boat-target.json")
        self.expect = {"schema": 1, "session": "d" * 64, "identity": {"generation": self.generation,
                       "commit": self.commit, "packageSha256": "c" * 64, "executableSha256": self.sha("executable")},
                       "createdUnix": 1800000000, "expiresUnix": 1800003600, "reviewedUnix": 1800000000,
                       "objectHashAlgorithm": m.OBJECT_HASH, "contextContentSha256": m.object_digest(context),
                       "treesContentSha256": m.object_digest(self.docs["inventory"]["trees"]),
                       "environmentContentSha256": m.object_digest(context["launchEnvironment"]), "artifacts": self.paths}

    def write(self, key, data, source):
        path = self.root / key
        path.write_bytes(data)
        self.paths[key] = {"path": str(path), "sourcePath": source, "sha256": m.digest(data)}

    def sha(self, key):
        return self.paths[key]["sha256"]

    def run(self):
        return m.validate(encoded(self.expect), self.now)


class ArtifactTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="signed-content-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_positive_no_mutations_no_authority(self):
        f = Fixture(self.root)
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        result = f.run()
        self.assertEqual(result["checkedArtifacts"], 12)
        self.assertIs(result["launchAuthorized"], False)
        self.assertIs(result["nativeCustodyQualified"], False)
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.root.iterdir()})

    def test_each_artifact_changed_or_missing(self):
        for key in m.RECORDS + m.RAW:
            for missing in (False, True):
                with self.subTest(key=key, missing=missing):
                    f = Fixture(self.root)
                    p = self.root / key
                    p.unlink() if missing else p.write_bytes(p.read_bytes() + b" ")
                    with self.assertRaises(m.Refused):
                        f.run()

    def test_repinned_cross_identity_mismatches(self):
        cases = [
            ("state", ("current",), "e" * 32), ("ownership", ("commit",), "e" * 40),
            ("ownership", ("packageSha256",), "e" * 64), ("prepared", ("context", "installation", "commit"), "e" * 40),
            ("ownership", ("managedFiles",), []),
            ("prepared", ("context", "installation", "executableSha256"), "e" * 64),
            ("prepared", ("context", "installation", "stateSha256"), "e" * 64),
            ("prepared", ("context", "installation", "ownershipSha256"), "e" * 64),
            ("prepared", ("baselineSha256",), "e" * 64), ("prepared", ("inputSha256",), "e" * 64),
            ("prepared", ("planSha256",), "e" * 64), ("prepared", ("inventorySha256",), "e" * 64),
            ("applied", ("recordSha256",), "e" * 64), ("applied", ("status",), "partial"),
            ("applied", ("profileSha256",), "e" * 64), ("applied", ("applicationLaunched",), True),
            ("active", ("record",), "C:\\Other\\prepared.json"),
            ("plan", ("inventorySha256",), "e" * 64), ("inventory", ("profileSha256",), "e" * 64),
            ("inventory", ("context", "profile"), "C:\\Other"),
            ("inventory", ("trees",), []),
            ("target", ("readOnlyAudit", "buildCommit"), "e" * 40),
            ("target", ("readOnlyAudit", "profileIniSha256"), "e" * 64),
            ("target", ("readOnlyAudit", "commissioning", "appliedSha256"), "e" * 64),
            ("target", ("readOnlyAudit", "connectionsOutputDisabled"), "true"),
            ("target", ("profileDirectory",), "C:\\Other"),
            ("prepared", ("createdUtc",), "2020-01-01T00:00:00Z"),
            ("plan", ("reviewedUtc",), "2020-01-01T00:00:00Z"),
            ("target", ("readOnlyAudit", "reviewedUtc"), "2030-01-01T00:00:00Z"),
        ]
        for key, route, value in cases:
            with self.subTest(key=key, route=route):
                def mutate(name, obj):
                    if name == key:
                        for part in route[:-1]:
                            obj = obj[part]
                        obj[route[-1]] = value
                f = Fixture(self.root, mutate)
                with self.assertRaises(m.Refused):
                    f.run()

    def test_expected_identity_and_time(self):
        for key, value in (("contextContentSha256", "e" * 64), ("treesContentSha256", "e" * 64),
                           ("environmentContentSha256", "e" * 64), ("expiresUnix", 1800000060),
                           ("createdUnix", 1800000061), ("expiresUnix", 1800014401),
                           ("reviewedUnix", 1799999999), ("schema", True), ("session", "0" * 64)):
            with self.subTest(key=key):
                f = Fixture(self.root)
                f.expect[key] = value
                with self.assertRaises(m.Refused):
                    f.run()

    def test_strict_json_and_limits(self):
        for data in (b'{"x":1,"X":2}', b'{"x":1,"x":2}', b'{"x":NaN}', b'{"x":1.2}', b'{"x":1e4}',
                     b'{"x":"\xff"}', b'{"x":"\\ud800"}', b'[]', b'{}x', b'{"x":9223372036854775808}',
                     b'{"x":' + b'[' * 25 + b'0' + b']' * 25 + b'}', b' ' * (m.JSON_LIMIT + 1)):
            with self.subTest(data=data[:35]):
                with self.assertRaises(m.Refused):
                    m.strict_json(data)

    def test_missing_field_and_unknown_input_fail_closed(self):
        f = Fixture(self.root)
        f.expect["artifacts"]["extra"] = f.paths["state"]
        with self.assertRaises(m.Refused):
            f.run()

    def test_source_paths_and_content_hashes_are_not_receipt_hashes(self):
        for path in ("C:\\test\\..\\other", "C:\\test\\file:stream", "C:\\test\\CON.txt",
                     "C:\\test\\tail.", "\\\\server\\share\\file", "C:/test/file"):
            with self.subTest(path=path):
                with self.assertRaises(m.Refused):
                    m.source_path(path)
        f = Fixture(self.root)
        f.expect["objectHashAlgorithm"] = "powershell-receipt-hash"
        with self.assertRaises(m.Refused):
            f.run()
        f = Fixture(self.root)
        f.paths["applied"]["sourcePath"] = "C:\\Other\\applied.json"
        with self.assertRaises(m.Refused):
            f.run()

    def test_linked_ancestor_and_same_file_alias_refused(self):
        f = Fixture(self.root)
        linked = self.root / "linked"
        linked.symlink_to(self.root, target_is_directory=True)
        f.paths["profile"]["path"] = str(linked / "profile")
        with self.assertRaises(m.Refused):
            f.run()
        linked.unlink()
        f = Fixture(self.root)
        f.paths["profile"]["path"] = f.paths["inputProfile"]["path"]
        with self.assertRaises(m.Refused):
            f.run()
        f = Fixture(self.root, lambda key, value: value.pop("context") if key == "prepared" else None)
        with self.assertRaises(m.Refused):
            f.run()

    def test_links_empty_oversized_unsupported_file(self):
        f = Fixture(self.root)
        original = self.root / "profile"
        for kind in ("symlink", "hardlink", "empty", "oversized", "directory"):
            with self.subTest(kind=kind):
                other = self.root / "other"
                if kind == "symlink":
                    other.symlink_to(original)
                elif kind == "hardlink":
                    os.link(original, other)
                elif kind == "directory":
                    other.mkdir()
                else:
                    other.write_bytes(b"" if kind == "empty" else b"x" * 17)
                desc = {"path": str(other), "sourcePath": "C:\\test\\file", "sha256": m.digest(original.read_bytes())}
                try:
                    with ExitStack() as stack, self.assertRaises(m.Refused):
                        m.Snapshot(desc, 16, stack)
                finally:
                    other.rmdir() if kind == "directory" else other.unlink()

    def test_replacement_and_in_place_mutation_after_open(self):
        for replace in (True, False):
            with self.subTest(replace=replace):
                f = Fixture(self.root)
                with ExitStack() as stack:
                    snap = m.Snapshot(f.paths["profile"], m.PROFILE_LIMIT, stack)
                    if replace:
                        replacement = self.root / "replacement"
                        replacement.write_bytes(snap.data)  # Same bytes, different inode.
                        os.replace(replacement, snap.path)
                    else:
                        snap.path.write_bytes(b"x" * len(snap.data))
                    with self.assertRaises(m.Refused):
                        snap.recheck()


if __name__ == "__main__":
    unittest.main(verbosity=2)
