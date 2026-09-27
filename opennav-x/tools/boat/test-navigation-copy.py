#!/usr/bin/env python3
"""Offline-copy preservation cases; no boat/profile access."""
import importlib.util
from contextlib import closing, contextmanager
from pathlib import Path
import sqlite3
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("audit", Path(__file__).with_name("audit-navigation-copy.py"))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


@contextmanager
def database(path):
    with closing(sqlite3.connect(path)) as db:
        with db:
            yield db


class NavigationCopy(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "private-copy.db"
        with database(self.path) as db:
            for name in sorted(audit.REQUIRED):
                db.execute('CREATE TABLE "' + name.decode() + '"(id INTEGER, name TEXT, payload)')
            db.execute("INSERT INTO routes VALUES(1,CAST(X'41726BE973756E64' AS TEXT),NULL)")

    def report(self):
        return audit.audit(self.path, audit.digest(self.path))

    def test_non_utf8_text_preserved_without_decode(self):
        before = self.path.read_bytes()
        report = self.report()
        self.assertEqual(report["integrity"], "ok")
        self.assertEqual(self.path.read_bytes(), before)
        self.assertNotIn("Ar", str(report))

    def test_hash_mismatch_refused(self):
        with self.assertRaises(ValueError):
            audit.audit(self.path, "0" * 64)

    def test_live_journal_refused(self):
        for suffix in ("-wal", "-shm", "-journal"):
            marker = Path(str(self.path) + suffix)
            marker.touch()
            with self.assertRaises(ValueError):
                self.report()
            marker.unlink()

    def test_row_order_does_not_change_content(self):
        with database(self.path) as db:
            db.execute("INSERT INTO tracks VALUES(2,'b',NULL)")
            db.execute("INSERT INTO tracks VALUES(1,'a',NULL)")
        before = self.report()
        with database(self.path) as db:
            db.execute("DELETE FROM tracks")
            db.execute("INSERT INTO tracks VALUES(1,'a',NULL)")
            db.execute("INSERT INTO tracks VALUES(2,'b',NULL)")
        self.assertTrue(audit.compare(before, self.report()))

    def test_same_count_changed_value_detected(self):
        before = self.report()
        with database(self.path) as db:
            db.execute("UPDATE routes SET name='other'")
        self.assertFalse(audit.compare(before, self.report()))

    def test_storage_type_change_detected(self):
        before = self.report()
        with database(self.path) as db:
            db.execute("UPDATE routes SET name=X'41726BE973756E64'")
        self.assertFalse(audit.compare(before, self.report()))

    def test_duplicate_row_detected(self):
        before = self.report()
        with database(self.path) as db:
            db.execute("INSERT INTO routes SELECT * FROM routes")
        self.assertFalse(audit.compare(before, self.report()))

    def test_schema_change_detected(self):
        before = self.report()
        with database(self.path) as db:
            db.execute("CREATE INDEX additional_name ON routes(name)")
        self.assertFalse(audit.compare(before, self.report()))

    def test_missing_navigation_table_refused(self):
        with database(self.path) as db:
            db.execute("DROP TABLE tracks")
        with self.assertRaises(ValueError):
            self.report()

    def test_corruption_refused(self):
        self.path.write_bytes(b"broken database")
        with self.assertRaises(sqlite3.DatabaseError):
            self.report()

    def test_symlink_refused(self):
        link = self.path.with_name("linked.db")
        try:
            link.symlink_to(self.path)
        except OSError:
            self.skipTest("Host does not permit unprivileged symlinks")
        with self.assertRaises(ValueError):
            audit.audit(link, audit.digest(self.path))


if __name__ == "__main__":
    unittest.main()
