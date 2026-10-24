#!/usr/bin/env python3
"""Exercise cloud row-lock ordering in a disposable local PostgreSQL database.

Apply the Edge SQL migrations first. Uses the existing TEST_POSTGRES_* settings;
requires the exact CI database name and a local host/socket, never production.
"""
import concurrent.futures
import os
import time
import unittest
import uuid

import psycopg
from psycopg.types.json import Jsonb


def connect():
    name = os.environ.get("TEST_POSTGRES_DB")
    host = os.environ.get("TEST_POSTGRES_SOCKET") or os.environ.get("TEST_POSTGRES_HOST", "")
    if name != "scrum94_schema" or not (host.startswith("/") or host in {"127.0.0.1", "localhost", "::1"}):
        raise RuntimeError("Disposable local scrum94_schema database required")
    return psycopg.connect(dbname=name, host=host, user=os.environ["TEST_POSTGRES_USER"],
                           port=os.environ["TEST_POSTGRES_PORT"],
                           password=os.environ.get("TEST_POSTGRES_PASSWORD", ""),
                           options="-c statement_timeout=8000 -c lock_timeout=6000", autocommit=True)


class CloudRevocationConcurrency(unittest.TestCase):
    def setUp(self):
        self.admin = connect()
        self.account, self.device = uuid.uuid4(), uuid.uuid4()
        suffix = self.account.hex
        self.email = suffix + "@cloud-lock-test.invalid"
        self.subscription = "sub_" + suffix
        self.admin.execute("INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (%s,%s,now())",
                           (self.account, self.email))
        self.admin.execute("INSERT INTO skager_private.store_customers(email,account_id) VALUES (%s,%s)",
                           (self.email, self.account))
        self.admin.execute("INSERT INTO skager_private.store_subscriptions(subscription_id,email,status,current_period_end) "
                           "VALUES (%s,%s,'active',now()+interval '1 day')", (self.subscription, self.email))
        self.admin.execute("INSERT INTO skager_private.cloud_devices(id,account_id,token_hash,name,platform,app_version) "
                           "VALUES (%s,%s,%s,'Lock fixture','linux','1')", (self.device, self.account, suffix * 2))
        self.first, self.second = connect(), connect()
        self.first.autocommit = self.second.autocommit = False
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.items = Jsonb({"navigation.sog": {"v": 5, "age": 0, "q": "live", "src": "fixture"}})

    def tearDown(self):
        self.first.rollback()
        self.second.cancel()
        self.executor.shutdown(wait=True)
        self.second.rollback()
        self.first.close()
        self.second.close()
        self.admin.execute("DELETE FROM skager_private.cloud_devices WHERE id=%s", (self.device,))
        self.admin.execute("DELETE FROM skager_private.store_subscriptions WHERE subscription_id=%s", (self.subscription,))
        self.admin.execute("DELETE FROM skager_private.store_customers WHERE email=%s", (self.email,))
        self.admin.execute("DELETE FROM auth.users WHERE id=%s", (self.account,))
        self.admin.close()

    def put(self, connection):
        return connection.execute("SELECT public.cloud_telemetry_put(%s,%s)", (self.device, self.items)).fetchone()[0]

    def blocked(self, action):
        pid = self.second.info.backend_pid
        future = self.executor.submit(action)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not future.done():
            if self.admin.execute("SELECT cardinality(pg_blocking_pids(%s)) > 0", (pid,)).fetchone()[0]:
                return future
            time.sleep(0.01)
        if future.done():
            self.fail("Operation did not wait for the competing transaction: " + repr(future.result()))
        self.fail("Operation did not acquire the expected database lock")

    def no_data(self):
        for table in ("cloud_telemetry_latest", "cloud_telemetry_history"):
            # Fixed local table allowlist, not caller input.
            count = self.admin.execute(f"SELECT count(*) FROM skager_private.{table} WHERE device_id=%s", (self.device,)).fetchone()[0]
            self.assertEqual(count, 0)

    def revoked_before_write(self, sql, args, expected="revoked"):
        self.first.execute(sql, args)
        future = self.blocked(lambda: self.put(self.second))
        self.first.commit()
        self.assertEqual(future.result(timeout=5)["state"], expected)
        self.second.commit()
        self.no_data()

    def test_unlink_commits_before_inflight_write(self):
        self.revoked_before_write("SELECT public.cloud_device_forget(%s)", (self.device,))

    def test_refund_commits_before_inflight_write(self):
        self.revoked_before_write("SELECT skager_private.store_revoke(%s)", (self.email,))

    def test_subscription_cancellation_commits_before_inflight_write(self):
        self.revoked_before_write("UPDATE skager_private.store_subscriptions SET status='canceled' WHERE subscription_id=%s",
                                  (self.subscription,), "inactive")

    def test_account_ban_commits_before_inflight_write(self):
        self.revoked_before_write("UPDATE auth.users SET banned_until=now()+interval '1 day' WHERE id=%s", (self.account,))

    def test_write_commits_before_unlink(self):
        self.assertEqual(self.put(self.first)["state"], "stored")
        future = self.blocked(lambda: self.second.execute("SELECT public.cloud_device_forget(%s)", (self.device,)).fetchone())
        self.first.commit()
        future.result(timeout=5)
        self.second.commit()
        self.no_data()


if __name__ == "__main__":
    unittest.main()
