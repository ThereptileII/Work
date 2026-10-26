# SKAGER Companion: the boat on your phone

Epic SCRUM-385 (features SCRUM-386–393), built on SKAGER Cloud
([skager-cloud.md](skager-cloud.md)). One TypeScript web codebase in
`opennav-x/companion`, packaged for Android and iOS with Capacitor 8.

Companion is **read-only**. It shows what SKAGER on board reports and alarms the
owner. It cannot start or stop the anchor watch, steer, or send anything to the
boat. The anchor alarm is decided on board by OpenCPN's anchor watch, and it keeps
sounding there without internet. The cloud stores what the boat reported and
sends notifications; it never decides that a boat is dragging. Companion needs
SKAGER Cloud and boat data sharing switched on in SKAGER.

## Parts

| Part | Where | Role |
|---|---|---|
| Phone sign-in, phones, events, track, push queue | `web/edge/companion.sql` | Applied after `cloud.sql`. Tokens and codes are stored as SHA-256 only. Writes recheck account, subscription and device under the lock order of `cloud_telemetry_put`. |
| Phone API | `web/edge/companion.ts` | `/api/companion/*`. Uses a bearer token only; cookies are refused, and Origin must be absent or the app WebView. |
| Boat events, anchor in snapshot, sign-in grant, phones in portal | `web/edge/cloud.ts` | `/api/device/events`, `anchor` in `/api/device/telemetry`, `/api/cloud/companion/authorize`, `/api/cloud/phones/revoke`. |
| Notifications | `web/edge/push.ts` | Sends directly to FCM HTTP v1 (Android) and APNs (iOS), signing JWTs with WebCrypto. |
| Worker | `web/edge/cloudflare-worker.mjs` | Maps `Bearer skc1_…` to `X-Skager-Companion`. Passes the query string, `If-None-Match` and `ETag` for the phone API only. Answers CORS for `capacitor://localhost` and `https://localhost`. Serves `/app/authorize/` (portal shell). |
| Sign-in page and phone list | `web/edge/static/portal.js`, `auth.js` | `/app/authorize/?challenge=…&state=…` asks the signed-in owner to allow the phone. The SKAGER Cloud tab lists phones with Sign out. |
| Boat side | `src/cloud/*`, `src/integration/OnlineCloud.*` | Copies OpenCPN's anchor watch into each upload and sends anchor and critical-alert events at once, with a retry queue. |
| App | `companion/src` | Screens: Overview, Map, Anchor, History, Alarms and Settings. Sign-in, foreground refresh, offline cache and push are handled by the app. |

## Sign-in (RFC 8252 + PKCE)

1. The app makes a 43-character verifier and a state, then opens
   `https://beta.skager.app/app/authorize/?challenge=S256(verifier)&state=…` in the
   system browser (Custom Tabs or SFSafariViewController).
2. The owner signs in as usual (password plus e-mail code). `auth.js` returns only
   to `/link/` or to that exact authorize URL. The page then asks "Sign in on this
   phone?" and POSTs `/api/cloud/companion/authorize {challenge}` (cookie + CSRF).
   It receives a one-time code (2 minutes, at most 5 pending per account).
3. The page navigates to `app.skager.companion://auth?code=…&state=…`. The app
   checks the state and POSTs `/api/companion/token {code, verifier, name,
   platform, version}`. The code is consumed whatever the outcome. The app gets
   `skc1_` + 64 hex.
4. The token is kept in the Keychain or Keystore (`@aparajita/capacitor-secure-storage`,
   never synced). A session expires after 90 days without use, and an account has
   at most 10 phones. Phones are signed out from the app, from the portal, by
   closing the account, or by a refund.

Hardening that is still open: verified links (Android App Links and iOS Universal
Links) instead of the custom scheme. These need the release signing certificate
and the Apple Team ID.

## Data

- **Boat → Cloud:** every snapshot (15 s) carries `anchor`, which is OpenCPN's watch:
  armed, alarm, mark, signed radius, and distance from a current position with its
  age. A change of watch or alarm, and each new or ended critical XNav alert
  (except the anchor alarm itself), is sent at once to `/api/device/events`. Each
  event has a random id, so a retry is stored once. Up to 32 events are queued
  (alarms are kept first) and retried every 15 s while offline. Nothing is sent
  while sharing is off, or in demo or replay.
- **Cloud:**
  - Latest snapshot and anchor, plus 1-minute history for 30 days (as before).
  - Track points (live or aging positions) for 7 days.
  - Events for 90 days.
  - If the anchor watch is on and the boat has not reported for 3 minutes, the
    1-minute job raises **No contact** once per gap. The next upload records
    **Contact restored**.
  - Unlinking a boat computer deletes its track and events.
- **Cloud → phone:**
  - **Polling:** only while the app is on screen. The boat is fetched every 15 s
    with `If-None-Match`, so an unchanged snapshot is a body-less 304. Boats and
    events are fetched every 60 s, and the track every 60 s on the map. Offline,
    retries back off from 15 s to 30 s to 60 s, and nothing runs in the background.
  - **Ages:** they come from the server (`snapshotAge`) plus time measured on the
    phone, never by comparing clocks.
  - **Offline:** the last answers are cached (Preferences) and shown with their time.
- **Freshness in the app:** Current, Aging (older than 30 s or flagged aging by the
  boat), Old (older than 2 min or stale), Estimated, Not reported. The connection
  shows Online until 45 s, then Delayed, then Offline after 2 min. Requested values
  the boat never sent are listed as *Not reported*.

## Notifications

Queue: `cloud_events.push_state` (pending → sending → sent, failed, unconfigured or
expired).

**When notifications are sent**
- A device event is sent straight away, and the 1-minute job retries. A claim is
  retried after 2 minutes, at most 3 attempts. Nothing is pushed if it was received
  more than an hour ago.
- Pushed: anchor alarm, alarm ended, critical alert, no contact, contact restored.
- Only in the log: anchor watch on or off, alert ended.

**Delivery**
- Android uses channels `skager-alarms` (importance high, for alarm and no contact)
  and `skager-events`.
- iOS marks alarms as `time-sensitive`.

**What a notification contains**
- Boat name and the fixed server wording. It never contains a position.
- A late report says how many minutes after the event it arrived.

**Tokens**
- A rejected token (FCM UNREGISTERED, APNs 410 or BadDeviceToken) is removed.

**Without credentials** nothing is sent, and the alarm log still works.

### Owner set-up (not done; needs your accounts)

1. **Firebase (Android):**
   - Create a Firebase project and an Android app `app.skager.companion`.
   - Download `google-services.json` and add it as the repository secret
     `SKAGER_COMPANION_GOOGLE_SERVICES_JSON`. CI then builds an APK with push enabled.
   - Create a service-account key with the *Firebase Cloud Messaging API Admin* role.
   - Set the whole JSON as the Supabase function secret `SKAGER_FCM_SERVICE_ACCOUNT`.
2. **APNs (iOS):**
   - In Apple Developer: App ID `app.skager.companion` with Push Notifications and
     Time Sensitive Notifications.
   - Create a `.p8` key. Set these Supabase secrets:
     - `SKAGER_APNS_KEY` (the .p8 text)
     - `SKAGER_APNS_KEY_ID`
     - `SKAGER_APNS_TEAM_ID`
     - `SKAGER_APNS_TOPIC=app.skager.companion`
     - `SKAGER_APNS_HOST=api.sandbox.push.apple.com` for development builds
   - In Xcode, add the Push Notifications capability (entitlements) and set signing.
     Build with `SKAGER_PUSH=android,ios`.

## Deployment order (not yet done)

1. Apply `web/edge/companion.sql` after `cloud.sql`, as owner, in one transaction.
2. Redeploy the `skager-web` function (it adds `companion.ts` and `push.ts`) and set
   the push secrets above.
3. Upload the new Worker (`/api/companion/*`, `/app/authorize/`).
4. Release a SKAGER build containing the boat-side change; older builds still
   upload without `anchor`, which shows as unknown, never as "off".
5. Install the app: the CI artifact APK for testing. Store releases need signing
   keys and store listings.

## Build and test

- `.github/workflows/skager-companion.yml`:
  - **test:** edge tests; app type check and unit tests; SQL including
    `companion.postgres.sql` on PostgreSQL 18.6; the phone flow end to end.
  - **android:** `assembleDebug` produces `skager-companion-<version>-<sha>-debug.apk`.
  - **ios:** an unsigned simulator build.
- `cloud_tests anchor` covers the wire format and the event queue (C++).
- The e2e test and the browser preview are described in
  [companion/tests/e2e/README.md](../../companion/tests/e2e/README.md).

## Limitations and remaining gates

- The app has not been run on a physical Android phone or iPhone. Deep-link
  sign-in, Keystore and Keychain, notification permission and background delivery
  need device testing.
- Push is untested against the real FCM and APNs: no credentials yet. The JWT and
  request formats are unit-tested against generated keys.
- The boat side needs the native Windows build and a boat test: anchor alarm with
  the boat computer online, then offline (no contact), then back online.
- Map tiles depend on OpenFreeMap. Without them the map shows land colour plus
  the boat, track and circle. The map is not a chart and says so.
- The custom-scheme deep link should become a verified link before store release.
- Background polling and live updates without the app open are deliberately absent:
  push covers alarms.
