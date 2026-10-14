# Original shortcut custody and installed-host capability — 2026-10-14

Exact component source `404cc83bd0873a6aa4231c1ba2ae2e4a1f8a3499` passes
[focused run 38528886583](https://github.com/ThereptileII/Work/actions/runs/38528886583).
Linux passes 143 top-level / 422 nested cases. Native Win32 Windows passes
216 top-level / 574 nested cases with three existing platform-only skips and
no failures. Source-closure checks pass. Both downloaded original artifacts
match provider hashes/lengths and all ZIP CRCs. `results.json` retains identity.

Normal, Legacy and Safe original shortcuts are retained through held native
handles with DELETE access and without delete sharing. Publication moves those
same objects to protected recovery. Unknown/modified additions are not adopted.
The first replacement fixture incorrectly demanded one exact Win32 error;
`failed-results.json` preserves it. The corrected fixture requires refusal and
checks both identities, locations, hashes and absent backup before publication.
The native replacement returned Access denied for every role. Success remains
forbidden; no product protection was relaxed.

The genuine B0 fixture now authenticates and runs the real Setup with all three
shortcuts, then retains the original same-account normal-token maintenance
process before allowing it to run. Sanitized launch evidence preserves native
failure provenance. Its portable tests are separate from installed acceptance.

[Read-only host probe 38528270601](https://github.com/ThereptileII/Work/actions/runs/38528270601)
at `67fadaae08eda62687f8701b5b1002a0f43db6dc` correctly reports **blocked**:
the hosted RID500 administrator has a high Default token and no linked normal
token (native error 1312). WinSta0/Default is visible; the consent policy is 0.
`host-capability.json` preserves authenticated report facts with the ephemeral
account SID omitted. This is a host limitation, not successful customer/UAC
acceptance. No user, policy, token, desktop, or installed product was changed.
A disposable interactive ordinary-administrator session is required for the
actual consent path; do not fabricate a restricted token or bypass UAC.

[Fresh-account probe 38531935235](https://github.com/ThereptileII/Work/actions/runs/38531935235)
at `68c3aa45552d9b3d5f89f79976ca410159658438` reached a genuine interactive
logon for one disposable ordinary administrator. Windows returned a limited
primary token and its linked full **impersonation** token, with matching SID and
session 2. The probe refused its assumption that both tokens were primary;
no child or product was launched. Its own account/profile cleanup passed.
Original artifact `11878317713` is 626 bytes, SHA-256
`a6bb4f00051335a34f7a97fa5b8d8e0a0c26493a364fe238ebdd8fd5c8daa58f`;
provider run/source/identity, length, digest and every ZIP CRC were checked.
Ephemeral account identity is omitted from this curated description.
This is failed capability evidence, not a successful installed test. The next
probe used the documented conversion of an original impersonation token to
a primary token while retaining the same security context; no groups,
privileges, UAC policy or desktop permissions were changed.
See Microsoft's [DuplicateTokenEx contract](https://learn.microsoft.com/en-us/windows/win32/api/securitybaseapi/nf-securitybaseapi-duplicatetokenex).

[Conversion probe 38532803351](https://github.com/ThereptileII/Work/actions/runs/38532803351)
at `bf1e3a1c5c9c5d60577efb5bee9c3de7c8fe9691` was refused by Windows with
`ERROR_BAD_IMPERSONATION_LEVEL` (1346), before process creation. Its own
account/profile cleanup again passed. Original artifact `11878638237` is
628 bytes, SHA-256
`212e6b8db6f89d80c5a099793c454657daad933531061d9ae03c8868d88d15fc`;
run/source, length, digest and every ZIP CRC were verified. This token route
stops here. Do not change impersonation levels, privileges, desktop ACLs or UAC
to force acceptance. A genuine authorized ordinary interactive Windows session
or disposable VM remains necessary for the unqualified customer path.

The failed conversion report did not retain the original FULL token's
`TOKEN_STATISTICS.ImpersonationLevel`. Error 1346 therefore establishes refusal
of that conversion, not impossibility of every genuine ordinary-account route.
The bounded follow-up probe records original FULL and LIMITED token statistics
before launch or conversion refusal. Its primary path uses the already genuine
LIMITED PRIMARY logon token directly, with unchanged security context and
AuthenticationId, for the fixed read-only `installed_customer_launch.py` noargs
child. It does not convert or launch with the linked FULL token.

The original child token/session/image and token AuthenticationId are checked
before its sole resume. Its actual WinSta0/Default observations are projected
separately as `limited-primary-read-only-child`; an inaccessible input desktop
stays blocked. `available` means only that this limited child observed the
interactive desktop. `elevatedParentLaunchAvailable`,
`sameAccountElevationTested`, `nativeCustomerAcceptance` and
`consentPromptAcceptance` remain false. The existing elevated-parent customer
launcher contract is unchanged. No UAC or product operation occurs. The explicit
desktop spelling avoids implicit permission additions; it does not grant the new
account desktop access. Native execution remains required to establish whether
that access exists. Original-account/profile cleanup and preservation of uncertain
live children remain mandatory. Portable fake-API checks grant no native pass.

Full application packaging, actual installed B0/B1, same-account customer/UAC,
repair/rollback/uninstall/profile preservation and boat acceptance remain open.
These component results do not close SCRUM-27 or authorize public release.
