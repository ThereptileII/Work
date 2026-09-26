# Repeatable boat source checkout

`tools/boat/update-source.ps1 -Workspace C:\XNav -Commit <40-character-SHA>`
only retrieves source. It still requires the supported stock target preflight;
it does not build, install, launch, modify a profile or communicate with hardware.
The entrypoint fixes its public origin to `https://github.com/ThereptileII/Work.git`.
There is no origin override, branch fallback or authentication flow.

The previous `clone --no-checkout` approach was incorrect: Git creates an empty
index while HEAD already contains tracked files, and `status --porcelain` then
reports those files as staged deletions. The script rejected its own fresh clone
as local user changes. The regression test reproduces this behavior.

Fresh source now uses a new, journaled staging repository inside the workspace.
It initializes Git with an explicitly empty owned template, verifies the effective origin, fetches the exact commit,
verifies that the object is a commit, and checks it out detached. Only a clean,
verified tree is published by same-volume directory rename to `source`.
Byte-identical ownership records exist in `source-owner.json` and the checkout's
`.git/opennav-source-owner.json`; the last managed commit is recorded locally.
A failed fetch leaves its stage and intent for inspection and never publishes an
incomplete source tree.

Existing source must have those ownership records, the exact effective origin,
the last managed detached HEAD and a completely clean worktree/index. Tracked
edits, untracked files, ignored build artifacts, user branches/commits, modified
ownership, redirected metadata/worktree entries, external object storage and hidden index flags cause refusal. A
matching origin alone does not authorize taking over an existing directory.
No `reset --hard`, force checkout, `clean`, deletion or automatic adoption occurs.
Build outside the source tree to keep it clean for later exact-commit updates.

Credential helpers, interactive prompts, checkout hooks, fsmonitor hooks and
submodule recursion and automatic background maintenance are disabled for these Git invocations. Replace-object refs
cannot substitute source contents for the requested commit. Ambient Git
directory/index/object redirection is refused. Credential environment changes
are confined to the tool process and restored exactly, including absent values;
machine/user Git settings and authentication are untouched. The working directory
is passed as a Git argument, so spaces and `&` are not interpreted by a shell.
An entry-by-entry tree check refuses symlinks and Windows junctions before
descending, before existing Git operations and after fetch. This includes the
Git config, index, objects and refs: ownership of `.git` alone does not authorize
writing through a linked child. Plain-text `commondir`, `alternates` and
`http-alternates` redirects are also refused.

If interrupted between durable ownership creation and directory publication,
the next call stops instead of guessing which source tree it owns. Inspect the
intent and preserved staging directory. If a user changes an owned HEAD or leaves
local changes, preserve or relocate that work explicitly before further updates.

`tools/boat/test-source-checkout.ps1 -PortableContracts` passes 16 Linux groups
using local temporary Git repositories, with no network or boat access. The
same 16 groups also pass with process-local `core.autocrlf=true`: the intentional
dirty-file fixture restores Git's original checkout bytes, including native
line endings, before subsequent operations. The production dirty guard is unchanged.
The tests cover the original no-checkout bug, fresh publication, subsequent update,
Windows-style spaces/ampersands, user changes including ignored files, hidden
index flags, wrong origins, user HEAD/branches, disabled hooks, inherited Git
redirection, ignored ambient templates, linked metadata with unchanged outside
bytes, metadata/object redirects, failed fetch and mismatched ownership.
The same 16 groups pass in native Windows PowerShell 5.1 with the installed Git
on the boat PC, using only temporary repositories and no network, profile or
application operations. Windows CI remains a separate gate; `-IsolatedLocal`
explicitly permits this temporary-only test outside CI.
