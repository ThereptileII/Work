# SCRUM-275 scoped inventory lifetime repair

The original exact e6 Release failure is retained separately at `64bc1ab`.
The shared per-pass inventory now has stable scoped ownership in a `unique_ptr`.
The renderer borrows its address only during the synchronous point pass. The
scope destructor restores the prior library pointer before the owned inventory
is destroyed. No chart/waypoint pointer escapes the integration boundary.

Disabled/Standard, Paper Chart and allocation-failed scopes install a null
pointer for their duration. This deliberately shadows a still-live outer
inventory rather than consuming its points. Disabled/Paper scopes make no
allocation. The nothrow object allocation and existing bounded-map bad-allocation
fallback leave original stock drawing available. There is no warning suppression,
compiler flag weakening, parser callback or chart geometry change.

Focused verification uses the actual shared header and extracted unchanged
core/private CA methods and pinned LIGHTS06 conditional. Tests now run at `-O3`
with warnings as errors: core **133**, private **133**, no-GL core **131** checks.
They include nested active/inactive scopes, exception and early return, empty
outer/inner allocation failures, original stock dispatch on failure, and prior
inventory restoration without consumption. The allocation failure is injected
only by the test executable's standard nothrow allocation overload; production
has no test hook. Previous 121/121/119 assertions remain, with empty disabled
inventory expectations updated to the equivalent null representation.

`method-receipt.json` binds actual helper, fixture, upstream methods, executables
and compiler commands. Terminal painters/projection remain recorded boundaries;
these checks do not establish canvas acceptance. Full normal Release application
compile/link and native Windows/boat rendering remain pending. No boat, installer,
credential or hardware-control operation was performed.
