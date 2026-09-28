# Return navigation audit — 28 September 2026

Each active page, sheet or dialog has one local return action. Persistent main navigation remains available. Save, Apply, Continue and other actions that complete work are separate from return navigation.

## Decisions by context

| Page or pane | Single return action | Reason |
| --- | --- | --- |
| Main chart | None | It is the destination, with main navigation available. |
| Energy, Instruments, Radar opened from main navigation | Close | Return to the chart workspace. No extra Back row or Chart shortcut. |
| A focused view opened from a settings pane | Back | Restore the initiating settings tab or pane. |
| Settings, including all eight tabs | Close | Tabs are peers within one root pane. |
| Passage, Traffic, Anchor, source health, alerts, search, chart presentation, route advisory, chart position, chart object, instrument configuration opened directly | Close | Dismiss the root sheet to the underlying workspace. |
| Any of those panes opened from another pane | Back | Return to the actual parent, retaining its tab, fields and scroll. |
| Vessel details opened from Traffic | Back | Return to Traffic; remove the redundant All vessels button. |
| Vessel details opened directly from the chart | Close | All vessels remains a separate destination when it is not the parent. |
| Chart inspection opened from vessel details | Back to the originating context | The chart is a temporary inspection, not a new root destination. |
| Software centre, installation/recovery, updates, backups, diagnostics, plugins, adapter, About, Help, guides, chart library, chart package, alarm thresholds, passage library, sensor list, sensor details | Back when nested; Close when root | One shared rule based on the entry path. Updates no longer add a second Later/return button. |
| Symbol atlas | Back | Restore its initiating Settings tab, Chart presentation or Help pane. Seamark guide and Complete library are peer tabs. Filters and pagination do not add history. |
| Symbol / seamark detail | Back | Return to the atlas with its search, category, palette, page, region and scroll preserved. The underlying atlas is inert and its return button hidden while the detail sheet is open. |
| Symbol sources & coverage | Close | Dismiss the one-level informational dialog back to the atlas/detail. Links and export are work actions, not extra returns. |
| Sensor wizard: Connection | Cancel, in footer | Leave the draft and return to the initiating pane/setup step. |
| Sensor wizard: Discovery, Assignment, Verify | Back, in footer | Revisit the previous editable step. No header return. Adding the sensor finishes the flow and returns to its origin without repeating the sensor list. |
| Installer: Welcome | Cancel, in footer | Return to the initiating pane. |
| Installer: Detect host, Options, Review | Back, in footer | Revisit the prior step without a duplicate header control. |
| Installer: running | Cancel installation, in footer | Opens the cancellation decision; no Back to editable steps during the operation. |
| Installer: verified/Ready | Done, in footer | Leave the finished flow without reopening installation steps. Continue/setup is a separate forward action. |
| Installation guide opened inside the installer | Back | Restore the installer step and checkbox; suppress the redundant Open setup wizard link. |
| Vessel setup: Your vessel | Cancel, in footer | Return to the initiating pane. |
| Vessel setup: Display, Sources, Energy, Helm control, Ready | Back, in footer | Ready is a review before committing the profile. Open your helm completes setup. |
| Sensor list or guide opened within a wizard | Back | Restore the wizard step rather than imply dismissing the entire flow. |
| Repair, rollback, uninstall: review | Cancel | Leave before applying the simulated operation. |
| Repair, rollback, uninstall: running | Cancel operation | Cancel the preview and invalidate its pending completion callback. |
| Repair, rollback, uninstall: complete | Done | Return to the initiating pane without stale progress history. |
| New waypoint, chart import, calibration, backup review, update review | Cancel in the action row | Form cancellation is the single return. No additional dialog-header Back or Close. |
| Waypoint edit | Close in header | Save and Delete remain work actions. |
| Delete waypoint/source/chart, enable control | Keep waypoint / Keep sensor / Keep package / Keep control off | Explicit safe choice replaces generic duplicate dismiss buttons. |
| Installation cancellation dialog | Continue installation | Returns to the running flow; Cancel & roll back confirms cancellation and exits it. |
| Energy model, keyboard help, GPX preview, update history, place search, import error | Close in header | Informational/preview dialog; no redundant footer return. |
| Workspace chooser | Close | Current workspace is a status row, not a second close link. |
| Legacy/Safe mode confirmation from chooser | Back in action row | Return to the chooser, retaining the dialog hierarchy. Directly opened confirmations use Cancel. |
| Passage naming | Keep plotting | Return to the chart draft. |
| Passage saved | Done | Dismiss the success dialog; activation is a separate work action. |
| Plot, measure, place-waypoint tools | Cancel; measurement result uses Done | Tool controls replace the chart return while active. Cancel/Escape restore the initiating context. Measure again does not add another history entry. |

## Lighthouse selection in v8

Selecting the lighthouse pins its extended sectors without opening a drawer. The small chart card contains one Collapse action and a Details action. Details opens the shared chart-object sheet with its normal Close; the chart card is hidden while that sheet is open. Closing Details restores the pinned chart selection. Second selection, blank-chart click, Collapse and Escape clear the pin. Hover alone does not add navigation history.

## Interaction and presentation

Chart-symbol additions in v6 follow the same model: a chart object's sheet has one **Close**; its atlas reference has one **Back**. **Place on demo chart** temporarily replaces the return control with **Cancel**. Cancel/Escape restores the originating detail; successful placement leaves one **Back to Symbol details** on the chart and focuses the new mark. Removing a custom example restores its previous chart context. Underlying atlas controls remain inert while a detail is open.

- Root Close is beside the heading; nested Back is on the left. Accessible labels/tooltips identify the destination without an extra breadcrumb row.
- Wizards use only their footer for return navigation. Mobile service footers remain above main navigation; short desktop layouts keep them above the status bar.
- A dialog with a Cancel/Keep action does not also render a header dismiss button. Other dialogs render one Close, or Back when nested.
- Escape and Alt+Left follow the current context's return behavior. Dialog backdrop dismissal also returns one dialog layer.
- Closing a root context restores focus to its opener when that element still exists. Nested contexts restore the parent and its input values.
- Installer completion/cancellation and sensor completion discard obsolete step history. Pending cancelled preview operations cannot complete into a newer flow.

## Verification

The live-browser audit record is in `screenshots/v4/navigation-audit.json`. It records the active context and its visible local return controls, gathered through UI navigation. Representative desktop/mobile captures are in the same folder. `QA.md` lists functional and layout checks; the source inventory includes rarely triggered error/empty-state branches as well as the exercised paths.

This specifies prototype behavior. Real installation/recovery must define transactional cancellation against its actual installer before adopting the preview's cancellation behavior.
