"""Match a published layout observation to the current native window controls.

This is a test-observation barrier, not a layout validator. Callers still assert
all content bounds and touch dimensions after a matching observation arrives.
"""


def matches_native_controls(record, expected, after_tick):
    """Require a later publication and exact visible native-control rectangles.

    `expected` comes from current HWND geometry, never from the diagnostic file.
    Duplicate/missing/hidden controls or stale snapshots cannot satisfy it.
    Content such as the rail is deliberately not part of this predicate: bad
    content bounds must reach the caller's assertions rather than be polled away.
    """
    if not expected:
        return False
    try:
        runtime = record['runtime']
        tick = int(runtime['ui_update']['ticks'])
        if tick <= after_tick:
            return False
        controls = runtime['display']['interaction_controls']
        for label, native in expected.items():
            found = [control for control in controls
                     if control['label'] == label and control['visible'] is True]
            if len(found) != 1:
                return False
            control = found[0]
            rect = (control['x'], control['y'],
                    control['x'] + control['width'],
                    control['y'] + control['height'])
            if rect != tuple(native):
                return False
        return True
    except (KeyError, TypeError, ValueError):
        return False
