"""Native Win32 UI automation and PNG capture; test tooling, never product code."""
import ctypes as C
from ctypes import wintypes as W
import json
from pathlib import Path
import re
import struct
import time
import zlib

user = C.WinDLL('user32', use_last_error=True)
gdi = C.WinDLL('gdi32', use_last_error=True)
CALLBACK = C.WINFUNCTYPE(W.BOOL, W.HWND, W.LPARAM)

def declare(dll, name, result, *args):
    function = getattr(dll, name)
    function.restype = result
    function.argtypes = args
    return function

EnumWindows = declare(user, 'EnumWindows', W.BOOL, CALLBACK, W.LPARAM)
EnumChildWindows = declare(user, 'EnumChildWindows', W.BOOL, W.HWND, CALLBACK, W.LPARAM)
GetWindowThreadProcessId = declare(user, 'GetWindowThreadProcessId', W.DWORD, W.HWND, C.POINTER(W.DWORD))
GetClassNameW = declare(user, 'GetClassNameW', C.c_int, W.HWND, W.LPWSTR, C.c_int)
GetWindowTextW = declare(user, 'GetWindowTextW', C.c_int, W.HWND, W.LPWSTR, C.c_int)
IsWindowVisible = declare(user, 'IsWindowVisible', W.BOOL, W.HWND)
IsWindowEnabled = declare(user, 'IsWindowEnabled', W.BOOL, W.HWND)
WindowFromPoint = declare(user, 'WindowFromPoint', W.HWND, W.POINT)
SetCursorPos = declare(user, 'SetCursorPos', W.BOOL, C.c_int, C.c_int)
MouseEvent = declare(user, 'mouse_event', None, W.DWORD, W.DWORD, W.DWORD, W.DWORD, C.c_size_t)
GetWindowRect = declare(user, 'GetWindowRect', W.BOOL, W.HWND, C.POINTER(W.RECT))
GetClientRect = declare(user, 'GetClientRect', W.BOOL, W.HWND, C.POINTER(W.RECT))
GetParent = declare(user, 'GetParent', W.HWND, W.HWND)
IsChild = declare(user, 'IsChild', W.BOOL, W.HWND, W.HWND)
ScreenToClient = declare(user, 'ScreenToClient', W.BOOL, W.HWND, C.POINTER(W.POINT))
ChildWindowFromPointEx = declare(user, 'ChildWindowFromPointEx', W.HWND, W.HWND, W.POINT, W.UINT)
GetDpiForWindow = declare(user, 'GetDpiForWindow', W.UINT, W.HWND)
SetForegroundWindow = declare(user, 'SetForegroundWindow', W.BOOL, W.HWND)
SetWindowPos = declare(user, 'SetWindowPos', W.BOOL, W.HWND, W.HWND, C.c_int, C.c_int, C.c_int, C.c_int, W.UINT)
PostMessageW = declare(user, 'PostMessageW', W.BOOL, W.HWND, W.UINT, W.WPARAM, W.LPARAM)
SendMessageW = declare(user, 'SendMessageW', C.c_ssize_t, W.HWND, W.UINT, W.WPARAM, W.LPARAM)
GetMenu = declare(user, 'GetMenu', W.HMENU, W.HWND)
GetSubMenu = declare(user, 'GetSubMenu', W.HMENU, W.HMENU, C.c_int)
GetMenuItemCount = declare(user, 'GetMenuItemCount', C.c_int, W.HMENU)
GetMenuItemID = declare(user, 'GetMenuItemID', W.UINT, W.HMENU, C.c_int)
GetMenuStringW = declare(user, 'GetMenuStringW', C.c_int, W.HMENU, W.UINT, W.LPWSTR, C.c_int, W.UINT)
GetDC = declare(user, 'GetDC', W.HDC, W.HWND)
ReleaseDC = declare(user, 'ReleaseDC', C.c_int, W.HWND, W.HDC)
CreateCompatibleDC = declare(gdi, 'CreateCompatibleDC', W.HDC, W.HDC)
SelectObject = declare(gdi, 'SelectObject', W.HGDIOBJ, W.HDC, W.HGDIOBJ)
DeleteObject = declare(gdi, 'DeleteObject', W.BOOL, W.HGDIOBJ)
DeleteDC = declare(gdi, 'DeleteDC', W.BOOL, W.HDC)
PrintWindow = declare(user, 'PrintWindow', W.BOOL, W.HWND, W.HDC, W.UINT)
BitBlt = declare(gdi, 'BitBlt', W.BOOL, W.HDC, C.c_int, C.c_int, C.c_int, C.c_int, W.HDC, C.c_int, C.c_int, W.DWORD)

class BitmapHeader(C.Structure):
    _fields_ = [('size', W.DWORD), ('width', W.LONG), ('height', W.LONG),
                ('planes', W.WORD), ('bitcount', W.WORD), ('compression', W.DWORD),
                ('image_size', W.DWORD), ('xppm', W.LONG), ('yppm', W.LONG),
                ('used', W.DWORD), ('important', W.DWORD)]

CreateDIBSection = declare(gdi, 'CreateDIBSection', W.HBITMAP, W.HDC,
                           C.POINTER(BitmapHeader), W.UINT, C.POINTER(C.c_void_p), W.HANDLE, W.DWORD)

# Keep automation coordinates in physical pixels during monitor DPI changes.
# The application uses the pinned upstream PerMonitorV2 manifest independently.
try:
    SetProcessDpiAwarenessContext=declare(user,'SetProcessDpiAwarenessContext',W.BOOL,C.c_void_p)
    if not SetProcessDpiAwarenessContext(C.c_void_p(-4)):
        user.SetProcessDPIAware()
except AttributeError:
    user.SetProcessDPIAware()

def text(handle):
    buffer = C.create_unicode_buffer(2048)
    GetWindowTextW(handle, buffer, len(buffer))
    return buffer.value

def windows(pid=None):
    result = []
    @CALLBACK
    def visit(handle, _):
        process = W.DWORD()
        GetWindowThreadProcessId(handle, C.byref(process))
        if IsWindowVisible(handle) and (pid is None or process.value == pid):
            result.append((handle, process.value, text(handle)))
        return True
    EnumWindows(visit, 0)
    return result

def children(handle):
    result = []
    @CALLBACK
    def visit(child, _):
        if IsWindowVisible(child):
            result.append((child, text(child)))
        return True
    EnumChildWindows(handle, visit, 0)
    return result

def wait_window(title, pid=None, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for handle, process, label in windows(pid):
            if label == title:
                return handle, process
        time.sleep(.1)
    raise RuntimeError(f'Window not found: {title}; visible: {windows(pid)}')

def cycle_light(pid):
    """Cycle the status-bar palette, independent of Display page choices."""
    matches=[]
    for root,_,_ in windows(pid):
        controls=children(root)
        alerts=[h for h,t in controls if re.fullmatch(r'Alerts(?: \d+)?',t)]
        if len(alerts)!=1:
            continue
        top=GetParent(alerts[0])
        matches.extend((h,t) for h,t in controls
                       if t in ('Day','Dusk','Night') and GetParent(h)==top)
    assert len(matches)==1, ('Status-bar palette control must be unique', matches)
    handle,label=matches[0]
    assert IsWindowEnabled(handle), ('Status-bar palette control is disabled', label)
    rect=W.RECT()
    assert GetClientRect(handle,C.byref(rect)) and rect.right>0 and rect.bottom>0
    position=(rect.right//2)|((rect.bottom//2)<<16)
    SendMessageW(handle,0x201,1,position)
    SendMessageW(handle,0x202,0,position)
    time.sleep(.4)

def accelerator(handle,key):
    """Exercise the actual frame keyboard path, including CI-only scenarios."""
    assert IsWindowEnabled(handle) and IsWindowVisible(handle)
    SetForegroundWindow(handle)
    foreground=declare(user,'GetForegroundWindow',W.HWND)
    deadline=time.monotonic()+3
    while foreground()!=handle and time.monotonic()<deadline:time.sleep(.05)
    assert foreground()==handle, 'Test frame did not receive keyboard focus'
    code=111+int(key[1:]) if re.fullmatch(r'F(?:[1-9]|1[0-2])',key) else ord(key.upper())
    keyboard=declare(user,'keybd_event',None,W.BYTE,W.BYTE,W.DWORD,C.c_size_t)
    for vk in (0x11,0x10,code):keyboard(vk,0,0,0)
    for vk in (code,0x10,0x11):keyboard(vk,0,2,0)
    time.sleep(.5)

def click_text(pid, label):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        for root, _, _ in windows(pid):
            for handle, caption in children(root):
                if caption == label:
                    native_class=C.create_unicode_buffer(128)
                    GetClassNameW(handle,native_class,len(native_class))
                    # A confirmation sheet can use the same heading and button
                    # text. Static text is never an actionable control.
                    if native_class.value.lower() == 'static':
                        continue
                    # XNav controls in scrolled content must actually be visible
                    # before interaction. HWND visibility alone includes clipped
                    # offscreen children and would hide higher-DPI regressions.
                    ancestor=GetParent(handle);viewport=None
                    while ancestor:
                        if text(ancestor).startswith(('OpenNav Alpha page:', 'OpenNav product page:', 'OpenNav page:')):
                            viewport=ancestor;break
                        ancestor=GetParent(ancestor)
                    if viewport:
                        item=W.RECT();area=W.RECT()
                        GetWindowRect(handle,C.byref(item));GetWindowRect(viewport,C.byref(area))
                        if item.top<area.top or item.bottom>area.bottom:
                            direction='Up' if item.top<area.top else 'Down'
                            buttons=[h for h,t in children(root) if t==direction and IsWindowEnabled(h)]
                            if len(buttons)==1:
                                b=buttons[0];r=W.RECT();GetClientRect(b,C.byref(r))
                                pos=(r.right//2)|((r.bottom//2)<<16)
                                SendMessageW(b,0x201,1,pos);SendMessageW(b,0x202,0,pos)
                                time.sleep(.3)
                            continue
                    rect = W.RECT()
                    GetClientRect(handle, C.byref(rect))
                    position = (rect.right // 2) | ((rect.bottom // 2) << 16)
                    SendMessageW(handle, 0x201, 1, position)
                    SendMessageW(handle, 0x202, 0, position)
                    time.sleep(.4)
                    return
        time.sleep(.1)
    visible = [(title, children(h)) for h, _, title in windows(pid)]
    raise RuntimeError(f'Control not found: {label}: {visible}')

def pointer_text(pid, label):
    """Click a fully visible native control through the actual Windows pointer.

    Unlike a direct HWND message, this cannot activate a covered or clipped
    action. Used by the visible Preferences recovery path after footer removal.
    """
    deadline = time.monotonic() + 8
    last_scroll = None
    rejected = {}
    foreground = declare(user, 'GetForegroundWindow', W.HWND)
    ancestor = declare(user, 'GetAncestor', W.HWND, W.HWND, W.UINT)
    def class_name(handle):
        value = C.create_unicode_buffer(128)
        GetClassNameW(handle, value, len(value))
        return value.value
    def pointer_evidence(handle):
        """Describe a native pointer target without dereferencing a NULL HWND."""
        if not handle:
            return dict(hwnd=0, native_class='', caption='', rect=None)
        rect = W.RECT()
        has_rect = bool(GetWindowRect(handle, C.byref(rect)))
        return dict(hwnd=int(handle), native_class=class_name(handle),
                    caption=text(handle),
                    rect=[rect.left, rect.top, rect.right, rect.bottom]
                    if has_rect else None)
    while time.monotonic() < deadline:
        candidates = {}
        scroll_candidates = {}
        for root, _, _ in windows(pid):
            for handle, caption in children(root):
                if caption != label or not IsWindowEnabled(handle):
                    continue
                native_class = class_name(handle)
                if native_class.lower() == 'static':
                    continue
                rect = W.RECT()
                assert GetWindowRect(handle, C.byref(rect))
                if rect.right <= rect.left or rect.bottom <= rect.top:
                    continue
                # The whole target, not just its midpoint, must fit every
                # containing pane, including a scrolled Preferences body.
                parent = GetParent(handle)
                surface = ancestor(handle, 2)
                contained = True
                parent_chain = []
                containment_rejection = None
                while parent:
                    area = W.RECT()
                    assert GetWindowRect(parent, C.byref(area))
                    parent_chain.append(dict(handle=int(parent), caption=text(parent),
                                             rect=[area.left, area.top, area.right, area.bottom]))
                    if not (area.left <= rect.left < rect.right <= area.right and
                            area.top <= rect.top < rect.bottom <= area.bottom):
                        contained = False
                        containment_rejection = int(parent)
                        if (text(surface) == 'OpenNav preferences' and
                                area.left <= rect.left < rect.right <= area.right):
                            scroll_candidates[handle] = (surface, parent, area, rect)
                        break
                    if parent == surface:
                        break
                    parent = GetParent(parent)
                point = W.POINT((rect.left + rect.right)//2, (rect.top + rect.bottom)//2)
                hit = WindowFromPoint(point)
                if contained and hit == handle:
                    candidates[handle] = (surface, point)
                    rejected.pop(handle, None)
                else:
                    rejected[handle] = dict(
                        handle=int(handle), native_class=native_class,
                        rect=[rect.left, rect.top, rect.right, rect.bottom],
                        surface=int(surface or 0), parent_chain=parent_chain,
                        containment_rejection=containment_rejection,
                        midpoint=[point.x, point.y], hit_handle=int(hit or 0),
                        hit_class=class_name(hit) if hit else '',
                        hit_caption=text(hit) if hit else '',
                        hit_is_descendant=bool(hit and IsChild(handle, hit)),
                        target_is_descendant=bool(hit and IsChild(hit, handle)))
        if len(candidates) == 1:
            handle, (surface, point) = next(iter(candidates.items()))
            SetForegroundWindow(surface)
            while foreground() != surface and time.monotonic() < deadline:
                time.sleep(.05)
            assert foreground() == surface, ('Recovery surface did not activate', label)
            assert WindowFromPoint(point) == handle and IsWindowEnabled(handle), ('Recovery target moved or became covered', label)
            assert SetCursorPos(point.x, point.y)
            MouseEvent(2, 0, 0, 0, 0)
            time.sleep(.05)
            MouseEvent(4, 0, 0, 0, 0)
            time.sleep(.4)
            return
        assert len(candidates) <= 1, ('Visible pointer action is ambiguous', label, list(candidates))
        if len(scroll_candidates) == 1:
            handle, (surface, viewport, area, rect) = next(iter(scroll_candidates.items()))
            observed = (handle, rect.left, rect.top, rect.right, rect.bottom)
            assert observed != last_scroll, ('Visible Preferences scroll did not move target', label)
            last_scroll = observed
            SetForegroundWindow(surface)
            while foreground() != surface and time.monotonic() < deadline:
                time.sleep(.05)
            current_foreground = foreground()
            fresh_viewport = W.RECT()
            fresh_target = W.RECT()
            assert GetWindowRect(viewport, C.byref(fresh_viewport))
            assert GetWindowRect(handle, C.byref(fresh_target))
            initial_viewport_rect = [area.left, area.top, area.right, area.bottom]
            initial_target_rect = [rect.left, rect.top, rect.right, rect.bottom]
            fresh_viewport_rect = [fresh_viewport.left, fresh_viewport.top,
                                   fresh_viewport.right, fresh_viewport.bottom]
            fresh_target_rect = [fresh_target.left, fresh_target.top,
                                 fresh_target.right, fresh_target.bottom]
            geometry_identity = dict(
                selected_target_handle=int(handle),
                viewport_handle=int(viewport),
                surface_handle=int(surface),
                target_rect_unchanged=(fresh_target_rect == initial_target_rect),
                viewport_rect_unchanged=(fresh_viewport_rect == initial_viewport_rect),
                target_is_viewport_descendant=bool(
                    handle and viewport and IsChild(viewport, handle)),
            )
            area = fresh_viewport
            rect = fresh_target
            point = W.POINT((area.left + area.right)//2, (area.top + area.bottom)//2)
            hit = WindowFromPoint(point)
            hit_is_viewport = bool(hit and hit == viewport)
            hit_is_viewport_descendant = bool(hit and viewport and IsChild(viewport, hit))
            if not (current_foreground == surface and
                    geometry_identity['target_rect_unchanged'] and
                    geometry_identity['viewport_rect_unchanged'] and
                    geometry_identity['target_is_viewport_descendant'] and
                    (hit_is_viewport or hit_is_viewport_descendant)):
                raise AssertionError((
                    'Preferences scrolling surface covered', label,
                    dict(selected_target=pointer_evidence(handle),
                         viewport=pointer_evidence(viewport),
                         surface=pointer_evidence(surface),
                         current_foreground=pointer_evidence(current_foreground),
                         hit=pointer_evidence(hit),
                         wheel_point=[point.x, point.y],
                         geometry_identity=geometry_identity,
                         descendant_relationship=dict(
                             hit_is_viewport=hit_is_viewport,
                             hit_is_viewport_descendant=hit_is_viewport_descendant,
                             target_is_viewport_descendant=bool(
                                 handle and viewport and IsChild(viewport, handle)),
                             viewport_is_surface_descendant=bool(
                                 viewport and surface and IsChild(surface, viewport))))))
            assert SetCursorPos(point.x, point.y)
            wheel = 240 if rect.top < area.top else -240
            MouseEvent(0x0800, 0, 0, wheel & 0xffffffff, 0)
            time.sleep(.3)
        time.sleep(.1)
    raise AssertionError(('Fully visible pointer action not found', label,
                          [(title, children(h)) for h, _, title in windows(pid)],
                          list(rejected.values())))

def open_system(pid):
    """Reach recovery using the same three visible actions as the user."""
    for label in ('Settings', 'System', 'Interface & recovery'):
        pointer_text(pid, label)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        pages = [h for root, _, _ in windows(pid) for h, caption in children(root)
                 if caption == 'OpenNav product page: System']
        if len(pages) == 1:
            return
        time.sleep(.1)
    raise AssertionError('Visible Settings / System / Interface & recovery did not open System')

def control_text(handle):
    # GetWindowText reads another process's cached caption, not its EDIT buffer.
    # https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getwindowtextw
    buffer=C.create_unicode_buffer(32769)
    SendMessageW(handle,0x000D,len(buffer),C.cast(buffer,C.c_void_p).value)
    return buffer.value

def dismiss_native_dialog(dialog, label, timeout=10):
    """Click the visible modal button once, then require actual dismissal.

    Synthetic messages can reach a disabled parent while a modal is still open.
    Recovery tests must model the human dismissal before any parent command.
    """
    deadline = time.monotonic() + timeout
    SetForegroundWindow(dialog)
    button = None
    while time.monotonic() < deadline:
        matches = []
        for child, _ in children(dialog):
            native_class = C.create_unicode_buffer(128)
            GetClassNameW(child, native_class, len(native_class))
            if (native_class.value.lower() == 'button' and IsWindowEnabled(child)
                    and control_text(child).replace('&', '') == label):
                matches.append(child)
        if len(matches) == 1:
            rect = W.RECT()
            assert GetWindowRect(matches[0], C.byref(rect))
            point = W.POINT((rect.left + rect.right) // 2, (rect.top + rect.bottom) // 2)
            if WindowFromPoint(point) == matches[0]:
                button = matches[0]
                break
        time.sleep(.1)
    assert button, ('Visible enabled modal button not found', label, children(dialog))
    assert SetCursorPos(point.x, point.y)
    MouseEvent(2, 0, 0, 0, 0)
    time.sleep(.05)
    MouseEvent(4, 0, 0, 0, 0)
    while time.monotonic() < deadline:
        if not IsWindowVisible(dialog):
            return
        time.sleep(.1)
    raise RuntimeError(f'Modal remained visible after actual click: {label}')

def set_text_in_dialog(pid, title, previous, value):
    dialog,_=wait_window(title,pid)
    matches=[h for h,_ in children(dialog) if control_text(h)==previous]
    assert len(matches)==1,(title,previous,children(dialog))
    buffer=C.create_unicode_buffer(value)
    assert SendMessageW(matches[0],0x000C,0,C.cast(buffer,C.c_void_p).value),'Edit field rejected text'
    observed=control_text(matches[0])
    assert observed==value,(title,previous,value,observed)

def set_dialog_fields(pid, title, values):
    dialog,_=wait_window(title,pid)
    fields=[]
    for handle,_ in children(dialog):
        name=C.create_unicode_buffer(128)
        GetClassNameW(handle,name,len(name))
        if name.value.lower()=='edit':
            rect=W.RECT();GetWindowRect(handle,C.byref(rect))
            fields.append((rect.top,handle))
    fields.sort()
    assert len(fields)==len(values),(title,len(fields),len(values))
    for (_,handle),value in zip(fields,values):
        buffer=C.create_unicode_buffer(value)
        assert SendMessageW(handle,0x000C,0,C.cast(buffer,C.c_void_p).value)
        assert control_text(handle)==value,(title,value,control_text(handle))

def click_menu(handle, label):
    assert IsWindowEnabled(handle), 'Cannot invoke a disabled window menu behind a modal'
    def search(menu):
        for position in range(GetMenuItemCount(menu)):
            buffer = C.create_unicode_buffer(512)
            GetMenuStringW(menu, position, buffer, len(buffer), 0x400)
            if buffer.value.replace('&', '').split('\t')[0] == label:
                return GetMenuItemID(menu, position)
            child = GetSubMenu(menu, position)
            if child:
                found = search(child)
                if found is not None:
                    return found
        return None
    found = search(GetMenu(handle))
    if found is None:
        raise RuntimeError(f'Menu item not found: {label}')
    PostMessageW(handle, 0x111, found, 0)

def size_window(handle):
    if not SetWindowPos(handle, None, 0, 0, 1280, 800, 4):
        raise C.WinError(C.get_last_error())
    time.sleep(.5)
    rect = W.RECT()
    GetWindowRect(handle, C.byref(rect))
    assert (rect.right - rect.left, rect.bottom - rect.top) == (1280, 800)

def assert_page_geometry(handle, child, horizon=False):
    """Require the page to fill the actual center, including a visible alert.

    Alerts share the fixed status row; neither rail nor chart loses height.
    Native pane edges, minimum usable area and occlusion remain mandatory.
    """
    # Advanced pages can legitimately contain another Configure
    # instruments action. Only shell siblings define the page's outer bounds;
    # exclude every descendant, not just one assumed parent/grandparent level.
    labels = [(h,caption) for h,caption in children(handle) if not IsChild(child,h)]
    navigation = [h for h, caption in labels if caption == 'Chart']
    alerts = [h for h, caption in labels if re.fullmatch(r'Alerts(?: \d+)?',caption)]
    footer = [h for h, caption in labels if caption == 'OpenNav status footer']
    rail = [h for h, caption in labels if caption == 'Configure instruments']
    assert len(navigation) == len(alerts) == len(footer) == len(rail) == 1
    def bounds(window):
        value = W.RECT()
        assert GetWindowRect(window, C.byref(value))
        return value
    frame, rect = bounds(handle), bounds(child)
    top = bounds(GetParent(alerts[0])).bottom
    bottom = bounds(footer[0]).top
    if horizon:
        # The HTML Instruments full view retains the advisory timeline. Use
        # its exact responsive CSS height; no allowance for unexplained gaps.
        client=W.RECT();assert GetClientRect(handle,C.byref(client))
        scale=GetDpiForWindow(handle)/96
        height=client.bottom/scale
        bottom-=round((98 if height<=600 else 112 if height<=740 else 132)*scale)
    left = bounds(GetParent(navigation[0])).right
    right = bounds(GetParent(GetParent(rail[0]))).left
    tolerance = 1
    dimensions = [rect.right - rect.left, rect.bottom - rect.top]
    assert abs(rect.left-left)<=tolerance and abs(rect.right-right)<=tolerance, ('Page must fill space between prototype rails', rect.left, rect.right, left, right)
    assert dimensions[1] >= (frame.bottom-frame.top)//2, dimensions
    assert 0 <= rect.top-top <= tolerance, ('Page overlaps/leaves space below status/alerts', rect.top, top)
    assert 0 <= bottom-rect.bottom <= tolerance, ('Page overlaps/leaves space above navigation', rect.bottom, bottom)
    assert frame.left <= rect.left < rect.right <= frame.right
    return rect, dimensions

def assert_preview_page(handle, page):
    """Check native page bounds and sibling z-order after a real resize.

    Data assertions alone cannot detect a chart covering the selected page.
    This check is in addition to, not a substitute for, screenshot review.
    """
    if page == 'Route':
        return assert_prototype_drawer(handle, 'OpenNav passage')
    label = 'OpenNav page: ' + page
    matches = [child for child, caption in children(handle) if caption == label]
    assert len(matches) == 1, f'Visible page not found: {label}'
    child = matches[0]
    rect, dimensions = assert_page_geometry(handle, child)
    point = W.POINT((rect.left + rect.right) // 2, (rect.top + rect.bottom) // 2)
    assert ScreenToClient(handle, C.byref(point))
    assert ChildWindowFromPointEx(handle, point, 1) == child, 'Another pane covers the page'
    return {'page': page, 'native_pixels': dimensions, 'visible_and_uncovered': True}

def assert_product_page(handle, page):
    drawers={'Settings':'OpenNav preferences','AIS targets':'OpenNav vessel traffic','Anchor watch':'OpenNav anchor watch','Manual autopilot':'OpenNav autopilot','Alerts':'OpenNav alerts'}
    if page in drawers:
        return assert_prototype_drawer(handle,drawers[page])
    label='OpenNav product page: '+page
    matches=[child for child,caption in children(handle) if caption==label]
    assert len(matches)==1,f'Visible XNav page not found: {label}'
    child=matches[0];rect,_=assert_page_geometry(handle,child,horizon=page=='Vessel instruments')
    point=W.POINT((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
    assert ScreenToClient(handle,C.byref(point))
    assert ChildWindowFromPointEx(handle,point,1)==child,'Another pane covers the XNav page'

def prototype_drawer_bounds(width, height, scale=1, origin=(0,0), wide=False):
    """Exact final HTML desktop drawer rules, in native client coordinates."""
    assert scale>0 and width/scale>760 and height>0
    dip=lambda value:int(value*scale+.5)
    logical_width,logical_height=width/scale,height/scale
    top=56 if logical_height<=600 else 60 if logical_height<=740 else 68
    rail=156 if logical_width<=1100 else 186
    drawer=(410 if logical_width<=1100 else 432) if wide else 398
    w=dip(drawer)
    return dict(x=origin[0]+width-dip(rail)-dip(14)-w,
                y=origin[1]+dip(top)+dip(12),width=w,
                height=height-dip(top)-dip(34)-2*dip(12))

def assert_prototype_drawer(handle, name):
    """Owned prototype drawer stays in the chart workspace, above its canvas."""
    pid=W.DWORD();GetWindowThreadProcessId(handle,C.byref(pid))
    matches=[h for h,_,title in windows(pid.value) if title==name]
    assert len(matches)==1, ('Visible owned drawer required',name)
    rect=W.RECT();frame=W.RECT()
    assert GetWindowRect(matches[0],C.byref(rect)) and GetWindowRect(handle,C.byref(frame))
    assert GetParent(matches[0])==handle, 'Drawer belongs to the tested frame'
    scale=GetDpiForWindow(handle)/96
    client=W.RECT();assert GetClientRect(handle,C.byref(client))
    expected=(410 if client.right/scale<=1100 else 432) if name=='OpenNav preferences' else 398
    assert abs(rect.right-rect.left-expected*scale)<=1, ('Prototype drawer width',name)
    assert frame.left<rect.left<rect.right<frame.right and frame.top<rect.top<rect.bottom<frame.bottom
    point=W.POINT((rect.left+rect.right)//2,rect.top+int(45*scale))
    hit=WindowFromPoint(point)
    ancestor=declare(user,'GetAncestor',W.HWND,W.HWND,W.UINT)
    assert ancestor(hit,2)==matches[0], 'Another window covers the drawer'
    return {'page':name,'native_pixels':[rect.right-rect.left,rect.bottom-rect.top], 'visible_and_uncovered':True}

def assert_route_summary_layout(handle):
    """Destination summary reflows in the prototype status row."""
    labels = children(handle)
    alerts = [h for h,t in labels if re.fullmatch(r'Alerts(?: \d+)?',t)]
    assert len(alerts)==1, 'Status row must have one alert action'
    top = GetParent(alerts[0])
    summary = [h for h, text in labels if GetParent(h) == top and
               (text in ('Route unavailable', 'No active route') or
                re.fullmatch(r'[0-9]+\.[0-9] NM  /  \S.*', text))]
    assert len(summary)==1, 'Route summary missing or ambiguous'
    a,pane,frame=(W.RECT() for _ in range(3))
    for window, rect in ((summary[0], a), (top, pane), (handle, frame)):
        assert GetWindowRect(window, C.byref(rect))
    assert (frame.left <= pane.left <= a.left < a.right <= pane.right <= frame.right and
            frame.top <= pane.top <= a.top < a.bottom <= pane.bottom <= frame.bottom), \
        'Route summary is clipped or outside its status pane'
    siblings=[h for h,_ in labels if GetParent(h)==top and h!=summary[0]]
    for other in siblings:
        b=W.RECT();assert GetWindowRect(other,C.byref(b))
        assert not (a.left<b.right and b.left<a.right and a.top<b.bottom and b.top<a.bottom), 'Route summary overlaps a status control'

def capture(handle, path, resize=True, screen_pixels=False):
    if resize:
        size_window(handle)
    rect=W.RECT()
    assert GetWindowRect(handle,C.byref(rect))
    width,height=rect.right-rect.left,rect.bottom-rect.top
    assert 0 < width <= 8192 and 0 < height <= 8192
    screen = GetDC(None)
    memory = CreateCompatibleDC(screen)
    header = BitmapHeader(C.sizeof(BitmapHeader), width, -height, 1, 32, 0, 0, 0, 0, 0, 0)
    bits = C.c_void_p()
    bitmap = CreateDIBSection(screen, C.byref(header), 0, C.byref(bits), None, 0)
    if not bitmap:
        DeleteDC(memory)
        ReleaseDC(None, screen)
        raise C.WinError(C.get_last_error())
    previous = SelectObject(memory, bitmap)
    try:
        if screen_pixels:
            if not BitBlt(memory,0,0,width,height,screen,rect.left,rect.top,0x00CC0020):
                raise RuntimeError('Native screen capture failed')
        elif not PrintWindow(handle, memory, 2):
            raise RuntimeError('Native PrintWindow failed')
        bgra = C.string_at(bits, width * height * 4)
        rgb = bytearray(width * height * 3)
        rgb[0::3], rgb[1::3], rgb[2::3] = bgra[2::4], bgra[1::4], bgra[0::4]
        rows = b''.join(b'\0' + rgb[y*width*3:(y+1)*width*3] for y in range(height))
        def chunk(kind, data):
            return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))
        Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' +
            chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))
        Path(path).with_suffix('.json').write_text(json.dumps({
            'title': text(handle), 'outer_pixels': [width, height],
            'window_dpi': GetDpiForWindow(handle), 'capture': 'visible screen pixels' if screen_pixels else 'PrintWindow',
            'authority': 'native Windows', 'visual_review': 'required'}, indent=2))
        return bytes(rgb)
    finally:
        SelectObject(memory, previous)
        DeleteObject(bitmap)
        DeleteDC(memory)
        ReleaseDC(None, screen)

def close(handle):
    PostMessageW(handle, 0x10, 0, 0)

class DevMode(C.Structure):
    _fields_ = [('device', W.WCHAR * 32), ('spec_version', W.WORD), ('driver_version', W.WORD),
                ('size', W.WORD), ('extra', W.WORD), ('fields', W.DWORD),
                ('x', W.LONG), ('y', W.LONG), ('orientation', W.DWORD), ('fixed_output', W.DWORD),
                ('color', W.SHORT), ('duplex', W.SHORT), ('y_resolution', W.SHORT),
                ('tt_option', W.SHORT), ('collate', W.SHORT), ('form', W.WCHAR * 32),
                ('log_pixels', W.WORD), ('bits', W.DWORD), ('width', W.DWORD), ('height', W.DWORD),
                ('flags', W.DWORD), ('frequency', W.DWORD), ('icm_method', W.DWORD),
                ('icm_intent', W.DWORD), ('media_type', W.DWORD), ('dither_type', W.DWORD),
                ('reserved1', W.DWORD), ('reserved2', W.DWORD), ('pan_width', W.DWORD), ('pan_height', W.DWORD)]

EnumDisplaySettingsW = declare(user, 'EnumDisplaySettingsW', W.BOOL, W.LPCWSTR, W.DWORD, C.POINTER(DevMode))
ChangeDisplaySettingsW = declare(user, 'ChangeDisplaySettingsW', W.LONG, C.POINTER(DevMode), W.DWORD)

def ensure_desktop(minimum_width=1280,minimum_height=800):
    current = DevMode()
    current.size = C.sizeof(current)
    if not EnumDisplaySettingsW(None, 0xFFFFFFFF, C.byref(current)):
        raise RuntimeError('Cannot read native display mode')
    before = (current.width, current.height, current.bits)
    if current.width >= minimum_width and current.height >= minimum_height:
        return {'before': before, 'after': before}
    modes = []
    index = 0
    while True:
        candidate = DevMode()
        candidate.size = C.sizeof(candidate)
        if not EnumDisplaySettingsW(None, index, C.byref(candidate)):
            break
        if candidate.width >= minimum_width and candidate.height >= minimum_height and candidate.bits >= 24:
            modes.append(candidate)
        index += 1
    modes.sort(key=lambda m: m.width * m.height)
    for candidate in modes:
        # Session-only display change on the disposable CI desktop, not persisted.
        result = ChangeDisplaySettingsW(C.byref(candidate), 0)
        if result == 0:
            time.sleep(1)
            return {'before': before, 'after': (candidate.width, candidate.height, candidate.bits)}
    raise RuntimeError(f'No usable native display mode; current={before}, candidates={len(modes)}')

if __name__ == '__main__':
    import json
    print(json.dumps(ensure_desktop()))

kernel = C.WinDLL('kernel32', use_last_error=True)
OpenProcess = declare(kernel, 'OpenProcess', W.HANDLE, W.DWORD, W.BOOL, W.DWORD)
WaitForSingleObject = declare(kernel, 'WaitForSingleObject', W.DWORD, W.HANDLE, W.DWORD)
GetExitCodeProcess = declare(kernel, 'GetExitCodeProcess', W.BOOL, W.HANDLE, C.POINTER(W.DWORD))
CloseHandle = declare(kernel, 'CloseHandle', W.BOOL, W.HANDLE)

def monitor_process(pid):
    handle = OpenProcess(0x00100000 | 0x1000, False, pid)
    if not handle:
        raise C.WinError(C.get_last_error())
    return handle

def wait_exit_code(handle, timeout_ms=30000):
    """Measure a retained process handle; the caller defines its exit contract."""
    try:
        if WaitForSingleObject(handle, timeout_ms) != 0:
            raise RuntimeError('Native process did not exit within the deadline')
        code = W.DWORD()
        if not GetExitCodeProcess(handle, C.byref(code)):
            raise C.WinError(C.get_last_error())
        return code.value
    finally:
        CloseHandle(handle)

def wait_clean_exit(handle, timeout_ms=30000):
    code = wait_exit_code(handle, timeout_ms)
    assert code == 0, f'Native process exited with code {code}'
