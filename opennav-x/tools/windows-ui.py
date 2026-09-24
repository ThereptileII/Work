"""Native Win32 UI automation and PNG capture; test tooling, never product code."""
import ctypes as C
from ctypes import wintypes as W
import json
from pathlib import Path
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
GetWindowRect = declare(user, 'GetWindowRect', W.BOOL, W.HWND, C.POINTER(W.RECT))
GetClientRect = declare(user, 'GetClientRect', W.BOOL, W.HWND, C.POINTER(W.RECT))
ScreenToClient = declare(user, 'ScreenToClient', W.BOOL, W.HWND, C.POINTER(W.POINT))
ChildWindowFromPointEx = declare(user, 'ChildWindowFromPointEx', W.HWND, W.HWND, W.POINT, W.UINT)
GetDpiForWindow = declare(user, 'GetDpiForWindow', W.UINT, W.HWND)
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

class BitmapHeader(C.Structure):
    _fields_ = [('size', W.DWORD), ('width', W.LONG), ('height', W.LONG),
                ('planes', W.WORD), ('bitcount', W.WORD), ('compression', W.DWORD),
                ('image_size', W.DWORD), ('xppm', W.LONG), ('yppm', W.LONG),
                ('used', W.DWORD), ('important', W.DWORD)]

CreateDIBSection = declare(gdi, 'CreateDIBSection', W.HBITMAP, W.HDC,
                           C.POINTER(BitmapHeader), W.UINT, C.POINTER(C.c_void_p), W.HANDLE, W.DWORD)

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

def click_text(pid, label):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        for root, _, _ in windows(pid):
            for handle, caption in children(root):
                if caption == label:
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

def control_text(handle):
    # GetWindowText reads another process's cached caption, not its EDIT buffer.
    # https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getwindowtextw
    buffer=C.create_unicode_buffer(32769)
    SendMessageW(handle,0x000D,len(buffer),C.cast(buffer,C.c_void_p).value)
    return buffer.value

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

def assert_preview_page(handle, page):
    """Check native page bounds and sibling z-order after a real resize.

    Data assertions alone cannot detect a chart covering the selected page.
    This check is in addition to, not a substitute for, screenshot review.
    """
    label = 'OpenNav page: ' + page
    matches = [child for child, caption in children(handle) if caption == label]
    assert len(matches) == 1, f'Visible page not found: {label}'
    child = matches[0]
    rect = W.RECT()
    assert GetWindowRect(child, C.byref(rect))
    dimensions = [rect.right - rect.left, rect.bottom - rect.top]
    assert dimensions[0] >= 940 and dimensions[1] >= 500, dimensions
    point = W.POINT((rect.left + rect.right) // 2, (rect.top + rect.bottom) // 2)
    assert ScreenToClient(handle, C.byref(point))
    assert ChildWindowFromPointEx(handle, point, 1) == child, 'Another pane covers the page'
    return {'page': page, 'native_pixels': dimensions, 'visible_and_uncovered': True}

def assert_product_page(handle, page):
    label='OpenNav Alpha page: '+page
    matches=[child for child,caption in children(handle) if caption==label]
    assert len(matches)==1,f'Visible Alpha page not found: {label}'
    child=matches[0];rect=W.RECT();assert GetWindowRect(child,C.byref(rect))
    assert rect.right-rect.left>=940 and rect.bottom-rect.top>=500
    point=W.POINT((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
    assert ScreenToClient(handle,C.byref(point))
    assert ChildWindowFromPointEx(handle,point,1)==child,'Another pane covers the Alpha page'

def assert_route_summary_layout(handle):
    """A label hidden at narrow startup must rejoin its sizer when expanded."""
    labels = children(handle)
    summary = [h for h, text in labels if text == 'Route unavailable' or text.endswith(' NM to destination')]
    demo = [h for h, text in labels if text == 'Demo']
    assert len(summary) == len(demo) == 1, 'Route summary or Demo button missing'
    a, b = W.RECT(), W.RECT()
    assert GetWindowRect(summary[0], C.byref(a)) and GetWindowRect(demo[0], C.byref(b))
    assert a.left >= b.right and b.top <= a.top < a.bottom <= b.bottom, 'Route summary overlaps bottom controls'

def capture(handle, path):
    size_window(handle)
    width, height = 1280, 800
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
        if not PrintWindow(handle, memory, 2):
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
            'window_dpi': GetDpiForWindow(handle), 'rendering': 'software --no_opengl',
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

def ensure_desktop():
    current = DevMode()
    current.size = C.sizeof(current)
    if not EnumDisplaySettingsW(None, 0xFFFFFFFF, C.byref(current)):
        raise RuntimeError('Cannot read native display mode')
    before = (current.width, current.height, current.bits)
    if current.width >= 1280 and current.height >= 800:
        return {'before': before, 'after': before}
    modes = []
    index = 0
    while True:
        candidate = DevMode()
        candidate.size = C.sizeof(candidate)
        if not EnumDisplaySettingsW(None, index, C.byref(candidate)):
            break
        if candidate.width >= 1280 and candidate.height >= 800 and candidate.bits >= 24:
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

def wait_clean_exit(handle, timeout_ms=30000):
    try:
        if WaitForSingleObject(handle, timeout_ms) != 0:
            raise RuntimeError('Native process did not exit within the deadline')
        code = W.DWORD()
        if not GetExitCodeProcess(handle, C.byref(code)):
            raise C.WinError(C.get_last_error())
        assert code.value == 0, f'Native process exited with code {code.value}'
    finally:
        CloseHandle(handle)
