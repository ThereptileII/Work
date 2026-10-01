"""Fail-closed disposable-profile boundary checks for peer sharing.

This module performs no network operations.  It checks only the target process
and files in the explicitly supplied disposable profile.
"""

from __future__ import annotations

import ctypes
import errno
import os
from pathlib import Path
import re
import socket
from ctypes import wintypes


_PORTS = frozenset((8443, 8444))
_MAX_PROC_TABLE_BYTES = 1 * 1024 * 1024
_SOCKET_RE = re.compile(r"^socket:\[(\d+)\]$")
_PROFILE_MARKER = b"Disposable disconnected UI test.\n"
_CLIENT_KEY = "boundary-peer:0123456789ab;"
_SERVER_KEY = "boundary-source:abcdef012345;"
_CERT_BYTES = b"OPENNAV_TEST_PROFILE_CERTIFICATE_SENTINEL\n"
_KEY_BYTES = b"OPENNAV_TEST_PROFILE_KEY_SENTINEL\n"


class PeerBoundaryError(RuntimeError):
    """The peer boundary could not be established safely."""


def _read(path: Path) -> bytes:
    try:
        return path.read_bytes()
    except OSError as exc:
        raise PeerBoundaryError(f"cannot read required profile file: {path.name}") from exc


def _config_value(config: bytes, section: str, key: str) -> str | None:
    current = None
    values = []
    for raw in config.decode("utf-8-sig", "strict").splitlines():
        line = raw.strip()
        if line.startswith("[") and line.endswith("]"):
            current = line[1:-1]
        elif current == section and "=" in line:
            name, value = line.split("=", 1)
            if name.strip() == key:
                values.append(value.strip())
    if len(values) > 1:
        raise PeerBoundaryError("ambiguous duplicate profile credential")
    return values[0] if values else None


def _insert_config(config: bytes, section: str, key: str, value: str) -> bytes:
    if _config_value(config, section, key) is not None:
        raise PeerBoundaryError(f"refusing to overwrite existing profile key: {key}")
    lines = config.decode("utf-8", "strict").splitlines(keepends=True)
    header = f"[{section}]\n"
    for index, raw in enumerate(lines):
        if raw.strip() == f"[{section}]":
            end = index + 1
            while end < len(lines) and not lines[end].lstrip().startswith("["):
                end += 1
            lines.insert(end, f"{key}={value}\n")
            return "".join(lines).encode("utf-8")
    if lines and not lines[-1].endswith("\n"):
        lines[-1] += "\n"
    lines.extend((header, f"{key}={value}\n"))
    return "".join(lines).encode("utf-8")


def _proc_identity(pid: int) -> tuple[str, str]:
    try:
        raw = Path(f"/proc/{pid}/stat").read_bytes()
    except OSError as exc:
        raise PeerBoundaryError("cannot read target process identity") from exc
    close = raw.rfind(b")")
    fields = raw[close + 2:].split() if close >= 0 else []
    if len(fields) <= 19:
        raise PeerBoundaryError("malformed target process identity")
    state = fields[0].decode("ascii", "strict")
    start = fields[19].decode("ascii", "strict")
    if state in ("Z", "X", "x") or not start.isdigit():
        raise PeerBoundaryError("target process is unavailable or a zombie")
    return state, start


def _owned_inodes(pid: int) -> set[int]:
    try:
        entries = os.listdir(f"/proc/{pid}/fd")
    except OSError as exc:
        raise PeerBoundaryError("cannot enumerate target descriptors") from exc
    result = set()
    for entry in entries:
        try:
            target = os.readlink(f"/proc/{pid}/fd/{entry}")
        except OSError as exc:
            if exc.errno == errno.ENOENT:
                continue
            raise PeerBoundaryError("cannot inspect target descriptor") from exc
        match = _SOCKET_RE.fullmatch(target)
        if match:
            result.add(int(match.group(1)))
    return result


def _linux_table(pid: int, family: str) -> dict[int, int]:
    path = Path(f"/proc/{pid}/net/{family}")
    try:
        with path.open("rb") as stream:
            data = stream.read(_MAX_PROC_TABLE_BYTES + 1)
    except OSError as exc:
        raise PeerBoundaryError(f"cannot read process {family} table") from exc
    if len(data) > _MAX_PROC_TABLE_BYTES:
        raise PeerBoundaryError(f"process {family} table is too large")
    lines = data.splitlines()
    if not lines:
        raise PeerBoundaryError(f"empty process {family} table")
    result = {}
    for raw in lines[1:]:
        fields = raw.split()
        if not fields:
            continue
        if len(fields) < 10:
            raise PeerBoundaryError(f"malformed process {family} row")
        try:
            state = fields[3].decode("ascii", "strict")
            inode = int(fields[9])
            port = int(fields[1].split(b":", 1)[1], 16)
        except (ValueError, IndexError, UnicodeDecodeError) as exc:
            raise PeerBoundaryError(f"malformed process {family} row") from exc
        if state == "0A":
            result[inode] = port
    return result


def linux_listeners(pid: int) -> list[int]:
    if type(pid) is not int or pid <= 0:
        raise PeerBoundaryError("invalid target process ID")
    _, expected_start = _proc_identity(pid)
    for _ in range(3):
        _, before = _proc_identity(pid)
        if before != expected_start:
            raise PeerBoundaryError("target process identity changed")
        owned = _owned_inodes(pid)
        tables = _linux_table(pid, "tcp")
        tables.update(_linux_table(pid, "tcp6"))
        found = sorted({port for inode, port in tables.items() if inode in owned})
        if set(found) & _PORTS:
            return found
        _, after = _proc_identity(pid)
        if before == after and owned == _owned_inodes(pid):
            return found
    raise PeerBoundaryError("target descriptor ownership changed during inspection")


DWORD = ctypes.c_uint32


class _TcpRow(ctypes.Structure):
    _fields_ = [(name, DWORD) for name in (
        "state", "local_addr", "local_port", "remote_addr",
        "remote_port", "pid"
    )]


class _Tcp6Row(ctypes.Structure):
    _fields_ = [
        ("local_addr", ctypes.c_ubyte * 16), ("local_scope", DWORD),
        ("local_port", DWORD), ("remote_addr", ctypes.c_ubyte * 16),
        ("remote_scope", DWORD), ("remote_port", DWORD),
        ("state", DWORD), ("pid", DWORD),
    ]


def windows_listeners(pid: int) -> list[int]:
    if type(pid) is not int or not 0 < pid <= 0xffffffff:
        raise PeerBoundaryError("invalid target process ID")
    if ctypes.sizeof(_TcpRow) != 24 or ctypes.sizeof(_Tcp6Row) != 56:
        raise PeerBoundaryError("unexpected native TCP row layout")
    kernel = ctypes.WinDLL("kernel32.dll")
    kernel.OpenProcess.argtypes = [DWORD, wintypes.BOOL, DWORD]
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(DWORD)]
    kernel.GetExitCodeProcess.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    handle = kernel.OpenProcess(0x1000, False, pid)
    if not handle:
        raise PeerBoundaryError("cannot inspect target process lifetime")

    def require_running():
        code = DWORD()
        if not kernel.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value != 259:
            raise PeerBoundaryError("target process is not running")

    try:
        require_running()
        api = ctypes.WinDLL("iphlpapi.dll").GetExtendedTcpTable
        api.argtypes = [ctypes.c_void_p, ctypes.POINTER(DWORD),
                        wintypes.BOOL, DWORD, DWORD, DWORD]
        api.restype = DWORD
        ports = []
        for family, row_type in ((2, _TcpRow), (23, _Tcp6Row)):
            size = DWORD(0)
            error = api(None, ctypes.byref(size), False, family, 3, 0)
            if error != 122 or not 4 <= size.value <= _MAX_PROC_TABLE_BYTES:
                raise PeerBoundaryError("GetExtendedTcpTable size query failed")
            for _ in range(4):
                buffer = ctypes.create_string_buffer(size.value)
                actual = DWORD(size.value)
                error = api(buffer, ctypes.byref(actual), False, family, 3, 0)
                if error == 0:
                    if not 4 <= actual.value <= len(buffer):
                        raise PeerBoundaryError("GetExtendedTcpTable output size is invalid")
                    break
                if error != 122 or not 4 <= actual.value <= _MAX_PROC_TABLE_BYTES:
                    raise PeerBoundaryError("GetExtendedTcpTable failed")
                size = actual
            else:
                raise PeerBoundaryError("GetExtendedTcpTable did not stabilize")
            count = DWORD.from_buffer_copy(buffer).value
            row_size = ctypes.sizeof(row_type)
            if count > (actual.value - 4) // row_size:
                raise PeerBoundaryError("GetExtendedTcpTable row count is invalid")
            for index in range(count):
                row = row_type.from_buffer_copy(buffer, 4 + index * row_size)
                if row.pid == pid and row.state == 2:
                    ports.append(socket.ntohs(row.local_port & 0xffff))
        require_running()
        return sorted(set(ports))
    finally:
        if not kernel.CloseHandle(handle):
            raise PeerBoundaryError("could not release process observation handle")


class PeerBoundary:
    def __init__(self, profile: str | os.PathLike[str]):
        self.profile = Path(profile)
        config = self.profile / "opencpn.conf"
        if _read(self.profile / "OPENNAV_TEST_PROFILE") != _PROFILE_MARKER:
            raise PeerBoundaryError("profile is not an OpenNav disposable profile")
        if not config.is_file():
            raise PeerBoundaryError("profile opencpn.conf is missing")
        self._config = config
        self.seed()

    def seed(self) -> None:
        config = _read(self._config)
        # Validate all preconditions before writing anything to this fixture.
        config = _insert_config(config, "Settings/RESTClient", "ServerKeys", _CLIENT_KEY)
        config = _insert_config(config, "Settings/RestServer", "ServerKeys", _SERVER_KEY)
        for name in ("cert.pem", "key.pem"):
            if os.path.lexists(self.profile / name):
                raise PeerBoundaryError(f"refusing to overwrite profile file: {name}")
        for name, content in (("cert.pem", _CERT_BYTES), ("key.pem", _KEY_BYTES)):
            path = self.profile / name
            with path.open("xb") as stream:
                stream.write(content)
        self._config.write_bytes(config)

    def assert_preserved(self) -> dict[str, object]:
        if _read(self.profile / "cert.pem") != _CERT_BYTES or _read(self.profile / "key.pem") != _KEY_BYTES:
            raise PeerBoundaryError("profile certificate or key changed")
        config = _read(self._config)
        if (_config_value(config, "Settings/RESTClient", "ServerKeys") != _CLIENT_KEY or
                _config_value(config, "Settings/RestServer", "ServerKeys") != _SERVER_KEY):
            raise PeerBoundaryError("profile REST credentials changed")
        return {"profile_preserved": True}

    def observe(self, pid: int) -> dict[str, object]:
        self.assert_preserved()
        if os.name == "nt":
            ports = windows_listeners(pid)
        else:
            ports = linux_listeners(pid)
        forbidden = sorted(set(ports) & _PORTS)
        if forbidden:
            raise PeerBoundaryError("target owns a forbidden peer listener")
        return {"profile_preserved": True, "pid": pid,
                "owned_tcp_listener_ports": ports}
