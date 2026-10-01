#!/usr/bin/env python3
"""Verify installed opencpn-cmd fails closed for unavailable peer keys."""

from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path
import re
import subprocess
import tempfile


UNAVAILABLE_REASON = (
    "Local peer sharing is unavailable until secure pairing is qualified."
)
TEST_HOST = "peer-cli-test.invalid"
TEST_KEY = "73810462"
TIMEOUT_SECONDS = 20


def disposable_environment(root: Path) -> dict[str, str]:
    """Redirect environment-based paths; Windows config is handled separately."""
    env = os.environ.copy()
    home = root / "home"
    xdg_config = root / "xdg" / "config"
    xdg_data = root / "xdg" / "data"
    xdg_cache = root / "xdg" / "cache"
    xdg_state = root / "xdg" / "state"
    for directory in (home, xdg_config, xdg_data, xdg_cache, xdg_state):
        directory.mkdir(parents=True, exist_ok=True)

    env.update(
        {
            "HOME": str(home),
            "USERPROFILE": str(home),
            "APPDATA": str(root / "appdata" / "roaming"),
            "LOCALAPPDATA": str(root / "appdata" / "local"),
            "XDG_CONFIG_HOME": str(xdg_config),
            "XDG_DATA_HOME": str(xdg_data),
            "XDG_CACHE_HOME": str(xdg_cache),
            "XDG_STATE_HOME": str(xdg_state),
            "TMP": str(root),
            "TEMP": str(root),
            "TMPDIR": str(root),
        }
    )
    if os.name == "nt":
        # These contain incidental environment-based paths only. Pinned
        # wxMSW resolves BasePlatform's config root through CSIDL_COMMON_APPDATA.
        env["HOMEDRIVE"], env["HOMEPATH"] = str(root.drive), str(root)[len(root.drive) :]
    else:
        env.pop("FLATPAK_ID", None)
    return env


def initial_config() -> bytes:
    return (
        "[Settings]\n"
        "PeerCliDisposableSentinel=preserve-exactly\n"
        "[Settings/RestServer]\n"
        "ServerKeys=existing-server.invalid:sentinel-hash;\n"
        "[Settings/RESTClient]\n"
        "ServerKeys=existing-client.invalid:sentinel-hash;\n"
    ).encode("utf-8")


def windows_runner_temp() -> Path | None:
    """Restrict the native Windows case to an ephemeral hosted runner."""
    if os.name != "nt":
        return None
    runner_temp = os.environ.get("RUNNER_TEMP")
    if (
        os.environ.get("GITHUB_ACTIONS", "").lower() != "true"
        or os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted"
        or not runner_temp
    ):
        raise RuntimeError(
            "Native Windows CLI test is restricted to a disposable GitHub-hosted runner."
        )
    runner_temp_path = Path(runner_temp).resolve(strict=True)
    if not runner_temp_path.is_dir():
        raise RuntimeError("RUNNER_TEMP is not a directory.")
    return runner_temp_path


def common_app_data_dir() -> Path:
    """Read CSIDL_COMMON_APPDATA exactly as the pinned wxMSW code does."""
    if os.name != "nt":
        raise RuntimeError("Windows known-folder lookup requested on a non-Windows host")
    buffer = ctypes.create_unicode_buffer(32768)
    shell32 = ctypes.WinDLL("shell32", use_last_error=True)
    get_folder = shell32.SHGetFolderPathW
    get_folder.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_wchar_p,
    ]
    get_folder.restype = ctypes.c_long
    result = get_folder(None, 0x23, None, 0, buffer)  # CSIDL_COMMON_APPDATA
    if result != 0 or not buffer.value:
        raise RuntimeError(f"Cannot query CSIDL_COMMON_APPDATA (HRESULT {result:#x}).")
    return Path(buffer.value).resolve(strict=True)


def windows_config_target(runner_temp: Path) -> tuple[Path, Path]:
    """Seed only the absent app config dir under the hosted runner's known root."""
    common_root = common_app_data_dir()
    program_data = os.environ.get("ProgramData")
    if not program_data or Path(program_data).resolve(strict=True) != common_root:
        raise RuntimeError("ProgramData does not match the read-only Windows known-folder result.")
    if not runner_temp.resolve().is_relative_to(Path(os.environ["RUNNER_TEMP"]).resolve()):
        raise RuntimeError("Native Windows temp root is outside RUNNER_TEMP.")

    # BasePlatform::GetHomeDir() uses wxStandardPaths::GetConfigDir(). With
    # the pinned wxMSW 3.2.8 bundle, GetConfigDir() appends the wx app name
    # 'opencpn' to CSIDL_COMMON_APPDATA; BasePlatform then uses opencpn.ini.
    config_dir = common_root / "opencpn"
    if os.path.lexists(config_dir):
        raise RuntimeError(
            "Refusing native Windows CLI test: the exact OpenCPN common-data "
            "config directory already exists."
        )
    if not config_dir.parent.is_dir() or config_dir.is_symlink():
        raise RuntimeError("OpenCPN common-data config parent is not a safe known-folder directory.")
    config_dir.mkdir()
    config_path = config_dir / "opencpn.ini"
    try:
        with config_path.open("xb") as stream:
            stream.write(initial_config())
    except BaseException:
        config_path.unlink(missing_ok=True)
        config_dir.rmdir()
        raise
    return config_dir, config_path


def snapshot_config(paths: list[Path]) -> dict[Path, bytes | None]:
    return {path: path.read_bytes() if path.exists() else None for path in paths}


def safe_output_excerpt(result: subprocess.CompletedProcess[str]) -> str:
    output = (result.stdout + result.stderr).replace(TEST_KEY, "[redacted-test-key]")
    output = re.sub(r"(?<!\d)\d{6}(?!\d)", "[redacted-pin]", output)
    return output[:1000]


def assert_config_unchanged(
    before: dict[Path, bytes | None], candidates: list[Path]
) -> None:
    after = snapshot_config(candidates)
    if after != before:
        raise AssertionError("opencpn-cmd changed the isolated OpenCPN config file")


def run_command(executable: Path, args: list[str], env: dict[str, str], root: Path):
    try:
        return subprocess.run(
            [str(executable), *args],
            cwd=root,
            env=env,
            capture_output=True,
            text=True,
            timeout=TIMEOUT_SECONDS,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise AssertionError(
            f"opencpn-cmd {' '.join(args[:1])} exceeded {TIMEOUT_SECONDS}s"
        ) from error


def verify_refusal(
    label: str,
    args: list[str],
    executable: Path,
    env: dict[str, str],
    root: Path,
    candidates: list[Path],
    expected_reason: str,
) -> None:
    before = snapshot_config(candidates)
    result = run_command(executable, args, env, root)
    assert_config_unchanged(before, candidates)
    if result.returncode == 0:
        raise AssertionError(f"{label} unexpectedly succeeded")
    if expected_reason not in result.stderr:
        raise AssertionError(
            f"{label} did not report the peer-sharing unavailable reason; "
            f"exit={result.returncode}, output={safe_output_excerpt(result)!r}"
        )
    if result.stdout:
        raise AssertionError(f"{label} wrote stdout; key material must never be echoed")
    combined = result.stdout + result.stderr
    if TEST_KEY in combined:
        raise AssertionError(f"{label} echoed the disposable test key")
    if re.search(r"(?<!\d)\d{6}(?!\d)", result.stdout):
        raise AssertionError(f"{label} emitted a PIN-like key")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", required=True, type=Path, help="exact installed opencpn-cmd executable")
    parser.add_argument(
        "--unavailable-reason", default=UNAVAILABLE_REASON,
        help="exact unavailable reason expected on stderr",
    )
    args = parser.parse_args()
    executable = args.cli.resolve(strict=True)
    if not executable.is_file() or executable.name.lower() not in {"opencpn-cmd", "opencpn-cmd.exe"}:
        parser.error("--cli must identify an installed opencpn-cmd executable")

    windows_temp = windows_runner_temp()
    temporary_root = tempfile.TemporaryDirectory(
        prefix="peer-cli-", dir=str(windows_temp) if windows_temp else "/tmp"
    )
    windows_config_dir = None
    windows_config_path = None
    try:
        root = Path(temporary_root.name).resolve()
        env = disposable_environment(root)
        if os.name == "nt":
            windows_config_dir, windows_config_path = windows_config_target(root)
            candidates = [windows_config_path]
        else:
            home = root / "home"
            candidates = [
                home / ".opencpn" / "opencpn.conf",
                root / "xdg" / "config" / "opencpn" / "opencpn.conf",
                root / "xdg" / "data" / "opencpn" / "opencpn.conf",
            ]
            for path in candidates:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("xb") as stream:
                    stream.write(initial_config())
        verify_refusal(
            "generate-key",
            ["generate-key", TEST_HOST],
            executable,
            env,
            root,
            candidates,
            args.unavailable_reason,
        )
        verify_refusal(
            "store-key",
            ["store-key", TEST_HOST, TEST_KEY],
            executable,
            env,
            root,
            candidates,
            args.unavailable_reason,
        )
        print("Installed opencpn-cmd refused both peer-key commands without output or config changes.")
        return 0
    finally:
        if windows_config_dir is not None:
            if windows_config_path is not None:
                windows_config_path.unlink(missing_ok=True)
            runner_log = windows_config_dir / "opencpn.log"
            if runner_log.exists() and runner_log.is_file() and not runner_log.is_symlink():
                runner_log.unlink()
            windows_config_dir.rmdir()
        temporary_root.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
