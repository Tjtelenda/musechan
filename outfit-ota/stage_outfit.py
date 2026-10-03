#!/usr/bin/env python3
"""Stage a MuseChan/CoreS3 avatar update as a pending over-the-air (OTA) update.

The avatar face is compiled into the firmware, so an outfit change is:

  1. the paired Muse redraws the avatar renderer
     (``esp32/components/muse/avatar/muse_pixel.c``; see
     ``esp32/tools/muse/avatar.py`` and ``outfit-ota/README.md``),
  2. this script rebuilds the firmware with a timestamped version, uploads
     the ``.bin`` to an HTTPS URL, and records a pending OTA entry,
  3. the paired Muse sends that entry with the gadget command
     ``device.ota`` the next time the device is online on Wi-Fi, then marks
     the entry sent.

Building and staging can happen on any machine with ESP-IDF v6.0.1 and this
repo checked out. No USB cable is needed for the outfit update itself. The
final send is a Muse device command, so it is done by the Muse paired with
that device, not by this script directly.

Uploads: in a Muse cloud workspace, the default uploader is Muse's
``remote-storage upload-file --path <bin>`` helper when it is installed. To
use another host, set ``MUSECHAN_OTA_UPLOAD_COMMAND`` (or pass
``--upload-command``) to a command template containing ``{path}``. The
command must print either the Muse uploader JSON (with ``url`` and
``expires_at``) or a single HTTPS URL on the last line.

Version stamping: builds get version ``YYYY.MMDD.HMM`` (for example
``2026.1003.2100``), which is newer than the ``999.0.0`` development build
only when sent with ``force: true`` and newer than earlier stamps the same
day. ``version.txt`` is restored after the build so the repo stays clean.
"""
import argparse
import hashlib
import json
import os
import pathlib
import shlex
import subprocess
import sys
from datetime import datetime

HERE = pathlib.Path(__file__).resolve().parent
ESP32 = HERE.parent / "esp32"
VERSION_FILE = ESP32 / "version.txt"
DEFAULT_UPLOADER = "/opt/hatch/bin/remote-storage"

BOARDS = {
    "stackchan": "build-muse-m5stack-stackchan",
    "cores3": "build-muse-m5stack-cores3",
}


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run(cmd, check=True, **kw)


def pending_path(board):
    return HERE / ("pending.json" if board == "stackchan" else f"pending-{board}.json")


def upload(bin_path, upload_command):
    """Upload BIN_PATH and return (url, expires_at)."""
    if upload_command:
        cmd = shlex.split(upload_command.format(path=str(bin_path)))
        out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    elif pathlib.Path(DEFAULT_UPLOADER).exists():
        out = subprocess.run(
            [DEFAULT_UPLOADER, "upload-file", "--path", str(bin_path)],
            check=True, capture_output=True, text=True).stdout
    else:
        sys.exit(
            "No uploader found. Install/run this in a Muse workspace with "
            "remote-storage, or pass --upload-command 'COMMAND {path}' / set "
            "MUSECHAN_OTA_UPLOAD_COMMAND to a command that prints an HTTPS "
            "URL for the firmware binary.")

    last = out.strip().splitlines()[-1] if out.strip() else ""
    try:
        info = json.loads(last)
        return info["url"], info.get("expires_at")
    except Exception:
        if last.startswith("https://"):
            return last, None
        raise


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--board", choices=sorted(BOARDS), default="stackchan",
                    help="firmware target to build (default: stackchan)")
    ap.add_argument("--upload-command", default=os.environ.get("MUSECHAN_OTA_UPLOAD_COMMAND"),
                    help="upload command template containing {path}; defaults to Muse remote-storage when available")
    args = ap.parse_args()

    if not (ESP32 / "components" / "muse" / "avatar" / "muse_pixel.c").exists():
        sys.exit("No custom avatar yet: draw one first (see outfit-ota/README.md).")

    bin_path = ESP32 / BOARDS[args.board] / "muse-gadget.bin"
    now = datetime.now()
    version = f"{now:%Y.%m%d}.{now:%H%M}"
    original = VERSION_FILE.read_text() if VERSION_FILE.exists() else None
    try:
        VERSION_FILE.write_text(version + "\n")
        run(["tools/muse/board.sh", "build", args.board], cwd=ESP32)
    finally:
        if original is not None:
            VERSION_FILE.write_text(original)

    data = bin_path.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    url, expires_at = upload(bin_path, args.upload_command)

    pending = {
        "board": args.board,
        "version": version,
        "sha256": sha,
        "bytes": len(data),
        "url": url,
        "expires_at": expires_at,
        "built_at": now.isoformat(timespec="seconds"),
        "sent": False,
    }
    out_path = pending_path(args.board)
    out_path.write_text(json.dumps(pending, indent=2) + "\n")
    print(f"Staged OTA {version} for {args.board} ({len(data)} bytes, sha256 {sha[:12]}...)")
    print(f"Pending entry: {out_path}")
    print('Send it with device.ota {"url": <url>, "force": true} next time the device is online;')
    print("the invoke call will time out while the device reboots, which is expected.")


if __name__ == "__main__":
    main()
