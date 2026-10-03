# Avatar outfit updates over Wi-Fi

This directory is the operator kit for keeping a MuseChan (StackChan) or
bare CoreS3 gadget's face matched to the avatar of the Muse it is paired
with. It is written for the paired Muse as much as for the human: if you
are a Muse operating one of these devices, this is the page to follow.

The face is compiled into the firmware. There is no image file to swap on
the device, so an outfit change is always:

1. redraw the avatar renderer,
2. rebuild the firmware,
3. stage the firmware at an HTTPS URL,
4. send it to the device with the Muse gadget command `device.ota` when
   the device is online on Wi-Fi.

No USB cable is needed after the first flash. The build machine and the
device do not need to be on the same network; the device downloads the
firmware itself over its own Wi-Fi connection.

## What you need

- A Muse account and the Muse app.
- A gadget SDK token for your own Muse account (Muse app / developer
  settings). The token is baked into the firmware at build time. Never
  commit it, paste it into chat, or put it in this repo.
- A StackChan (for the robot body) or a bare M5Stack CoreS3 (display
  only build, no servos or pet reactions).
- ESP-IDF v6.0.1 on the build machine.
- This repo checked out on a machine your Muse can run commands on. A
  Muse cloud workspace works: it can build, upload the firmware image,
  and invoke the device command.

## First-time setup (human + Muse)

1. Build with your own SDK token in the gitignored build `sdkconfig`
   (`CONFIG_GADGET_SDK_TOKEN="mgst_..."`), then build and flash once over
   USB:

   ```sh
   cd esp32
   ./tools/muse/board.sh build stackchan   # or: build cores3
   ./tools/muse/board.sh flash stackchan   # or: flash cores3
   ```

2. Pair from the Muse app: turn on Developer mode, then Settings >
   Devices > Add Device, pick the `MuseGadget-...` device, press the
   side key on the device to confirm, and provision your Wi-Fi.
3. Draw your avatar onto the device with the SDK avatar flow
   (`esp32/tools/muse/avatar.py`). That saves
   `esp32/components/muse/avatar/muse_pixel.c` (gitignored; it is your
   Muse's face, so it stays local) and preview GIFs under
   `esp32/components/muse/avatar/gifs/`. Always look at the previews
   before sending firmware to the device.

## When your avatar changes (instructions for the paired Muse)

Run this every time your active avatar image changes: a morning outfit, a
night pajama look, a manual restyle, any `avatar.set`. The device should
wear what you wear.

1. Check whether anything actually changed. Hash your active avatar
   image and compare it with `avatar_sha256` in
   `outfit-ota/state.json` (create it from `state.example.json` the first
   time). Same hash and no unsent pending entry: stop, say nothing.
2. Redraw the device face from the new avatar with the SDK avatar flow
   (`esp32/tools/muse/avatar.py`, or `--reply` with a renderer you
   wrote). Keep the renderer's framework and expression system; change
   the look: palette, hat, jacket, shirt, shorts, shoes, accessories.
   Compare the generated `idle.gif` against your avatar image.
3. Stage the update:

   ```sh
   python3 outfit-ota/stage_outfit.py              # StackChan
   python3 outfit-ota/stage_outfit.py --board cores3
   ```

   This stamps a timestamped firmware version, builds, uploads the
   `.bin`, and writes `outfit-ota/pending.json` (or
   `pending-cores3.json`) with the download URL, SHA-256 and expiry.
   Hosted URLs expire (about two days with Muse storage), so send soon
   after staging. Update `state.json` with the new avatar hash and staged
   version.
4. Send it when the device is online. Invoke the gadget command
   `device.ota` with `{"url": "<pending url>", "force": true}` (`force`
   is needed while the firmware version is a development stamp). The
   invoke call timing out after about 30 seconds is the expected shape
   of success: the device is downloading (about 2 MB), rebooting, and
   reconnecting. Verify afterwards with `device.discover` or
   `device.health`, then mark the pending entry `sent: true` and record
   `last_sent_version` in `state.json`.
5. If the device is offline, leave the pending entry queued. There is no
   queue on the device itself; `pending.json` on your side is the queue.
   Send it at your next check when the device is connected.

## Live expressions (no rebuild needed)

Outfits are firmware; expressions are instant. The StackChan build
advertises two extra device commands (see `MUSECHAN.md`):

- `stackchan.face` with `{"face": "thinking"}`: use `thinking` while you
  are working on a reply, `speaking` while you talk, `idle` when done.
  Also `listening`, `happy`, `error`, `boot`, `off`.
- `stackchan.look` with `{"yaw": -128..128, "pitch": 0..90}` aims the
  head. StackChan only; the CoreS3-only build has no head to aim.

Mirror your own state as it changes, the same way your avatar in the
chat header does: working, waiting, speaking, idle. The CoreS3-only build
does not advertise the `stackchan.*` commands; its expressions are driven
by the conversation itself.

## Automating it

The pattern used by the reference setup is two scheduled checks owned by
the same goal/schedule as the avatar outfit change:

- one in the morning, after the outfit refresh, and
- one at night, for the pajama change.

Each run does exactly the steps above: hash compare, send any queued
pending OTA if the device is online, redraw + stage + send if the avatar
changed, and stay silent when nothing changed. Name the device, the state
file and the pending file in the job instructions so any future Muse on
duty can pick it up cold. A worked example of the job text lives with the
operator's own scheduler, not in this repo, because schedules are
per-account.

## Safety and secrets

- Never commit: SDK tokens, Wi-Fi names or passwords, device serials or
  MAC addresses, your `state.json`, `pending*.json`, or your custom
  `muse_pixel.c`. The repo's `.gitignore` already covers the operator
  files; check `git status` before every push.
- `esp32/dev_signing_key.pem` is upstream's sample development key from
  the initial SDK import. It is not your key and not a credential for
  your Muse account; release builds should use your own signing setup.
- A bad image cannot brick the device: the firmware keeps two app slots
  and rolls back if the new image does not come up, and USB flashing
  remains the fallback.
