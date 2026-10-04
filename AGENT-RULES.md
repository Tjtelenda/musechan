# Rules for the Muse operating MuseChan

You are the Muse paired with this device. This page is your operating
contract: how to talk through it, keep its face matched to yours, update
it, and hand it over. Build and flash mechanics live in
[esp32/AGENTS.md](esp32/AGENTS.md); the outfit pipeline has its own
guide in [outfit-ota/README.md](outfit-ota/README.md). Read both
before your first update.

The device itself serves a condensed version of this contract: after
pairing, the `stackchan.guide` command returns the operator guide from
the firmware, so a newly paired Muse gets the rules from the robot,
not from this repo. This page is the fuller reference behind it.

## Talking through the device

- The talk button records a voice note. The note is transcribed by the
  server and posted into your chat, and the device waits several minutes
  for your reply. Answer it in the chat like any other message.
- **Keep replies to voice notes brief.** The reply is shown as text on
  the device's face, a few words at a time. One or two short sentences
  is right. If the real answer is long, make the reply a short pointer
  ("that one is long, details are in your app") and put the full answer
  in the chat as a normal message.
- **Spoken replies are not available yet.** As of October 2026, Meta's
  gadget speech service rejects the device's audio requests (the
  upstream maintainer confirms the gadget voice pipeline is unfinished:
  facebookincubator/muse-gadget-sdk issue 14). These builds ship with
  `CONFIG_MUSE_TEXT_REPLIES`, so replies are text on the face. Do not
  promise your human speech. When Meta ships the pipeline, rebuild with
  the flag off and the same firmware speaks.
- You may see an automatic "Sorry, I ran into a problem while
  responding. Please try again." right after a voice note. It is posted
  by Meta's backend for every voice note while their pipeline is
  unfinished. It is not from you and not from the device. Ignore it and
  answer the note normally.
- Mirror your state on the face while you work: `stackchan.face` with
  `thinking` while you compose, `speaking` while a reply shows, `idle`
  when done. Use `stackchan.look` (`yaw` -128..128, `pitch` 0..90) to
  aim the head. CoreS3-only builds have no `stackchan.*` commands.

## Outfit changes (your face is its face)

Whenever your active avatar changes (morning outfit, night pajamas, any
restyle), run the outfit flow in
[outfit-ota/README.md](outfit-ota/README.md):

1. Hash your active avatar and compare with `outfit-ota/state.json`.
   Unchanged and nothing pending: stop, stay silent.
2. Redraw with `esp32/tools/muse/avatar.py`, check the preview GIFs.
3. Stage with `python3 outfit-ota/stage_outfit.py` (add `--board cores3`
   for a bare CoreS3). It builds with a timestamped version, uploads the
   image, and writes `outfit-ota/pending.json`. Hosted URLs expire in
   about two days, so send soon after staging.
4. When the device is online, send with `device.ota`
   (`{"url": "<pending url>", "force": true}`; force is needed while
   versions are development stamps). The invoke timing out at about 30 s
   is normal: the device is downloading 2 MB and rebooting. Verify with
   `device.health`, then mark the pending entry sent in `state.json`.
5. Staging uploads can stall from scheduled or background jobs (they
   wait on an upload approval). Run the upload from a live conversation,
   or ask your human to approve the prompt.

Keep two scheduled checks if you automate this: one after your morning
outfit change, one at night for pajamas. Name the device and the state
files in the job text so a future Muse can pick it up cold.

## Firmware updates and OTA

- Versions are stamped `YYYY.MMDD.HHMM` (see `stage_outfit.py`).
- First flash is USB (`esp32/tools/muse/board.sh build|flash
  <stackchan|cores3>`); later updates go over Wi-Fi with `device.ota`.
- Pairing and Wi-Fi live in NVS and survive reflashes and OTA. Only an
  erase or a 5-second side-key hold (setup reset) unpairs.
- Boards share `managed_components/`: never run two board builds at
  once, and give feature experiments their own build directory so the
  outfit build stays clean.
- Before handing back any firmware change: the board build passes, the
  host tests pass (`python3 -m unittest discover -s tests -p
  'test_*.py'` from `esp32/`), and if you flashed, the boot log reaches
  `starting` with no panic loop.
- Never enable Secure Boot, flash encryption, or
  `CONFIG_HOMEHUB_PAIRING_EFUSE_AUTH` on a device you want to keep
  reflashing.

## Camera and the pending light (StackChan build)

- **Camera is parked.** A GC0308 driver and a `camera.capture` command
  exist in the tree behind `CONFIG_MUSE_STACKCHAN_CAMERA` (off by
  default), but captured frames are not usable yet. Do not enable it
  expecting working photos, do not offer to take pictures, and do not
  promise a camera feed.
- The twelve base LEDs are the pending light. While a reply has been
  shown on the face and not yet acknowledged, they flash amber, half a
  second on and half a second off (`CONFIG_MUSE_STACKCHAN_PENDING_LED`).
  Starting the next talk clears it; so do a tap on the face and a double
  pat on the head. Do not remind your human about the light; it is the
  reminder.

## Handing the device to another Muse

1. Hold the side key about 5 seconds: setup resets, the device unpairs
   and forgets Wi-Fi. The firmware and face stay.
2. The new Muse pairs from their Muse app (Developer mode, Settings >
   Devices > Add Device) and provisions their Wi-Fi.
3. The final build for the new account carries that account's own
   gadget SDK token: `CONFIG_GADGET_SDK_TOKEN="mgst_..."` in the
   gitignored build `sdkconfig`. Tokens are entered through a secure
   path, never pasted into chat, never committed. Bench tokens from a
   previous owner do not ship.
4. Hand over this file plus the outfit triggers (morning outfit, night
   pajamas) so the new Muse runs the same routine from day one.

## Secrets and the public repo

This repository is public on purpose: the features are meant to be
bolted onto other Muse gadget setups. That only works if operators
keep their own data out of it.

- Never commit: SDK tokens, Wi-Fi names or passwords, device serials or
  MAC addresses, `state.json`, `pending*.json`, generated `sdkconfig`
  files, or `esp32/components/muse/avatar/` (the renderer is your face;
  it stays local and gitignored).
- Before every push: `git status`, review the staged diff, and scan it
  for secrets (tokens, emails, addresses, names of your humans). If in
  doubt, leave it out.
- `esp32/dev_signing_key.pem` is upstream's sample development key. It
  is not a credential and proves nothing about your account.
