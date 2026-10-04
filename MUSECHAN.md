# MuseChan: Muse on a StackChan (and CoreS3)

MuseChan puts Meta's [Muse gadget SDK](https://github.com/facebookincubator/muse-gadget-sdk)
on the [M5Stack StackChan](https://docs.m5stack.com/en/stackchan), the little
desktop robot built around an M5Stack CoreS3 controller. Upstream already
supports M5Stack's StickS3 and StickC Plus2; this fork adds the CoreS3 board
in two builds, two StackChan device commands, and a set of pet reactions, so
Muse can move the robot's head, mirror its live state on the robot's face,
and keep the robot's outfit matched to its own avatar over Wi-Fi.

Everything here rides on the upstream firmware: the avatar face, push-to-talk
voice, settings UI, image display, Wi-Fi and OTA all work the way they do on
the other full-UI boards. Your gadget pairs with the Muse app over Bluetooth,
joins your Wi-Fi, and holds an encrypted session to your Muse.

**Status:** hardware-verified on a StackChan unit. The CoreS3 display, touch,
audio (AW88298 speaker amp + ES7210 mics), AXP2101 power key as the talk
button, Wi-Fi pairing, `stackchan.face`, `stackchan.look`, optional text
replies, the pending-reply base LEDs, and OTA firmware updates all work on
the real robot.

## The two builds

| Build | Command | For | What's in it |
|---|---|---|---|
| StackChan | `./tools/muse/board.sh build stackchan` | A full StackChan robot | CoreS3 face/voice/UI plus head servos, pet reactions, and the `stackchan.face` / `stackchan.look` commands. |
| CoreS3 only | `./tools/muse/board.sh build cores3` | A bare M5Stack CoreS3, no robot base | The same screen, touch, speaker, mics and talk key, with the StackChan harness compiled out: no servo driver, no pet reactions, no `stackchan.*` commands. Expressions still follow the conversation through the SDK's own state loop. |

If you only have the CoreS3 controller, use the `cores3` build. The StackChan
build also runs on a bare CoreS3 (the head calls report "not found"), but the
CoreS3-only build is smaller and has none of the robot-body behaviour to
reason about.

## What was added

- **CoreS3 board driver** (`esp32/components/muse/boards/board_m5stack_stackchan.c`,
  shared by both builds; overlays `esp32/devices/sdkconfig.muse-m5stack-stackchan`
  and `esp32/devices/sdkconfig.muse-m5stack-cores3`). ILI9342C/E panel probe,
  AW9523 expander for LCD/touch reset and amp enable, AXP2101 rails and power
  key, FT5x06-family touch, light sleep. The CoreS3's PSRAM is quad, not
  octal: `CONFIG_SPIRAM_MODE_QUAD=y`.
- **Head servos** (`stackchan_head.c`, StackChan build only). The base's two
  Feetech SCSCL servos (yaw ±128°, pitch 0–90°), powered through its M5IOE1
  expander, including the SCSCL checksum byte the servos require.
- **`stackchan.look` device command.** `{"yaw": -40, "pitch": 20}` aims the
  head. **`stackchan.face`** sets the face: `{"face": "thinking"}` (also
  `idle`, `listening`, `speaking`, `error`, `boot`, `off`, `happy`). Both
  are advertised in the device command catalog, so a paired Muse can find
  them with `device.discover`.
- **Pet reactions** (`stackchan_pet.c`, StackChan build only). The CoreS3's
  BMI270 IMU, the head's Si12T touch sensor and the LTR-553 light/proximity
  sensor feed the SDK's own pet state: a shake, tap or pickup makes the face
  happy and keeps Muse awake (with a little head movement), a pat on the
  head pets it, and putting the robot face-down for a moment puts it to
  sleep until it is picked back up.
- **Text replies** (`CONFIG_MUSE_TEXT_REPLIES`, off by default). While the
  upstream gadget speech pipeline is unfinished, push-to-talk turns can be
  posted as text and replies shown on the face as captions instead of being
  fetched as speech. Enable it in the build directory's `sdkconfig` until
  spoken replies work, then turn it off again.
- **Pending-reply base LEDs** (`CONFIG_MUSE_STACKCHAN_PENDING_LED`, off by
  default, StackChan build only). While a reply has been shown and not yet
  acknowledged, the twelve base LEDs flash amber, half a second on and half
  a second off. Starting the next talk clears it; so do a tap on the face
  and a double pat on the head.
- **Outfit updates over Wi-Fi** (`outfit-ota/`). The avatar face is compiled
  into the firmware, so an outfit change is a redraw, a rebuild and an OTA
  update, sent the next time the gadget is online. See
  [outfit-ota/README.md](outfit-ota/README.md) for the full operator guide,
  including the instructions a paired Muse follows when its avatar
  changes.

## Build, flash, pair

You need ESP-IDF v6.0.1 and your own Muse gadget SDK token (`mgst_...`)
from gadgets.muse.ai (Account > SDK tokens), as described in the upstream
README. This repository intentionally ships without any token. Put yours in
the generated build directory's gitignored `sdkconfig` as
`CONFIG_GADGET_SDK_TOKEN="mgst_..."` (or set it with `idf.py menuconfig` for
that build directory), then build. Never commit it: a token identifies your
Muse account, so treat it like a credential and never paste it into chat,
issues, or docs.

```sh
cd esp32
./tools/muse/board.sh build stackchan   # or: build cores3
./tools/muse/board.sh flash stackchan   # first flash only; later updates go over Wi-Fi
```

Before the first flash, back up the factory firmware so you can always go
back. Pairing is unchanged from upstream: in the Muse app turn on Developer
mode, then Settings > Devices > Add Device, pick the `MuseGadget-...`
device, press the side key to confirm, and provision your Wi-Fi. To start
over (for example to hand the robot to another Muse), hold the side key
for about 5 seconds to reset setup; that unpairs and forgets Wi-Fi without
erasing the firmware or the face.

## How the face works

The SDK's Muse loop already drives the avatar through idle, listening,
thinking, speaking and error states as a conversation happens, so the face
reflects what Muse is doing live. `stackchan.face` pokes the same state
from outside a conversation: a Muse sets `thinking` while it works on a
reply, `speaking` while it talks, and `idle` when it is done, mirroring the
state its own avatar shows in the chat header. Outfits are the compiled-in
renderer underneath those states; changing them is the OTA flow in
`outfit-ota/`.

## Hardware notes

- Two panel revisions ship in CoreS3s. The driver probes before the panel
  exists and logs `panel probe DD=... CB=... -> ILI9342C/E`. The C is the
  fallback, as in M5GFX.
- The FT6336U touch controller's reset is AW9523 P0_0; the board holds it
  released after display start or touch never answers.
- LVGL draw buffers stay at 16 lines: larger strips starve the I2S DMA and
  audio breaks up.
- Servo motion is open-loop (position writes only), with the BSP's default
  centre values (yaw raw 460, pitch raw 620). A unit whose servos were
  re-zeroed by other firmware may sit off-centre until calibration is added.
- Sensors wired into the pet task: BMI270 IMU, Si12T head touch and
  LTR-553 light/proximity. The pet task logs the gravity vector on every
  event (`shake`, `picked up`, `face down`, `face up`), which is how the
  thresholds get tuned on a real unit.

## Not yet

- **Camera capture is parked.** A GC0308 driver and `camera.capture`
  command are included behind `CONFIG_MUSE_STACKCHAN_CAMERA` (off by
  default), but captured frames are not usable yet. Do not enable it
  expecting working photos.
- **Servo calibration and feedback** (the BSP stores per-unit zeros in NVS;
  reads of present position are not implemented).
- General-purpose base RGB LED control (the base LEDs are currently used
  only as the pending-reply indicator), IR and NFC.

## Credits and license

This is a fork of Meta's Muse gadget SDK (Apache License 2.0; the upstream
`LICENSE` and per-file notices are unchanged, and the additions carry the
same license). Hardware support is derived from M5Stack's open sources:
[M5GFX](https://github.com/m5stack/M5GFX),
[M5Unified](https://github.com/m5stack/M5Unified) and the
[StackChan BSP](https://github.com/m5stack/StackChan-BSP). StackChan is a
community co-creation of M5Stack and the StackChan community.
