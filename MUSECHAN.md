# MuseChan: Muse on a StackChan

MuseChan puts Meta's [Muse gadget SDK](https://github.com/facebookincubator/muse-gadget-sdk)
on the [M5Stack StackChan](https://docs.m5stack.com/en/stackchan), the little
desktop robot built around the CoreS3 controller. Upstream already supports
M5Stack's StickS3 and StickC Plus2; StackChan support is what this fork adds,
plus two device commands and a set of pet reactions, so Muse can move the
robot's head and the robot can react to being handled.

Everything here rides on the upstream firmware: the avatar face, push-to-talk
voice, settings UI, image display, Wi-Fi and OTA all work the way they do on
the other full-UI boards. Your StackChan pairs with the Muse app over
Bluetooth, joins your Wi-Fi, and holds an encrypted session to Muse.

**Status:** the port builds. It has not been run on hardware yet, so treat
the [bring-up notes](#first-run-on-hardware) below as the checklist for the
first flash.

## What was added

- **CoreS3 / StackChan board** (`esp32/components/muse/boards/board_m5stack_stackchan.c`,
  `esp32/devices/sdkconfig.muse-m5stack-stackchan`, board alias `stackchan`).
  Display, touch, speaker and microphones, battery, the side power key as
  the talk button, and light sleep. The same build runs on a bare CoreS3.
- **Head servos** (`stackchan_head.c`). The base's two Feetech SCSCL servos
  (yaw ±128°, pitch 0–90°), powered through its M5IOE1 expander. Absent on a
  bare CoreS3, where every head call quietly reports "not found".
- **`stackchan.look` device command.** `{"yaw": -40, "pitch": 20}` aims the
  head. **`stackchan.face`** sets the face: `{"face": "thinking"}` (also
  `idle`, `listening`, `speaking`, `error`, `boot`, `off`, `happy`).
- **Pet reactions** (`stackchan_pet.c`). The CoreS3's BMI270 IMU and the
  head's Si12T touch sensor feed the SDK's own pet state: a shake, tap or
  pickup makes the face happy and keeps Muse awake (with a little head
  movement when the base is attached), a pat on the head pets it, and
  putting the robot face-down for a moment puts it to sleep until it is
  picked back up.

## Build

You need ESP-IDF v6.0.1 and a Muse gadget SDK token (`mgst_...`) from the
Muse app, as described in the upstream README. Put the token in the build
directory's `sdkconfig` as `CONFIG_GADGET_SDK_TOKEN="mgst_..."` before
building, then:

```sh
cd esp32
./tools/muse/board.sh build stackchan
```

This produces `build-muse-m5stack-stackchan/`. Flashing, when you are ready
(and have backed up the factory firmware), is
`./tools/muse/board.sh flash stackchan`. Nothing about this fork changes
pairing: press the side key to confirm, on a trusted network, as upstream
documents.

## How the face works

There is no custom face here. The SDK's Muse loop already drives the
avatar through idle, listening, thinking, speaking and error states as a
conversation happens, so the face reflects what Muse is doing live.
`stackchan.face` pokes the same state from outside a conversation (useful
for an agent that wants the robot to look thoughtful while it works), and
the pet reactions call the same `muse_state_make_happy()` the UI calls when
you tap the face.

## First run on hardware

The board code was written from M5's published sources (M5GFX, M5Unified,
the StackChan BSP) rather than probed on a unit, so the first boot log is
part of the bring-up:

- `panel probe DD=... CB=... -> ILI9342C/E` — two panel revisions ship in
  CoreS3s. If yours logs "neither key answered, ILI9342C assumed" and the
  picture looks wrong, the raw values in that line are what is needed to
  add its alignment.
- `BMI270 ready` / `Si12T head touch ready` — sensor presence. The pet task
  logs the gravity vector on every event (`shake`, `picked up`,
  `face down`, `face up`), which is how the shake/pickup thresholds and the
  face-down Z sign get tuned for a real unit.
- Servo motion is open-loop (position writes only), with the BSP's default
  centre values (yaw raw 460, pitch raw 620). A unit whose servos were
  re-zeroed by other firmware may sit off-centre until calibration is added.

## Not yet

- **Camera.** The CoreS3 has a GC0308 camera, but M5 does not publish its
  data pins in any source we could find, so `camera.capture` is not offered
  rather than guessed at.
- **Servo calibration and feedback** (the BSP stores per-unit zeros in NVS;
  reads of present position are not implemented).
- The base's RGB LEDs, IR and NFC.

## Credits and license

This is a fork of Meta's Muse gadget SDK (Apache License 2.0; the upstream
`LICENSE` and per-file notices are unchanged, and the additions carry the
same license). Hardware support is derived from M5Stack's open sources:
[M5GFX](https://github.com/m5stack/M5GFX),
[M5Unified](https://github.com/m5stack/M5Unified) and the
[StackChan BSP](https://github.com/m5stack/StackChan-BSP). StackChan is a
community co-creation of M5Stack and the StackChan community.
