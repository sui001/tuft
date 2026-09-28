# tuft

A tuft of grass made of servos: a grid of SG92Rs with long stainless rod
horns, swaying as one field. Part of the Cybernetic Garden machine roster
alongside Groundskeeper/Proty/Platty in `gen3d`.

The software idea is the "notwled" one. WLED stitches several LED matrices into
one virtual canvas and runs effects over it without knowing where the panel
boundaries are. Tuft does the same for servo patches: panel layout plus
effects-over-a-virtual-grid, with PWM angle as the output instead of RGB
brightness. Not a WLED fork, the output layer is too different for a fork to
buy anything.

This repo was called **polyp** until 28 Sep 2026 and was merged with the older
private tuft repo (the ceiling-arm concept, now in `arms/`). GitHub redirects
the old `sui001/polyp` URLs.

## Core model

- A **patch** = one PCA9685 board = up to 16 channels, arranged as rows x cols.
- Patches sit in a shared virtual grid (position + orientation), same as WLED's
  panel setup screen.
- **Effects** are `value = f(worldX, worldY, t)` and address the stitched grid,
  never a literal channel number. The layout layer is what turns
  `(worldX, worldY)` into `(I2C address, channel)`.

## Hardware

From v7.0 the servos run off a PCA9685, not direct GPIO. Direct GPIO (v3 to
v6.1) topped out at 20 channels and needed soldered underside pads, which a
replacement SuperMini doesn't have.

- PCA9685 at 0x40: SDA GPIO9, SCL GPIO10, VCC 3V3, OE unconnected.
- Servo power into the PCA9685's V+ screw terminal only, never the ESP's 5V
  pin. Ground shared with the ESP.
- SuperMini "VIN" is the 5V rail feeding a ~6V abs max regulator. 19V killed
  one board on 28 Sep 2026, keep the buck at 5.0V if it feeds the ESP.

## Contents

- `firmware/` — one folder per version, `tuft_<layout>_<effect>_v<X>_<Y>`.
  Latest is the highest version number.
- `visualizer/` — WebGL console: arrange patches, preview an effect as
  servo-paddle tilt.
- `index.html` at repo root mirrors `visualizer/tuft-console.html`, it's what
  GitHub Pages serves at sui001.github.io/tuft. Keep both in sync by hand.
- `arms/` — the earlier ceiling-mounted reaching-arm concept (a different
  machine, same name). Concept notes and its own console.

Repo is **public** (so the free GitHub Pages plan can serve the console). No
thesis/theory writing here, that belongs in `affective-devices/docs/`.

## Conventions

- No em dashes in anything written for Sui.
- Default ESP32 board is the SuperMini, GPIO 1-13 only, see gen3d memory
  `esp32s3-supermini-board`.
