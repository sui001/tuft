# tuft

A tuft of grass made of servos. Servo patches arranged into a shared grid,
with effects (ripple, sweep, chase, wind, dominoes, breeze) running across the
stitched space the same way WLED's 2D panel setup does for LED matrices. See
[CLAUDE.md](CLAUDE.md) for the full concept.

Previously called **polyp**; old links redirect here. The earlier
ceiling-mounted reaching-arm concept that also went by Tuft lives in
[`arms/`](arms/).

## Build

4x4 grid of SG92R servos with 1100mm x 2mm stainless welding-rod horns,
running the `breeze` effect: continuous, spatially-correlated sway that
crosses the grid rather than each stalk twitching independently.

<img src="docs/images/tuft-4x4-rods.jpg" width="420" alt="4x4 SG92R panel with 1100mm stainless rod horns fanning out">
<img src="docs/images/tuft-4x4-servos.jpg" width="420" alt="Close-up of the 4x4 SG92R grid on cardboard, wiring converging to the SuperMini">

Up to v6.1 the servos ran on direct GPIO from an ESP32-S3 SuperMini (LEDC +
MCPWM, 20 channels max), shown here at bring-up:

<img src="docs/images/tuft-controller-wiring.jpg" width="420" alt="ESP32-S3 SuperMini and breadboard wiring for the 16-channel controller">

From v7.0 they run off a PCA9685 over I2C (SDA GPIO9, SCL GPIO10), which frees
the ESP's pins and scales by adding boards.

Firmware lives in [`firmware/`](firmware/), one folder per version, see its
history for the bring-up (10-bit vs 14-bit PWM resolution, the S3's 8-channel
LEDC limit, extending to MCPWM, then moving to the PCA9685).

## Console

**[sui001.github.io/tuft →](https://sui001.github.io/tuft/)**

Arrange boards, tune each servo (type, horn length/thickness/orientation,
mount tilt/pan), pick wall/floor/roof mounting aspect, run a directional
effect, then export the whole configuration as JSON, enough to hand to real
PCA9685 firmware.

`index.html` at the repo root is what Pages serves, keep it in sync with
`visualizer/tuft-console.html` when the console changes.
