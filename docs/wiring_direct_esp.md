# Direct ESP wiring (v8, perfboard, no PCA9685)

Decided 9 Oct 2026. The PCA9685 rig (v7.x) went jittery past ~12 servos
moving at once, whatever the pattern; pressing the lid didn't help and the
buck held 5.21V on a meter. Rebuilt on perfboard, servos driven straight
from the ESP32-S3 SuperMini.

## Pin map

Servo N is numbered bottom-left first, as before:

```
 13 14 15 16    <- top row
  9 10 11 12
  5  6  7  8
  1  2  3  4    <- bottom row
```

| Servo | GPIO | Where on the SuperMini |
|---|---|---|
| 1 | 1 | left header |
| 2-6 | 3-7 | left header (GPIO2 skipped, board-specific boot trouble) |
| 7-12 | 8-13 | right header, runs **up** from the bottom (horseshoe) |
| 13 | 43 | left header, marked TX |
| 14 | 44 | left header, marked RX |
| 15 | 14 | underside pad, soldered |
| 16 | 15 | underside pad, soldered |

Servos 1-12 keep v6's rule (servo N -> GPIO N+1 after servo 1). v6 put
13-16 on underside pads 14-17; this map moves 13 and 14 onto TX/RX so only
two pads are needed.

## TX / RX as servo pins

Fine because USB CDC On Boot is enabled: Serial and flashing run over
native USB (GPIO 19/20), so 43/44 are ordinary GPIOs. Two costs:

- The boot ROM prints to GPIO43 at power-on, before the sketch runs. Servo
  13 may twitch once on boot. Unavoidable without burning eFuses.
- No hardware UART left on spare pins (none are spare anyway).

Not tested on hardware before the build: v6.1 used GPIO43 but was never
flashed.

## Perfboard

Three straight rows of header pins, 16 columns, same order as a PCA9685:

- **signal** row: each pin wired to its GPIO above
- **V+** row: bussed along the back with thick wire to the buck's +5V
- **GND** row: bussed to the buck's GND, and to the SuperMini's GND

Servo plug: brown = GND, red = V+, orange = signal.

Servo power never comes from the SuperMini's 5V pin. The SuperMini's VIN/5V
is fed separately (USB or the buck at 5.0V; 19V killed one board).

Underside pads: thin (28-30AWG) wire, tin both, a second of heat, then hot
glue or Kapton as strain relief straight away. Pads die from being tugged.

## Firmware notes for v8

- 8 LEDC + 8 MCPWM channels, as v6 (LEDC is capped at 8 on the S3).
- Stagger pulse starts across the 20ms frame. LEDC and MCPWM, like the
  PCA9685, otherwise start every pulse at the same instant, so all 16
  servos' current bursts stack.
- Sui's good range from the tuner: +-9-12 deg sway over a 4-5 s period.
  Tiny sways (+-1-3 deg) look dead.
