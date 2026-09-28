# tuft

Ceiling-mounted cluster of reaching arms, working title Tuft. One center arm, rings
of followers around it, feeding/mouth metaphor: a ToF sensor at the center triggers
a reach toward motion below, followers echo it with falloff by ring. Part of the
Cybernetic Garden PhD machine-creature roster, alongside Groundskeeper/Proty/Platty
in `gen3d`. Full concept detail lives in `gen3d/groundskeeper/CONCEPT.md`, not
duplicated here.

Two hardware tracks in this repo, likely doing different jobs:

- `servo/` — the reaching arms themselves. Same mechanical family as
  [genarm](https://github.com/sui001/genarm) (CNC shield + TMC2209-class drivers
  is the genarm pattern, but the arm links here are RC servo driven, not stepper).
- `stepper/` — whatever in Tuft needs stepper precision instead (candidate:
  a shared rotating/traversing base, not yet decided).
- `visualizer/` — WebGL concept console, simulates the reach/echo/fold-back
  behavior before any hardware exists. Not calibrated to real geometry yet.

This was the whole of the private tuft repo until 28 Sep 2026, when it was
merged into the (public) servo-grid repo. The `servo/` and `stepper/` folders
above were never created. No thesis/theory writing here, that belongs in
`affective-devices/docs/`.

## Conventions

- No em dashes in anything written for Sui.
- Default board for any ESP32 work is the SuperMini, GPIO 1-13 only (see gen3d
  memory `esp32s3-supermini-board`), not a generic S3 pinout.
