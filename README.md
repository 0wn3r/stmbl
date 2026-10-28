[![Join the chat at https://gitter.im/rene-dev/stmbl](https://badges.gitter.im/rene-dev/stmbl.svg)](https://gitter.im/rene-dev/stmbl?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

DISCLAIMER
===

THE AUTHORS OF THIS SOFTWARE ACCEPT ABSOLUTELY NO LIABILITY FOR
ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
harming persons must have provisions for completely removing power
from all motors, etc, before persons enter any danger area.  All
machinery must be designed to comply with local and national safety
codes, and the authors of this software can not, and do not, take
any responsibility for such compliance.

This software is released under the GPLv3.

STMBL
=====

Servo drive firmware for the STMBL V4.0 board with the IRAM256-2067A power module. This tree is a fork of [STMBL](https://github.com/rene-dev/stmbl), an open-source servo drive designed for retrofitting CNC machines and robots. It supports industrial AC and DC servos.

The F3 image in this tree is built for the V4.0 power stage only: IRAM256-2067A (20 A rms at Tc 25 C, 30 A peak), its FLT/EN driver enable (high = run), its 47k NTC on a separate pin and 3 mOhm shunts. It is not compatible with boards that use other power modules.

- Hardware: [hw/kicad/v4.0](hw/kicad/v4.0)
- Documentation: [docs/stmbldoc](docs/stmbldoc/content/docs/), start with [Supply](docs/stmbldoc/content/docs/supply.md)
- Building and flashing: [docs/procedures](docs/procedures/)
