# Atari G42 for MiSTer FPGA

A work-in-progress FPGA implementation of Atari Games' **G42** arcade
hardware, the board under **Road Riot 4WD**, **Guardians of the 'Hood** and
**Danger Express**, written for the [MiSTer](https://github.com/MiSTer-devel)
platform.

> **This core was made with AI.** The RTL, testbenches, reference models and
> documentation were written with an AI assistant (Claude, by Anthropic),
> working from MAME's source and the games' own test screens. Every block was
> checked in simulation against a MAME-derived reference, and the whole core
> was run in a full-system simulation against MAME. It has **not yet been run
> on a DE10-Nano**. This is disclosed so you can decide whether to use it.

---

## Games

| Game | Year | Players | ROM sets | Status |
|---|---|---|---|---|
| Road Riot 4WD | 1991 | 1 | roadriot (04 Dec 1991, conversion kit); roadriota (13 Nov 1991, conversion kit); roadriotb (04 Jun 1991, dedicated twin) | **Runs in simulation** (roadriot): start-up tests, title, attract race, coins and start match MAME; 20 of 22 compared frames pixel-identical. Not yet run on hardware. The two older sets are checked as MRAs only. |
| Guardians of the 'Hood | 1992 | 3 | guardian | **Runs in simulation**: attract, title and player select after a coin match MAME; 9 of 15 compared frames pixel-identical, the rest the same screens 2–5 frames apart. Not yet run on hardware. |
| Danger Express | 1992 (prototype) | 2 | dangerex | **Runs in simulation**: attract, title, coin and jump match MAME; 16 of 19 compared frames pixel-identical, the rest one frame apart. Not yet run on hardware. |

All five sets are named as in MAME 0.264 and load from merged, split or
non-merged ROM sets.

## Progress

```
MiSTer integration     ███████████████░░░░░   75%
ROM loading (.mra)     ██████████████████░░   90%
CPU (68000 + SLOOP)    ██████████████████░░   90%
ASIC65 coprocessor     █████████████████░░░   85%
Memory subsystem       ███████████████░░░░░   75%
Video                  █████████████████░░░   85%
Sound (JSA III)        █████████████████░░░   85%
I/O and controls       ████████████████░░░░   80%
                       ────────────────────
Project                █████████████████░░░   83%
```

Everything is implemented and matches MAME in simulation; nothing is yet
confirmed on a DE10-Nano, and the Quartus build is still being brought up
(block RAM is nearly full).

## The hardware

Atari G42 is the successor of Atari G1: a 68000 with Atari's RLE "growth
renderer" for scaled motion objects, a scrolling playfield and a text layer,
the ASIC65 maths coprocessor, the SLOOP program-ROM bank controller and the
separate JSA III sound board. Board photographs and chip lists are at
[System 16 — Atari G42 Hardware](https://www.system16.com/hardware.php?id=774).

| | |
|---|---|
| Main CPU | Motorola 68000 @ 14.318181 MHz |
| Protection | SLOOP: an 8 KB bank window at $078000, switched by address sequences (Road Riot and Guardians; not on Danger Express) |
| Coprocessor | ASIC65: a TMS32015 DSP running its own program on Road Riot; Guardians and Danger Express use its fixed commands |
| Sound | JSA III: 6502 @ 1.789773 MHz, YM2151 @ 3.579545 MHz, OKI6295 @ 1.193182 MHz with banked samples, mono |
| Video | 336 × 240 visible in a 456 × 262 raster, 59.923 Hz |
| Layers | 128 × 64 playfield of 8 × 8 tiles, 6 bpp, per-line scroll; 64 × 32 text layer; RLE motion objects, scaled, double-buffered |
| Palette | 2,048 entries, IRGB-1555 |
| Settings | 2 KB EEPROM (2816) behind an unlock; no DIP switches |
| Road Riot | ADC0809 for the wheel and pedal |

Where the core differs from MAME, it follows the board: the sound CPU sees
the test switch, and the ASIC65's data RAM is the real TMS320C15's 256 words.
The 68000 reads its program through a 16 KB cache, so its ROM reads run
without wait states, as they do from the board's EPROMs.

---

## Using the core

ROMs are not distributed with this core. Put the `.mra` files from `mra/`
(all five sets) or `releases/` (the three main sets) in `_Arcade`, the core
(`Arcade-Atari-G42.rbf`, optionally renamed `Arcade-Atari-G42_<date>.rbf`)
in `_Arcade/cores`, and `roadriot.zip`, `guardian.zip` and `dangerex.zip`
in `/games/mame/`. Any MiSTer SDRAM module is enough. The first boot after a
load takes one to two seconds longer than the game's own, while the core
checks the SDRAM and indexes the motion-object ROM.

To build the core, open `Arcade-Atari-G42.qpf` in Quartus Prime 17.0 Lite and
compile; the result is `output_files/Arcade-Atari-G42.rbf`.

### Controls

**Road Riot 4WD**: the **left analog stick** is the steering wheel (the d-pad
and a paddle also steer); the gas pedal is **R** or the **right stick** pushed
up. Triggers **A** and **B**, **Start**, **Coin** Select.

**Guardians of the 'Hood**: stick, **Punch** Y and X, **Kick** B and A,
**Defend** R (also Start), **Coin** Select. It ships set up for 2 players;
choose 3-PLAYER in its game options for the third.

**Danger Express**: stick, **Fire** A, **Jump** B (also Start), **Duck** X,
**Coin** Select.

### Settings

Coinage, difficulty and the other operator settings are in each game's own
service menu: turn on **OSD → Service Menu** and reset, change the settings,
then turn it off and reset to play. They are kept in the game's EEPROM,
which the core saves to the SD card. Road Riot's wheel and pedal calibration
is in its switch test.

The OSD has:

* **Aspect ratio**, **Scandoubler Fx** and **Scale**: MiSTer's standard
  options.
* **[CRT Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)**: the
  picture's width and position on a 15 kHz CRT (H-Size, H-Position,
  V-Shift). It switches itself off while a Scandoubler Fx option is on, and
  the HDMI picture follows the adjustment too.
* **Service Menu**: the game's own test menu, from the next reset.
* **Controls** (Road Riot only): wheel sensitivity for an analog stick.
* **Debug**: a diagnostic overlay, layer switches and SDRAM timing options,
  for diagnosis only.

---

## Credits and references

This core is a reimplementation. It would not have been possible without:

**[MAME](https://www.mamedev.org/)**, the reference for the hardware's
behaviour: `atarig42.cpp`, `atarig42_v.cpp` (Aaron Giles), `atarirle.cpp`,
`asic65.cpp`, `atarijsa.cpp`, `eeprompar.cpp`, `adc0808.cpp`. No MAME code is
compiled into the core.

**[MiSTer-CRT-Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)** by
Umberto Parisi (**rmonic79**), with Andrea Bogazzi (**asturur**): the CRT
geometry module in `rtl/crt/`, used unmodified.

**[fx68k](https://github.com/ijor/fx68k)** by Jorge Cwik: the 68000.

**T65** by Daniel Wallner, Mike Johnson, Wolfgang Scherr and Morten Leikvoll:
the JSA III's 6502.

**[JT51 and JT6295](https://github.com/jotego)** by Jose Tejada (**jotego**):
the YM2151 and OKI6295.

**[Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer)** by
Alexey Melnikov (**Sorgelig**): the framework in `sys/`, unmodified, and the
project template. MiSTer's `mra_loader.cpp` is the reference for the MRA
checks.

The **Atari G1 core** (Gm0rk) is the base of this one's SDRAM controller,
self-test, clock scheme and overlay.

## License

The core's own RTL is released under the GNU General Public License v2.0 or
later (`LICENSE`), like the MiSTer framework it builds on. `rtl/crt/` (CRT
Adjust) is GPL-3.0 (`rtl/crt/LICENSE`); a built core that includes it is
therefore distributed under GPL-3.0. `sys/`, fx68k, T65, JT51 and JT6295
keep their own licences and authorship.

No ROM data is included or distributed.
