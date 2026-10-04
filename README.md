# Atari G42 for MiSTer FPGA

A work-in-progress FPGA implementation of Atari Games' **G42** arcade
hardware, the board under **Road Riot 4WD**, **Guardians of the 'Hood** and
**Danger Express**, written for the [MiSTer](https://github.com/MiSTer-devel)
platform.

**All three games are playable on a DE10-Nano, with sound.** See
[Games](#games).

> **This core was made with AI.** The RTL, testbenches, reference models and
> documentation were written with an AI assistant (Claude, by Anthropic),
> working from MAME's source and the games' own test screens. Every block was
> checked in simulation against a MAME-derived reference, the whole core was
> run in a full-system simulation against MAME, and it was then tested on a
> DE10-Nano. This is disclosed so you can decide whether to use it.

---

## Games

| Game | Year | Players | ROM sets | Status |
|---|---|---|---|---|
| Road Riot 4WD | 1991 | 1 | roadriot (04 Dec 1991, conversion kit); roadriota (13 Nov 1991, conversion kit); roadriotb (04 Jun 1991, dedicated twin) | **Playable on hardware, with sound** (roadriot). |
| Guardians of the 'Hood | 1992 | 3 | guardian | **Playable on hardware, with sound.** |
| Danger Express | 1992 (prototype) | 2 | dangerex | **Playable on hardware, with sound.** |

All five sets are named as in MAME 0.264 and load from merged, split or
non-merged ROM sets.

## Progress

```
MiSTer integration     ███████████████████░   95%
ROM loading (.mra)     ███████████████████░   95%
CPU (68000 + SLOOP)    ████████████████████  100%
ASIC65 coprocessor     ███████████████████░   95%
Memory subsystem       ████████████████████  100%
Video                  ██████████████████░░   90%
Sound (JSA III)        ██████████████████░░   90%
I/O and controls       █████████████████░░░   85%
                       ────────────────────
Project                ███████████████████░   94%
```

All three games are playable on a DE10-Nano with sound. Still to be
checked on hardware: Road Riot's two older sets, the third Guardians player,
Road Riot's wheel and pedal from an analog stick, CRT Adjust, and EEPROM
settings saved across a reload.

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

ROMs are not distributed with this core. Copy what is in `mra/` to
`_Arcade`: the three games' MRAs are at its root, and the two older Road
Riot 4WD sets are in `_alternatives/_Road Riot 4WD`, where MiSTer lists
other versions of a game. Put the core from `releases/`
(`Arcade-Atari-G42_<date>.rbf`) in `_Arcade/cores`, and `roadriot.zip`,
`guardian.zip` and `dangerex.zip` in `/games/mame/`. Any MiSTer SDRAM module is enough. The first boot after a
load takes one to two seconds longer than the game's own, while the core
checks the SDRAM and indexes the motion-object ROM.

To build the core, open `Arcade-Atari-G42.qpf` in Quartus Prime 17.0 Lite and
compile. When the compilation succeeds, `release_rbf.tcl` moves the core from
`output_files/Arcade-Atari-G42.rbf` to `releases/Arcade-Atari-G42_<date>.rbf`,
dated like the version in the OSD; a second build on the same day replaces
that day's file.

For diagnosis there is a debug build: open `Arcade-Atari-G42_debug.qpf`
instead, which builds `output_files/Arcade-Atari-G42_debug.rbf` (it stays
there), the same core with the OSD's **Debug** page added. The MRAs start
either build, so keep only one of them in `_Arcade/cores`: with both there,
MiSTer picks the debug build.

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

The analog picture sits in the middle of a standard 15 kHz screen with
equal margins: the sync pulses are where broadcast timing puts them around
the picture. The height is the monitor's own (its vertical size).

The OSD's main page has:

* **Aspect ratio**, **Scandoubler Fx** and **Scale**: MiSTer's standard
  options.
* **[CRT Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)** page: the
  picture's width and position on a 15 kHz CRT, working outward from the
  centre, so with every amount at 0 the picture is where it is with CRT
  Adjust off. **CRT Auto-Width** makes the picture about 48.4 µs wide, so on
  a screen with ordinary overscan it reaches the edges with only a few dots
  hidden; **H-Size** trims it about the middle of the screen (each step
  about 3 %, up to +5 in all); **H-Position** and **V-Shift** move it.
  CRT Adjust switches itself off while a Scandoubler Fx option is on, and
  the HDMI picture follows the adjustment too.
* **Service Menu**: the game's own test menu, from the next reset.
* **Controls** page, Road Riot only: wheel sensitivity for an analog stick.
* **Debug** page, debug build only: a diagnostic overlay, the watchdog, half
  CPU speed, layer switches and SDRAM timing options.

---

## Credits and references

This core is a reimplementation. It would not have been possible without:

**[MAME](https://www.mamedev.org/)**, the reference for the hardware's
behaviour: `atarig42.cpp`, `atarig42_v.cpp` (Aaron Giles), `atarirle.cpp`,
`asic65.cpp`, `atarijsa.cpp`, `eeprompar.cpp`, `adc0808.cpp`. No MAME code is
compiled into the core.

**[MiSTer-CRT-Adjust](https://github.com/rmonic79/MiSTer-CRT-Adjust)** by
Umberto Parisi (**rmonic79**), with Andrea Bogazzi (**asturur**): the CRT
geometry module in `rtl/crt/`, with one fix (negative H-Position offsets),
the same as in the ITech8 core.

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
self-test, clock scheme and overlay, and the **ITech8 core** (Gm0rk) of its
analog centring, CRT Adjust glue and CRT Auto-Width.

## License

The core is released under the GNU General Public License v3.0 (`LICENSE`).
CRT Adjust in `rtl/crt/` is GPL-3.0 as well (`rtl/crt/LICENSE`). `sys/`,
fx68k, T65, JT51 and JT6295 keep their own licences and authorship.

No ROM data is included or distributed.
