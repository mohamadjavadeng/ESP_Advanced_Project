# Building a DWIN DGUS HMI project

Notes distilled from two real DWIN T5L projects on this machine:

* **`D:\Embedded\Excavator Project\HMI`** — *our* in-progress project (480×272,
  DGUS mode). Already imports the generated screens (`00_main_screen`,
  `02_settings_screen`, `03_numeric_keypad`, `04_text_keyboard`).
* **`D:\Project Qolilu\Final Project\FinalCode\V1.0`** — a complete reference
  project (13 pages, keyboard + keypad, icon libraries, error popups). NOTE: its
  kernel is the **Modbus / UART4 / 9600** variant — a *different* protocol mode.
  Use it as a layout/structure reference only, **not** for baud/protocol.

Both were drawn with the **DGUS Tool** ("DwinTerminal" / "BizDraw", ProVersion
641, editor v7.3.6.0). This is the Windows PC tool you build the panel side in.

---

## 0. The one thing to get right first: protocol MODE

A DWIN T5L panel runs one of two firmware kernels. They are **not** wire
compatible — pick the one your MCU/Pi driver speaks:

| Mode | Frame on the wire | Our stack |
|------|-------------------|-----------|
| **DGUS** (default) | `5A A5 <len> <cmd> <addrHi> <addrLo> <data…>`, cmd `0x82` write / `0x83` read | **YES** — `raspberry_pi/dwin_lcd.py`, `dwin_hmi_app.py`, 115200 8N1 |
| **Modbus RTU** | `<id> <fn> <addrHi> <addrLo> <cnt> … <CRC>` | Qolilu's kernel only — ignore here |

`dwin_lcd.py` is a DGUS-mode driver (`_HDR0=0x5A`, `_HDR1=0xA5`). So the panel
must be flashed/configured in **DGUS mode** and the UART set to **115200**. The
Qolilu kernel filename (`DWINOS_DWIN_T5L_9600__UART4_ModBus.bin`) is the wrong
mode for us — don't copy its baud or its kernel.

Baud is set in the panel config (`T5LCFG.CFG` / the DGUS Tool "Configuration
Generate" step), not in the project file.

---

## 1. Project anatomy — what every file is

A DGUS project = a Windows editor project (`DWprj.*`) plus a generated SD-card
image folder (`DWIN_SET`). When you "build", the editor compiles your visual
layout into the `.bin` config files the panel actually runs.

```
HMI/
├── DWprj.hmi          INI manifest (resolution, var count, SPADDRESS, image list)
├── DWprj.tft          editor project (.NET-serialized; the editable layout — keep in git)
├── DWIN_SET/          === the SD-card image: copy this whole folder to the panel ===
│   ├── 0_DWIN_ASC.HZK     font/charset library (ASCII here; +CJK in Qolilu, 3 MB)
│   ├── NN_<name>.bmp       page background images, NN = page index (480×272, 24-bit)
│   ├── 13TouchFile.bin     COMPILED touch config  (touch zones → actions/VPs)
│   ├── 14ShowFile.bin      COMPILED display config (VP → on-screen widget)
│   ├── 22_Config.bin       variable/NOR storage image (persisted values, 128 KB)
│   ├── NN.icl              icon libraries (NN = icon-bank index; for image-array widgets)
│   └── T5LCFG.CFG          hardware config (controller id, resolution, baud, flags)
├── TFT/               per-image editor layers (NN_name.bmp.tft) — editor scratch
├── ICON/  image/      editor working dirs
```

### `DWprj.hmi` (read it — it's plain INI)

Ours:

```ini
[INIT]
PICFIX=1
VARCount=64
Version=-1
ProVersion=641
SCREENDSIZE=480X272      ; panel resolution — must match the .bmp size
SPADDRESS=5000           ; "scratch-pad" base VP for some controls (hex 0x5000)
[IMG]
00=00_main_screen.bmp    ; page 0  <- index is the PAGE NUMBER you switch to
02=02_settings_screen.bmp; page 2
03=03_numeric_keypad.bmp ; page 3
04=04_text_keyboard.bmp  ; page 4
```

The `[IMG]` index **is the page number**. `goto_page(2)` in the driver shows
`02_settings_screen.bmp`. So name files `NN_*.bmp` with the page number you want.

### The compiled `.bin` files

You never hand-edit these — the DGUS Tool writes them from your layout. But it
helps to know what they hold, because they are the contract with the firmware:

* **`14ShowFile.bin`** — one record per on-screen widget: *"watch VP `0xNNNN`,
  render it here as <data number / text / icon / graph>"*. Header `DGUS_2…`.
  Ours contains records for VPs `0x0001`, `0x0012`, `0x0016`, `0x0330`, `0x0350`
  — i.e. depth / beam / stick / ssid / password.
* **`13TouchFile.bin`** — one record per touch rectangle: *"if pressed, do
  <page switch / write value V to VP / start variable input>"*. Ours contains
  records pairing VP `0x0010`, `0x0011`, `0x0030` with the value `0x0022` — the
  cancel / save / page-flag buttons (see the VP map below).

---

## 2. The two widget families you place

### A. Display widgets (panel READS a VP, shows it) → `14ShowFile.bin`

These are how the Pi pushes data to the screen. The Pi writes a VP; the widget
repaints. Common types:

| Widget | Shows | Pi writes with |
|--------|-------|----------------|
| **Data Variable** | a number (with decimals/units) from a 16-bit VP | `write_single_reg(vp, int_value)` |
| **Text Variable** | an ASCII/UTF string from a run of VPs | `write_text(vp, "…", pad=N)` |
| **Icon Variable** | icon #N from an `.icl` bank, chosen by VP value | `write_single_reg(vp, icon_index)` |
| **Data graph / bar** | trend from a VP | `write_single_reg(vp, value)` |

Text widgets: set the **string length** (bytes) in the widget — that is the
max you can write. Our driver pads with spaces (`write_text(..., pad=N)`) so a
shorter new value overwrites leftover characters. Terminator is `0x0000`/`0xFFFF`.

### B. Touch widgets (panel reacts to a press) → `13TouchFile.bin`

| Touch type | Effect | Seen on the Pi as |
|------------|--------|-------------------|
| **Page Switch** | jump to page N (panel-side, no UART needed) | nothing |
| **Return Key Value** (a.k.a. "key value", "basic touch→write") | writes a fixed value to a VP | an auto-upload frame `5A A5 06 83 <vp> 01 <val>` |
| **Variable Data Input** | lets the user type into a VP via a keyboard/keypad page | the typed value, auto-uploaded |
| **Incremental Adjust** | +/- a VP | auto-upload of new value |

**The critical setting for our firmware: "data auto-upload" / "upload after
write".** When enabled on a touch/input control, the panel *pushes* the new VP
value to the UART the moment it changes — no polling. `dwin_hmi_app.py` is built
entirely around this: `read_event()` receives those `0x83` frames, and
`handle_event()` dispatches on `(addr, value)`. If a control does nothing on the
Pi, the first thing to check is that auto-upload is ticked for it.

A "Return Key Value" button writing `0x0022` to VP `0x0010` is exactly our
CANCEL button — that's the `0x0010 == 0x0022` case in the app.

---

## 3. Build workflow (PC → panel)

1. **New project** in the DGUS Tool. Set resolution **480×272** (must equal the
   background `.bmp` size) and the variable/SPADDRESS defaults.
2. **Set protocol = DGUS, UART = 115200 8N1** in the configuration step (so it
   matches `dwin_lcd.py`). Generate `T5LCFG.CFG`.
3. **Import page backgrounds**: add each `NN_screen.bmp` (24-bit, 480×272). The
   index becomes the page number. Generate the font (`*.HZK`) and any icon
   libraries (`*.icl`) you reference.
4. **On each page, place widgets**:
   * Display widgets over the value areas of the background (our generated
     screens leave recessed boxes exactly for this) — assign each its VP.
   * Touch widgets over the buttons — page switches, key-value writes, or
     variable-data-input linked to the keypad/keyboard page. **Tick auto-upload**
     on anything the Pi must react to.
5. **Build** → the tool writes `13TouchFile.bin`, `14ShowFile.bin`, etc. into
   `DWIN_SET`.
6. **Deploy**: copy `DWIN_SET` to a microSD, insert, power-cycle to flash; or
   download over USB/UART with the DWIN download tool. Remove SD, reboot.
7. **Verify from the Pi**: `python3 dwin_hmi_app.py --port /dev/serial0` (or the
   `dwin_page_example.py` self-test). A buzzer beep + a successful `read 0x0014`
   (current page) proves the Pi→panel direction; touching a configured button
   should print a `[recv] 0x00NN = …` line.

---

## 4. Keyboards & keypads (the images we generated)

DGUS input works like this: a **Variable Data Input** touch control on a normal
page is linked to a **keyboard page** (our `03_numeric_keypad` / `04_text_keyboard`).
Pressing the field opens that page; each key on it is a touch zone with a **key
code**; pressing keys edits the target VP; an **OK/Enter** key commits and
(with auto-upload) the value lands on the Pi.

The key-code map is stored as ASCII in the touch file — both reference projects
contain the QWERTY string `!1 @2 #3 … Qq Ww Ee …` and the numeric set. So when
you wire up `04_text_keyboard.bmp`, each drawn key gets a touch zone returning
that character; `aA` is the shift/case key, `DEL`/`CLR`/`OK` are function keys.
This matches the layouts we already drew in `HMI_images/generate_hmi.py`.

---

## 5. This project's VP map (DGUS ↔ firmware contract)

Confirmed against the compiled `.bin` files (✓) or declared in `dwin_hmi_app.py`
(△ = wire it in the DGUS Tool to match):

| VP | Meaning | Direction | Widget / touch type | In bins |
|----|---------|-----------|---------------------|---------|
| `0x0001` | current depth (TEXT) | Pi → panel | Text Variable | ✓ show |
| `0x0200` | driver name (TEXT ≤20) | Pi → panel | Text Variable | △ |
| `0x0300` | target depth field | Pi → panel | Text/Data Variable | △ |
| `0x0012` | beam length (4-char) | panel → Pi | Variable Data Input + **auto-upload** | ✓ show |
| `0x0016` | stick length | panel → Pi | Variable Data Input + **auto-upload** | ✓ show |
| `0x0330` | Wi-Fi SSID | panel → Pi | Variable Data Input + **auto-upload** | ✓ show |
| `0x0350` | Wi-Fi password | panel → Pi | Variable Data Input + **auto-upload** | ✓ show |
| `0x0030` | "on settings page" flag = `0x0022` | panel → Pi | Return Key Value | ✓ touch |
| `0x0011` | SAVE button = `0x0022` | panel → Pi | Return Key Value + **auto-upload** | ✓ touch |
| `0x0010` | CANCEL button = `0x0022` | panel → Pi | Return Key Value + **auto-upload** | ✓ touch |

Note these are **low VPs** (`0x0010`–`0x0016` overlap the DGUS system RTC block
on paper). Both reference projects use this low range anyway and it works in
their DGUS configuration — so it's an accepted convention *here*. If beam/stick
ever read back as garbage or fight the RTC, relocate user data to ≥`0x1000`.

---

## 6. Gotchas learned

* **Auto-upload must be ON** for every control the Pi reacts to — this is the
  single most common "button does nothing" cause.
* **Resolution lock**: `.bmp` size, `SCREENDSIZE`, and `T5LCFG.CFG` must all be
  480×272. A mismatched bmp imports blank/scaled.
* **Protocol/baud**: DGUS mode + 115200 to match `dwin_lcd.py`. Don't reuse
  Qolilu's Modbus/9600 kernel.
* **Writes vs ACKs**: in an event loop, write with `ack=False` (our driver) so a
  write's `0x82` ACK isn't mistaken for — and doesn't consume — an incoming
  auto-upload frame. See `dwin_page_example.py` and `dwin_hmi_app.py`.
* **Text length**: set the widget's byte length ≥ what you'll write; pad on the
  Pi so stale tail characters are cleared.
* **Keep `DWprj.tft` in version control**; treat `DWIN_SET/*.bin` as build
  output (regenerated by the tool).
```
