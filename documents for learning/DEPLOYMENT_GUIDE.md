# Excavation Depth + Safety System — Deployment Guide

How the whole system fits together and how to deploy it. The Raspberry Pi is the
**hub**: it ingests raw angles from the two ESP32 units, computes depth, drives
the DWIN HMI, detects people on the camera, and sends alarms + pictures to the
**GEOMind** (GeoBox) cloud.

---

## 1. System overview

```
   STICK ESP32 ─┐ (WiFi, POST /ingest raw angle, 10 Hz)
                ├──────────────►  RASPBERRY PI 4  ──► DWIN LCD (USART: depth, alarm, offset)
   BEAM  ESP32 ─┘                 │  - compute depth = L1·sin(boom)+L2·sin(stick)
                                   │  - person detection (YOLO on RTSP)
   EZVIZ H8c ───────(LAN cable)────┘  - alarm + JPEG  ──► GEOMind cloud (internet)
                eth0, low latency                        (geobox SDK)
```

| Component        | Job                                              | Talks to Pi via            |
|------------------|--------------------------------------------------|----------------------------|
| ESP32 **stick**  | MPU6050 → raw stick tilt                          | WiFi, `POST /ingest`       |
| ESP32 **beam**   | MPU6050 → raw boom tilt                            | WiFi, `POST /ingest`       |
| **Raspberry Pi** | depth calc, HMI, person detection, cloud upload   | —                          |
| **EZVIZ H8c**    | video for person detection                        | **LAN cable** (RTSP)       |
| **DWIN LCD**     | show depth/alarm, set offset                       | UART / USART (TTL)         |
| **GEOMind**      | cloud: receives alarms + pictures                  | internet (geobox SDK)      |

> Depth model (first-order): `depth = L1·sin(theta_boom) + L2·sin(theta_stick)`.
> The ESP32s send **raw degrees only**; the Pi owns L1/L2, sign and offsets.
> This is implemented in `raspberry_pi/sensor_receiver.py`.

---

## 2. Network topology — the one thing to get right

The Pi needs three links at once: **internet** (cloud), **WiFi** (the 2 ESP32s),
**LAN** (camera). Keep the camera on its **own subnet** so it never fights the
WiFi/internet routing.

### Recommended (mobile excavator): one 4G WiFi router

| Pi interface | Connects to            | Subnet             | Purpose                                          |
|--------------|------------------------|--------------------|--------------------------------------------------|
| `wlan0`      | 4G modem **"AMAN 2"**  | `192.168.100.0/24` | **internet** + the 2 ESP32s; Pi = `192.168.100.38` |
| `eth0`       | EZVIZ H8c (direct)     | `192.168.50.0/24`  | camera RTSP only (no gateway)                    |

- The 4G modem ("AMAN 2" / "AMAN2018") gives the Pi internet **and** is the WiFi
  both ESP32s join. The Pi is `192.168.100.38` on it → that is `RPI_HOST` in the
  ESP32 firmware. Reserve that lease in the modem so it never changes.
- **The camera link MUST be a different subnet** (here `192.168.50.0/24`) with
  **no default gateway**. A Pi cannot have `eth0` and `wlan0` on the same subnet
  — and since the AMAN 2 modem already owns `192.168.100.x`, the camera goes on
  `192.168.50.x`. Keeping the gateway on `wlan0` means internet keeps flowing
  while camera traffic stays local on `eth0`.

### Alternative (no router in the field): Pi as its own AP

- `wlan0` runs `hostapd` as an access point the 2 ESP32s join.
- Internet via a **4G USB dongle** (`wwan0`/`ppp0`).
- `eth0` still the camera. More setup (hostapd + dnsmasq); use only if there is
  no 4G WiFi router available.

---

## 3. ESP32 ↔ Raspberry Pi — how data is shared (and why)

**Chosen method: each ESP32 is a WiFi *station* that HTTP-POSTs a small JSON
packet to the Pi at 10 Hz.** Files: `ESP32_Stick/`, `ESP32_Beam/` →
`raspberry_pi/sensor_receiver.py`.

```
POST http://<RPI_HOST>:8080/ingest      Content-Type: application/json
{"id":"stick","angle_deg":12.34,"pitch_deg":..,"yaw_deg":..,
 "mpu_ok":true,"seq":123,"uptime_ms":456789}
```

The Pi stores the latest sample per unit, computes depth, and serves it on
`GET /status` (same field names the old ESP32 server used, so the DWIN driver,
the cloud uploader and `excavation_monitor.py` all read it unchanged):

```json
{"boom_deg":34.5,"stick_deg":12.3,"depth_m":1.42,"target_depth":1.5,
 "depth_alarm":false,"sensor_ok":true,"boom_age_ms":40,"stick_age_ms":55}
```

**Why HTTP POST:** reuses the team's existing HTTP + ArduinoJson code
(`ExcavatorClient`, `ESP32_serverExcavation`), no broker to run, and is trivial
to debug with `curl`. 10 Hz × 2 units = 20 tiny requests/s — nothing for a Pi 4.

**Safety / liveness:** every packet carries `seq` + `mpu_ok`; the Pi timestamps
arrivals and marks a unit **stale** if silent for `--stale-ms` (default 1500 ms).
`sensor_ok=false` ⇒ raise an alarm — a dead/disconnected sensor must never read
as "shallow and safe".

**Alternatives (not used now, easy to switch to):**
- **MQTT** (mosquitto on the Pi, `PubSubClient` on ESP32) — best *production*
  upgrade: persistent connection, lower overhead, and a **Last-Will** message so
  the Pi learns instantly when a unit drops. Recommended once the system is field-proven.
- **UDP** — lowest latency, fire-and-forget; fine because a missed 100 ms sample
  is replaced by the next one. Use if HTTP latency ever becomes an issue.

---

## 4. Build & flash the two ESP32 units

PlatformIO (VS Code or CLI). Per unit, edit the config block at the top of
`src/main.cpp`:

| Constant       | Set to                                            |
|----------------|---------------------------------------------------|
| `WIFI_SSID/PASS` | `"AMAN 2"` / `"AMAN2018"` (already set)         |
| `RPI_HOST`     | `192.168.100.38` (the Pi on AMAN 2; already set)  |
| `RPI_PORT`     | `8080` (matches `sensor_receiver.py`)             |
| `DEVICE_ID`    | already `"stick"` / `"beam"` per project — leave it |

```bash
cd "documents for learning/ESP32_Stick"   # then again for ESP32_Beam
pio run -t upload          # build + flash over USB
pio device monitor         # watch: WiFi join + [post] ... -> HTTP 200
```

`platformio.ini` reuses the vendored MPU6050 from `ExcavatorClient/lib` (via
`lib_extra_dirs`) and pulls `bblanchon/ArduinoJson` from the registry.
MPU6050 wiring: **SDA=GPIO21, SCL=GPIO22, VCC=3V3, GND=GND**. Onboard LED solid =
WiFi connected. Keep the sensor **still** during the 2 s auto-calibration at boot.

---

## 5. Connect the EZVIZ H8c camera over the LAN cable

The H8c has both WiFi and a wired RJ45 port. Wired is used here for **low,
constant latency** to the Pi.

### 5a. One-time onboarding (needs internet ONCE)
EZVIZ cameras must be activated through the EZVIZ **app/cloud** before local RTSP
works. On any normal internet network first:
1. Add the camera in the EZVIZ app; note the **verification code** (6-char
   UPPERCASE on the label) — this is the RTSP password, user is always `admin`.
2. Enable RTSP: *Device Settings → … → Local Service / LAN Live View → RTSP*.

### 5b. Wire it to the Pi + fix the addresses
Plug the camera's LAN port → Pi `eth0`. Use the `192.168.50.0/24` subnet — **NOT**
`192.168.100.x`, which the AMAN 2 modem already owns on `wlan0`.

A direct Pi↔camera cable has **no DHCP server**, so the camera falls back to a
link-local `169.254.x.x` address that the Pi can't reach ("No route to host").
Make the Pi run **DHCP + NAT** on `eth0` so the camera (DHCP by default) gets a
stable address — and can even reach the EZVIZ cloud for activation — without
`eth0` becoming the Pi's default route. On Raspberry Pi OS Bookworm
(NetworkManager) the `shared` method does all of this:

```bash
sudo nmcli con add type ethernet ifname eth0 con-name cam-lan \
     ipv4.method shared ipv4.addresses 192.168.50.1/24
sudo nmcli con up cam-lan
# power-cycle the camera, then read the IP it leased:
cat /var/lib/NetworkManager/dnsmasq-eth0.leases     # or: ip neigh show dev eth0
```

Quick bootstrap if the camera is currently on `169.254.x.x` and you just want to
reach it right now: `sudo nmcli con modify cam-lan ipv4.method link-local &&
sudo nmcli con up cam-lan` puts eth0 on the same link-local /16.

### 5c. Verify the stream, then run detection
`<CAM_IP>` = the address from the lease above (e.g. `192.168.50.50`):
```bash
ping <CAM_IP>
ffprobe "rtsp://admin:<CODE>@<CAM_IP>:554/H264/ch1/sub/av_stream"

# reuse the existing detector (headless, person-only, TCP transport):
# NOTE: rpi_stream.py defaults to 192.168.100.29 — pass --ip to override.
python3 ezviz_camera/rpi_stream.py --ip <CAM_IP> --code <CODE>
```

> Pitfall: if RTSP refuses to connect, the camera was probably not activated /
> RTSP not enabled (5a), or the verification code is wrong. The PTZ/AI features
> in `ptz_and_detection.py` go through the EZVIZ **cloud** (need internet); local
> **person detection runs entirely on the Pi** from the RTSP frames — no cloud.

---

## 6. Raspberry Pi software stack

Four processes, started by `systemd` on boot:

| Service          | What it runs                                  | Notes                          |
|------------------|-----------------------------------------------|--------------------------------|
| `receiver`       | `raspberry_pi/sensor_receiver.py`             | `:8080`, depth + `/status`     |
| `detector`       | `ezviz_camera/rpi_stream.py` (+ cloud hook)   | person detect → alarm + JPEG   |
| `dwin`           | DWIN UART driver (§8)                          | reads `/status`, draws HMI     |
| (cloud uploader) | `geobox` upload, triggered by detector        | §7                             |

Install once:
```bash
sudo apt update && sudo apt install -y python3-opencv libgl1 mosquitto-clients
pip install flask requests ultralytics geobox
```

Example unit (`/etc/systemd/system/excavator-receiver.service`):
```ini
[Unit]
Description=Excavation sensor receiver
After=network-online.target
[Service]
ExecStart=/usr/bin/python3 /home/pi/ESP_Advanced_Project/raspberry_pi/sensor_receiver.py --l1 1.1 --l2 1.0 --target 1.5
Restart=always
User=pi
[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable --now excavator-receiver
```
Start order: `receiver` → `detector`/`dwin` (they read `/status` from it).

---

## 7. Send alarms + pictures to GEOMind (GeoBox cloud)

SDK: `pip install geobox`. Put credentials in the environment, **not** in code
(you have `PDO@excavator1` for both username + password — confirm the GEOMind
**server endpoint** with your admin; the package targets a pre-configured host):

```bash
export GEOBOX_USERNAME=PDO@excavator1
export GEOBOX_PASSWORD=PDO@excavator1
```

When the detector sees a person, save a JPEG and upload it:
```python
import os
from geobox import GeoboxClient

client = GeoboxClient(username=os.environ["GEOBOX_USERNAME"],
                      password=os.environ["GEOBOX_PASSWORD"])

# inside the detector, on a person event:
file = client.upload_file(path="/var/excavator/alarms/2026-06-24T12-00-00.jpg")
task = file.publish(layer_name="excavator1_alarms")
# task.wait() == TaskStatus.SUCCESS  -> task.output_asset
```
Throttle uploads (e.g. one frame every N seconds while a person is present) so a
standing worker does not flood the cloud or the 4G link. Queue alarms to disk and
retry when offline — the 4G link will drop in the field.

---

## 8. DWIN LCD (T5L, DGUS II) over USART

The Pi drives the DWIN over a UART (`/dev/serial0`, GPIO14 TX / GPIO15 RX). DWIN
TTL is **5 V** on many panels — use a 3.3 V↔5 V level shifter on the Pi RX line.
Protocol = DGUS II variable-pointer (VP) writes/reads:

```
Write VP : 5A A5 <len> 82 <VPh> <VPl> <data…>      ; len = bytes after len
Read  VP : 5A A5 04   83 <VPh> <VPl> <wordcount>
```

Example VP map (define matching controls in the DWIN DGUS project):

| VP      | Meaning                  | Direction        |
|---------|--------------------------|------------------|
| `0x1000`| depth ×1000 (mm)         | Pi → DWIN        |
| `0x1002`| target depth ×1000       | Pi → DWIN        |
| `0x1010`| alarm state (0/1)        | Pi → DWIN        |
| `0x5000`| offset adjust (touch)    | DWIN → Pi        |

Use the Python driver `raspberry_pi/dwin_lcd.py` (a port of the Arduino
`documents for learning/DWINLCD` library; `pip install pyserial`):
```python
from dwin_lcd import DwinLCD, BuzzerDuration
dwin = DwinLCD("/dev/serial0", 115200)

# Pi -> DWIN: push values (depth shown in mm = metres * 1000)
dwin.write_single_reg(0x1000, int(depth_m * 1000))
dwin.write_single_reg(0x1002, int(target_m * 1000))
(dwin.set_single_bit if alarm else dwin.reset_single_bit)(0x1010, 0)
if alarm:
    dwin.buzzer(BuzzerDuration.BUZZ_500MSEC)

# DWIN -> Pi: operator turns the offset on screen (touch writes VP 0x5000)
offset_mm = dwin.read_single_reg(0x5000)
```
Flow: the `dwin` service polls `GET /status`, pushes `depth_m`/`target_depth`/
alarm to the VPs above; it polls `0x5000` for the operator's **offset** and, when
it changes, `POST /config` to the receiver to re-zero the calculation. Enable the
Pi UART first (`raspi-config` → Serial: login shell **No**, hardware **Yes**) and
use a 3.3 V↔5 V level shifter on the Pi RX line.

---

## 9. Depth calibration (do this on the real machine, once)

1. Measure **L1** (boom pivot→stick pivot) and **L2** (stick pivot→bucket tip);
   set via `--l1 --l2` (or `POST /config`).
2. Park the arm so both segments are **horizontal**; read `boom_deg`/`stick_deg`
   from `/status` and set those as `boom_offset_deg`/`stick_offset_deg` (so level
   reads 0°).
3. Dig to a known depth, compare `depth_m`; if it has the wrong sign, flip
   `boom_sign`/`stick_sign` (±1). Re-check at two depths.

---

## 10. Field checklist & power

- Power: ESP32s from machine 12/24 V via buck → 5 V (fuse + reverse-polarity
  protection). Pi from a clean, capable 5 V/3 A supply (brownouts corrupt the SD).
- Mount the MPU6050 rigidly to the arm steel; vibration adds noise — the DMP
  filtering + the 10 Hz rate already smooth most of it.
- Boot order: power Pi first (receiver up), then ESP32s; LED solid = connected.
- Quick health check: `curl http://192.168.100.38:8080/status`.

## 11. Troubleshooting

| Symptom                          | Likely cause / fix                                        |
|----------------------------------|-----------------------------------------------------------|
| ESP32 `[post] failed`            | wrong `RPI_HOST`/port, Pi not on same WiFi, receiver down |
| `sensor_ok:false`, depth frozen  | a unit stale/dead — check that ESP32's power + WiFi LED    |
| RTSP won't open                  | RTSP not enabled / not activated in app / wrong code      |
| Internet lost when camera plugged| `eth0` got a gateway — remove it; gateway must be `wlan0`  |
| DWIN shows nothing               | TX/RX swapped, baud mismatch, or missing level shifter    |
| Depth sign inverted              | flip `boom_sign`/`stick_sign` in `/config`                |
```
