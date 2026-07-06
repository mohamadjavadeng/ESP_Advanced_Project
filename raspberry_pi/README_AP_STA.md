# RPi4 concurrent WiFi client + Access Point + ESP32 HTTP demo

Turn a Raspberry Pi 4 into a WiFi **client and access point at the same time**
(one radio), give the AP a **fixed IP**, and have an **ESP32** join it and send
data over **HTTP POST + GET**.

```
   home router ──wifi──▶ wlan0 (client, gets internet)  ┐
                                                          ├─ Raspberry Pi (one radio)
   ESP32 ────────wifi──▶ uap0  (AP, 192.168.50.1)        ┘  ▲ Flask :8080
                                                             POST /ingest · GET /latest
```

| item        | value            |
|-------------|------------------|
| AP SSID     | `RPi_AP`         |
| AP password | `raspberry123`   |
| Pi AP IP    | `192.168.50.1` (fixed) |
| DHCP pool   | `192.168.50.10`–`192.168.50.100` |
| server port | `8080`           |

Files: `ap_sta_setup.py` (network setup), `ap_demo_server.py` (HTTP server),
`../esp32_ap_client/` (PlatformIO project).

## How the "one radio" trick works
The Pi has a single WiFi chip, so it can't run two real interfaces on two
channels. We add a **virtual** interface `uap0` on the same chip: `wlan0` stays
the client, `uap0` runs the AP via `hostapd`, `dnsmasq` serves DHCP, and
NetworkManager/dhcpcd is told to ignore `uap0`. **Limitation:** the AP is forced
onto whatever channel `wlan0` is connected on — the setup script auto-detects it.

## Raspberry Pi — setup
1. Make sure `wlan0` is already joined to your normal WiFi (`nmtui` or
   `sudo raspi-config`) so the Pi keeps internet.
2. Run the setup once (installs hostapd/dnsmasq, writes all config, enables it):
   ```bash
   cd raspberry_pi
   sudo python3 ap_sta_setup.py
   sudo reboot        # recommended, to confirm it comes up on boot
   ```
3. Start the demo server:
   ```bash
   pip install flask          # once
   python3 ap_demo_server.py  # POST /ingest, GET /latest, GET / on :8080
   ```

### Verify the AP is up
```bash
iw dev                       # lists uap0 (type AP) next to wlan0
ip addr show uap0            # shows 192.168.50.1
sudo systemctl status hostapd dnsmasq ap-sta
iw dev wlan0 link            # confirms wlan0 is still connected as a client
```

## ESP32 — flash & run
```bash
cd esp32_ap_client
pio run -t upload -t monitor
```
Serial (115200) prints each cycle:
```
POST /ingest  seq=1  -> 200  resp={"ok":true,"stored":{...}}
GET  /latest            -> 200  server_seq=1  temp_c=24.71
----
```
Edit SSID/pass/IP at the top of `esp32_ap_client/src/main.cpp` if you changed
them in `ap_sta_setup.py`.

## Optional — run the server on boot (systemd)
```ini
# sudo nano /etc/systemd/system/ap-demo-server.service
[Unit]
Description=ESP32 AP demo HTTP server
After=hostapd.service
[Service]
ExecStart=/usr/bin/python3 /home/pi/ESP_Advanced_Project/raspberry_pi/ap_demo_server.py
Restart=always
User=pi
[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable --now ap-demo-server
```

## Troubleshooting
- **AP doesn't appear / ESP32 can't connect** — almost always the channel. The
  AP must share `wlan0`'s channel. Check `iw dev wlan0 link`, set that number as
  `channel=` in `/etc/hostapd/hostapd.conf`, then `sudo systemctl restart hostapd`.
  (Re-running `ap_sta_setup.py` re-detects it.)
- **`hostapd` won't start** — it ships masked; the script unmasks it. Manually:
  `sudo systemctl unmask hostapd && sudo systemctl restart hostapd`. Check
  `sudo journalctl -u hostapd -b`.
- **Wrong country / weak signal** — set `country_code=` in the hostapd conf and
  the WiFi country in `sudo raspi-config` (Localisation).
- **No DHCP lease on the ESP32** — `sudo journalctl -u dnsmasq -b`; make sure
  `uap0` has `192.168.50.1` (`ip addr show uap0`).
- **5 GHz** — not supported in this concurrent single-radio mode; stay on 2.4 GHz
  (the ESP32 is 2.4 GHz only anyway).
- **Rock-solid alternative** — plug in a USB WiFi dongle and run the AP on it and
  the client on `wlan0` (or vice-versa). Two radios = no shared-channel limit.
