/*
 * web_config.h -- on-device Wi-Fi AP + web configuration portal, shared by the
 * BEAM and the STICK firmware.
 * -----------------------------------------------------------------------------
 * This file (and web_config.cpp) is BYTE-IDENTICAL in ESP32_Beam/src and
 * ESP32_Stick/src. Everything unit-specific arrives through the FieldCfgIdentity
 * block that main.cpp hands to fieldcfg_begin(), so when you edit one copy just
 * copy both files over the other project.
 *
 * What it gives each unit:
 *   - a SoftAP with a fixed SSID + fixed IP that is ALWAYS up (WIFI_AP_STA), so
 *     the unit stays reachable even when the field-Wi-Fi credentials are wrong;
 *   - a small web page (http://<AP IP>/) that shows the LIVE angle (and GNSS fix
 *     on the stick) and lets the operator type the field Wi-Fi SSID + password,
 *     the Raspberry Pi IP/port, and the length of the arm this unit is bolted to;
 *   - persistence in NVS (Preferences namespace "fieldcfg") -- survives reflash
 *     of the app as long as the nvs partition is not erased;
 *   - a ONE-SHOT length hand-off to the Pi: a new length is POSTed to
 *     /length and retried ONLY until the Pi answers ok=true, then never again.
 *
 * Threading contract (matters on the stick, which is dual-core):
 *   fieldcfg_handle()  -> call often from ONE task only (>= 20 Hz). It services
 *                         the HTTP server, so it also owns the config state.
 *   fieldcfg_sync()    -> call ~1 Hz from the task that owns WiFi.begin().
 *   fieldcfg_set_*()   -> safe from any core (tiny critical section).
 */
#pragma once

#include <Arduino.h>

// Per-unit identity + compile-time factory defaults. main.cpp fills this in.
struct FieldCfgIdentity {
  const char *device_id;      // "beam" / "stick" -- the id the Pi keys on
  const char *title;          // shown at the top of the web page
  const char *ap_ssid;        // SoftAP name, e.g. "EXCAV-BEAM"
  const char *ap_pass;        // SoftAP password, MUST be >= 8 chars (WPA2)
  uint8_t     ap_ip[4];       // fixed SoftAP address, e.g. {192,168,4,1}
  const char *mdns_host;      // "beam" -> http://beam.local on the field Wi-Fi
  bool        has_gnss;       // true on the stick (SIM7000G): show the GPS card
  // Factory defaults, used until the operator saves something on the web page.
  const char *def_ssid;
  const char *def_pass;
  const char *def_rpi_host;
  uint16_t    def_rpi_port;
  uint32_t    def_len_mm;     // arm length in millimetres
};

// Plausible boom/stick length -- same window the Pi enforces (LEN_MM_MIN/MAX in
// monitoring_control.py). Outside it the entry is junk and is refused, because a
// silently accepted 4 mm arm makes the depth readout look frozen.
#define FIELDCFG_LEN_MM_MIN 100UL
#define FIELDCFG_LEN_MM_MAX 20000UL

// --- lifecycle ---------------------------------------------------------------
// Loads NVS, switches the radio to AP+STA, brings up the SoftAP on its fixed IP,
// starts the web server and mDNS. Does NOT call WiFi.begin() -- main.cpp does
// that with fieldcfg_ssid()/fieldcfg_pass() so one task owns the STA link.
void fieldcfg_begin(const FieldCfgIdentity &id);

// Service the HTTP server. Call from ONE task, as often as you can (>= 20 Hz).
void fieldcfg_handle();

// One-shot length hand-off to the Pi + retry timer. Call ~1 Hz from the task
// that owns WiFi.begin(); it opens its own short-lived socket.
void fieldcfg_sync(uint32_t now_ms);

// True exactly once after the operator saved new Wi-Fi credentials: the caller
// must re-run WiFi.begin(fieldcfg_ssid(), fieldcfg_pass()).
bool fieldcfg_take_wifi_reapply();

// --- saved configuration (read by main.cpp) ---------------------------------
const char *fieldcfg_ssid();
const char *fieldcfg_pass();
const char *fieldcfg_host();      // Raspberry Pi IP / hostname
uint16_t    fieldcfg_port();
uint32_t    fieldcfg_len_mm();
uint32_t    fieldcfg_len_seq();   // edit counter; 0 = never set on the web page

// --- live telemetry shown on the web page (safe from any core) ---------------
void fieldcfg_set_angles(float roll, float pitch, float yaw, bool mpu_ok);
void fieldcfg_set_gps(bool fix, float lat, float lon, float alt,
                      float speed_kph, int sats, float hacc);

// --- length hand-off plumbing -----------------------------------------------
// Feed every /ingest response body here. When the Pi answers "need_length":true
// (it lost its state file, or was replaced) the ack is dropped so fieldcfg_sync()
// pushes the length again. Cheap: the reply is a ~30-byte JSON object.
void fieldcfg_check_ingest_reply(const String &body);
