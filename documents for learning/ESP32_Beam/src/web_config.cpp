/*
 * web_config.cpp -- see web_config.h. BYTE-IDENTICAL in ESP32_Beam/src and
 * ESP32_Stick/src: copy both files across when you edit one.
 *
 * Everything here is plain Arduino-ESP32 core (2.0.x): WebServer, ESPmDNS and
 * Preferences ship with the framework, so no new lib_deps are needed.
 */
#include "web_config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// --------------------------------------------------------------- saved config --
static const char *NVS_NS = "fieldcfg";      // Preferences namespace
static const uint32_t LEN_RETRY_MS = 5000;   // gap between length-push attempts

struct Cfg {
  String   ssid, pass, host;
  uint16_t port    = 5000;
  uint32_t len_mm  = 0;
  uint32_t len_seq = 0;    // bumped on every accepted edit; 0 = never set here
  bool     len_ack = false;// the Pi confirmed len_seq (then we stop sending)
};
static Cfg               cfg;
static FieldCfgIdentity  ident;
static WebServer         server(80);
static bool              begun         = false;
static bool              wifi_reapply  = false;
static bool              len_tried     = false;   // at least one push attempted
static uint32_t          len_last_try  = 0;
static uint32_t          len_tries     = 0;
static int               len_last_code = 0;       // last HTTP code from /length

// Live telemetry. Written by the sensor code (on the stick that is the OTHER
// core), read by the HTTP handler -> copy in/out under a tiny spinlock so the
// page never shows a half-updated GNSS fix.
static portMUX_TYPE live_mux = portMUX_INITIALIZER_UNLOCKED;
struct Live {
  float roll = 0, pitch = 0, yaw = 0;
  bool  mpu_ok = false;
  bool  gps_fix = false;
  float lat = 0, lon = 0, alt = 0, speed = 0, hacc = 0;
  int   sats = 0;
  uint32_t gps_ms = 0;      // millis() of the last GNSS update (0 = never)
};
static Live live;

// ------------------------------------------------------------------- storage --
static void loadCfg() {
  Preferences p;
  // Read-WRITE even though we only read: opening a namespace that does not exist
  // yet read-only fails and dumps an "nvs_open ... NOT_FOUND" error on the first
  // boot of a fresh unit. Read-write creates it quietly; the defaults below are
  // what an unprovisioned unit then runs on.
  p.begin(NVS_NS, false);
  cfg.ssid    = p.getString("ssid", ident.def_ssid);
  cfg.pass    = p.getString("pass", ident.def_pass);
  cfg.host    = p.getString("rpi_host", ident.def_rpi_host);
  cfg.port    = p.getUShort("rpi_port", ident.def_rpi_port);
  cfg.len_mm  = p.getULong("len_mm", ident.def_len_mm);
  cfg.len_seq = p.getULong("len_seq", 0);
  cfg.len_ack = p.getBool("len_ack", false);
  p.end();
}

static void saveCfg() {
  Preferences p;
  p.begin(NVS_NS, false);
  p.putString("ssid", cfg.ssid);
  p.putString("pass", cfg.pass);
  p.putString("rpi_host", cfg.host);
  p.putUShort("rpi_port", cfg.port);
  p.putULong("len_mm", cfg.len_mm);
  p.putULong("len_seq", cfg.len_seq);
  p.putBool("len_ack", cfg.len_ack);
  p.end();
}

static void saveAck() {                        // just the ack flag (called often)
  Preferences p;
  p.begin(NVS_NS, false);
  p.putBool("len_ack", cfg.len_ack);
  p.end();
}

const char *fieldcfg_ssid()    { return cfg.ssid.c_str(); }
const char *fieldcfg_pass()    { return cfg.pass.c_str(); }
const char *fieldcfg_host()    { return cfg.host.c_str(); }
uint16_t    fieldcfg_port()    { return cfg.port; }
uint32_t    fieldcfg_len_mm()  { return cfg.len_mm; }
uint32_t    fieldcfg_len_seq() { return cfg.len_seq; }

bool fieldcfg_take_wifi_reapply() {
  if (!wifi_reapply) return false;
  wifi_reapply = false;
  return true;
}

// ------------------------------------------------------------ live telemetry --
void fieldcfg_set_angles(float roll, float pitch, float yaw, bool mpu_ok) {
  portENTER_CRITICAL(&live_mux);
  live.roll = roll; live.pitch = pitch; live.yaw = yaw; live.mpu_ok = mpu_ok;
  portEXIT_CRITICAL(&live_mux);
}

void fieldcfg_set_gps(bool fix, float lat, float lon, float alt,
                      float speed_kph, int sats, float hacc) {
  uint32_t now = millis();
  portENTER_CRITICAL(&live_mux);
  live.gps_fix = fix;
  if (fix) {
    live.lat = lat; live.lon = lon; live.alt = alt;
    live.speed = speed_kph; live.sats = sats; live.hacc = hacc;
  } else {
    live.sats = sats;
  }
  live.gps_ms = now;
  portEXIT_CRITICAL(&live_mux);
}

// ------------------------------------------------------------------ the page --
// Static HTML in flash; every value is filled in by fetch() from /api/live and
// /api/config, so nothing here has to be string-substituted at runtime.
static const char PAGE_INDEX[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Excavator sensor unit</title><style>
*{box-sizing:border-box}
body{margin:0 auto;max-width:640px;padding:14px;background:#0f1115;color:#e8ecf1;
font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
h1{font-size:19px;margin:0 0 2px}
.sub{color:#8b95a5;font-size:12px;margin-bottom:14px}
.card{background:#171a21;border:1px solid #242a35;border-radius:10px;padding:12px;margin-bottom:12px}
.card h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:#8b95a5;margin:0 0 10px;font-weight:700}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(108px,1fr));gap:10px}
.k{color:#8b95a5;font-size:10px;text-transform:uppercase;letter-spacing:.06em}
.v{font-size:20px;font-weight:600;font-variant-numeric:tabular-nums}
.v.sm{font-size:13px;font-weight:500;word-break:break-all}
.b{display:inline-block;padding:2px 8px;border-radius:999px;font-size:11px;font-weight:700}
.ok{background:#12351f;color:#4ade80}.bad{background:#3a1618;color:#f87171}
.warn{background:#3a2c10;color:#fbbf24}.off{background:#20242c;color:#8b95a5}
label{display:block;font-size:12px;color:#8b95a5;margin:10px 0 4px}
input{width:100%;padding:9px 10px;border-radius:8px;border:1px solid #2c3340;background:#0d1014;color:#e8ecf1;font:inherit}
input:focus{outline:2px solid #3ea6ff55;border-color:#3ea6ff}
.row{display:flex;gap:8px}.row>*{flex:1}
button{margin-top:12px;width:100%;padding:11px;border:0;border-radius:8px;background:#3ea6ff;color:#04121f;font:inherit;font-weight:700}
button.alt{background:#242a35;color:#e8ecf1}
.hint{color:#6b7686;font-size:11px;margin-top:6px}
#msg{margin-top:10px;font-size:13px;min-height:18px}
a{color:#3ea6ff}
</style></head><body>
<h1 id="title">unit</h1><div class="sub" id="sub">connecting...</div>

<div class="card"><h2>Angle (live)</h2><div class="grid">
<div><div class="k">roll / arm tilt</div><div class="v" id="roll">-</div></div>
<div><div class="k">pitch</div><div class="v" id="pitch">-</div></div>
<div><div class="k">yaw</div><div class="v" id="yaw">-</div></div>
<div><div class="k">MPU6050</div><div class="v sm"><span id="mpu" class="b off">?</span></div></div>
</div></div>

<div class="card"><h2>Links</h2><div class="grid">
<div><div class="k">field wi-fi</div><div class="v sm"><span id="sta" class="b off">?</span></div></div>
<div><div class="k">joined ssid</div><div class="v sm" id="ssid">-</div></div>
<div><div class="k">this unit's IP</div><div class="v sm" id="ip">-</div></div>
<div><div class="k">signal</div><div class="v sm" id="rssi">-</div></div>
<div><div class="k">raspberry pi</div><div class="v sm" id="pi">-</div></div>
<div><div class="k">config AP</div><div class="v sm" id="ap">-</div></div>
<div><div class="k">uptime</div><div class="v sm" id="up">-</div></div>
<div><div class="k">free heap</div><div class="v sm" id="heap">-</div></div>
</div></div>

<div class="card" id="gpscard" style="display:none"><h2>Location (GNSS)</h2><div class="grid">
<div><div class="k">fix</div><div class="v sm"><span id="fix" class="b off">?</span></div></div>
<div><div class="k">satellites</div><div class="v" id="sats">-</div></div>
<div><div class="k">latitude</div><div class="v sm" id="lat">-</div></div>
<div><div class="k">longitude</div><div class="v sm" id="lon">-</div></div>
<div><div class="k">altitude</div><div class="v sm" id="alt">-</div></div>
<div><div class="k">accuracy</div><div class="v sm" id="hacc">-</div></div>
<div><div class="k">speed</div><div class="v sm" id="spd">-</div></div>
<div><div class="k">fix age</div><div class="v sm" id="gage">-</div></div>
</div><div class="hint" id="maplink"></div></div>

<div class="card"><h2>Arm length -> Raspberry Pi</h2><div class="grid">
<div><div class="k">assigned length</div><div class="v" id="lenv">-</div></div>
<div><div class="k">hand-off to pi</div><div class="v sm"><span id="lenb" class="b off">?</span></div></div>
<div><div class="k">edit no. (seq)</div><div class="v sm" id="lseq">-</div></div>
<div><div class="k">last attempt</div><div class="v sm" id="lcode">-</div></div>
</div>
<div class="hint">The length is sent to the Pi ONCE per edit and retried every 5 s
only until the Pi confirms it. After that it is never sent again.</div>
<button class="alt" id="resend">Re-send length to the Pi</button></div>

<div class="card"><h2>Settings</h2>
<label>Field Wi-Fi SSID (the network the Raspberry Pi is on)</label>
<input id="f_ssid" list="nets" autocapitalize="off" autocomplete="off" spellcheck="false">
<datalist id="nets"></datalist>
<button class="alt" id="scan">Scan for networks</button>
<label>Field Wi-Fi password <span id="pwset"></span></label>
<input id="f_pass" type="password" autocomplete="off" placeholder="leave blank = keep current">
<div class="row">
<div><label>Raspberry Pi IP</label><input id="f_host" autocapitalize="off" autocomplete="off" spellcheck="false"></div>
<div><label>Port</label><input id="f_port" type="number" min="1" max="65535"></div>
</div>
<label>Length of the arm THIS unit is bolted to (mm)</label>
<input id="f_len" type="number" min="100" max="20000" step="1">
<div class="hint">100-20000 mm. The Pi uses it as L1 (beam) or L2 (stick) in
depth = L1*sin(boom) + L2*sin(stick).</div>
<button id="save">Save &amp; apply</button>
<button class="alt" id="reboot">Reboot unit</button>
<div id="msg"></div>
<div class="hint">Saving new Wi-Fi credentials re-joins the field network. This
config AP stays up the whole time, so a wrong password can always be corrected
from here -- but the AP hops to the router's radio channel, so your phone may
have to re-join it.</div></div>

<script>
var $=function(s){return document.getElementById(s)};
function bdg(el,cls,txt){el.className='b '+cls;el.textContent=txt}
function fmt(n,d){return (n===null||n===undefined)?'-':Number(n).toFixed(d)}
function dur(s){s=Math.floor(s);var h=Math.floor(s/3600),m=Math.floor(s%3600/60);
return h?h+'h '+m+'m':(m?m+'m '+(s%60)+'s':s+'s')}
var seenCfg=false;
function paint(d){
 $('title').textContent=d.title; document.title=d.title;
 $('sub').textContent='device id "'+d.id+'"  |  config AP '+d.net.ap_ssid+
   ' @ '+d.net.ap_ip+'  |  '+d.net.ap_clients+' client(s) on the AP';
 $('roll').textContent=fmt(d.imu.roll,2)+'°';
 $('pitch').textContent=fmt(d.imu.pitch,2)+'°';
 $('yaw').textContent=fmt(d.imu.yaw,2)+'°';
 bdg($('mpu'),d.imu.mpu_ok?'ok':'bad',d.imu.mpu_ok?'DMP ready':'no data');
 bdg($('sta'),d.net.sta_ok?'ok':'bad',d.net.sta_ok?'joined':'not joined');
 $('ssid').textContent=d.cfg.ssid||'-';
 $('ip').textContent=d.net.sta_ok?d.net.sta_ip:'-';
 $('rssi').textContent=d.net.sta_ok?d.net.rssi+' dBm':'-';
 $('pi').textContent=d.cfg.host+':'+d.cfg.port;
 $('ap').textContent=d.net.ap_ssid+' / '+d.net.ap_ip;
 $('up').textContent=dur(d.sys.uptime_s);
 $('heap').textContent=Math.round(d.sys.heap/1024)+' kB';
 if(d.gps.has){
  $('gpscard').style.display='';
  bdg($('fix'),d.gps.fix?'ok':'warn',d.gps.fix?'3D fix':'searching');
  $('sats').textContent=d.gps.sats;
  $('lat').textContent=d.gps.fix?fmt(d.gps.lat,6):'-';
  $('lon').textContent=d.gps.fix?fmt(d.gps.lon,6):'-';
  $('alt').textContent=d.gps.fix?fmt(d.gps.alt_m,1)+' m':'-';
  $('hacc').textContent=d.gps.fix?fmt(d.gps.hacc_m,1)+' m':'-';
  $('spd').textContent=d.gps.fix?fmt(d.gps.speed_kph,1)+' km/h':'-';
  $('gage').textContent=d.gps.age_s<0?'never':dur(d.gps.age_s)+' ago';
  $('maplink').innerHTML=d.gps.fix?('<a target="_blank" rel="noreferrer" href="https://www.openstreetmap.org/?mlat='+
    d.gps.lat+'&mlon='+d.gps.lon+'#map=18/'+d.gps.lat+'/'+d.gps.lon+'">open this fix in a map</a>'):
    'no fix yet - a cold start needs 30-60 s with sky view';
 }
 $('lenv').textContent=d.cfg.len_mm?d.cfg.len_mm+' mm':'not set';
 $('lseq').textContent=d.cfg.len_seq?d.cfg.len_seq:'never edited here';
 $('lcode').textContent=d.cfg.len_tries?('HTTP '+d.cfg.len_code+' after '+d.cfg.len_tries+' try/tries'):'-';
 if(!d.cfg.len_seq) bdg($('lenb'),'off','nothing to send');
 else if(d.cfg.len_ack) bdg($('lenb'),'ok','confirmed by pi');
 else bdg($('lenb'),'warn','pending');
 if(!seenCfg){seenCfg=true;
  $('f_ssid').value=d.cfg.ssid; $('f_host').value=d.cfg.host;
  $('f_port').value=d.cfg.port; $('f_len').value=d.cfg.len_mm||'';
  $('pwset').textContent=d.cfg.pass_set?'(a password is stored)':'(none stored)';
 }
}
function live(){fetch('/api/live',{cache:'no-store'}).then(function(r){return r.json()})
 .then(paint).catch(function(){$('sub').textContent='lost the unit - reconnect to its AP'})}
live(); setInterval(live,1000);

$('save').onclick=function(){
 var b={ssid:$('f_ssid').value.trim(),host:$('f_host').value.trim(),
        port:Number($('f_port').value),len_mm:Number($('f_len').value)};
 var p=$('f_pass').value; if(p.length) b.pass=p;
 $('msg').textContent='saving...';
 fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(b)}).then(function(r){return r.json().then(function(j){return [r.ok,j]})})
 .then(function(x){var ok=x[0],j=x[1];
  $('msg').innerHTML=ok?'<span class="b ok">saved</span> '+(j.note||''):
   '<span class="b bad">rejected</span> '+(j.errors||[]).join('; ');
  seenCfg=false; $('f_pass').value=''; live();})
 .catch(function(e){$('msg').textContent='save failed: '+e})};

$('reboot').onclick=function(){if(!confirm('Reboot this unit now?'))return;
 fetch('/api/reboot',{method:'POST'});$('msg').textContent='rebooting - re-join the AP in ~10 s'};

$('resend').onclick=function(){fetch('/api/resend',{method:'POST'})
 .then(function(){$('msg').textContent='length queued for the Pi again';live()})};

$('scan').onclick=function(){var b=$('scan');b.textContent='scanning...';
 var poll=function(){fetch('/api/scan',{cache:'no-store'}).then(function(r){return r.json()})
  .then(function(j){if(j.scanning){setTimeout(poll,800);return}
   var dl=$('nets');dl.innerHTML='';
   j.nets.forEach(function(n){var o=document.createElement('option');
    o.value=n.ssid;o.label=n.rssi+' dBm';dl.appendChild(o)});
   b.textContent='Scan for networks ('+j.nets.length+' found)'})
  .catch(function(){b.textContent='scan failed - try again'})};
 poll()};
</script></body></html>
)HTML";

// -------------------------------------------------------------- HTTP helpers --
static void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void handleIndex() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE_INDEX);
}

static void handleLive() {
  Live L;
  portENTER_CRITICAL(&live_mux);
  L = live;
  portEXIT_CRITICAL(&live_mux);

  bool sta = (WiFi.status() == WL_CONNECTED);
  JsonDocument d;
  d["id"]    = ident.device_id;
  d["title"] = ident.title;

  JsonObject imu = d["imu"].to<JsonObject>();
  imu["roll"] = L.roll; imu["pitch"] = L.pitch; imu["yaw"] = L.yaw;
  imu["mpu_ok"] = L.mpu_ok;

  JsonObject net = d["net"].to<JsonObject>();
  net["sta_ok"]     = sta;
  net["sta_ip"]     = sta ? WiFi.localIP().toString() : String("-");
  net["rssi"]       = sta ? WiFi.RSSI() : 0;
  net["ap_ssid"]    = ident.ap_ssid;
  net["ap_ip"]      = WiFi.softAPIP().toString();
  net["ap_clients"] = WiFi.softAPgetStationNum();

  JsonObject gps = d["gps"].to<JsonObject>();
  gps["has"] = ident.has_gnss;
  gps["fix"] = L.gps_fix;
  gps["lat"] = L.lat; gps["lon"] = L.lon;
  gps["alt_m"] = L.alt; gps["speed_kph"] = L.speed;
  gps["sats"] = L.sats; gps["hacc_m"] = L.hacc;
  gps["age_s"] = L.gps_ms ? (int)((millis() - L.gps_ms) / 1000) : -1;

  JsonObject c = d["cfg"].to<JsonObject>();
  c["ssid"] = cfg.ssid;
  c["pass_set"] = cfg.pass.length() > 0;
  c["host"] = cfg.host;
  c["port"] = cfg.port;
  c["len_mm"] = cfg.len_mm;
  c["len_seq"] = cfg.len_seq;
  c["len_ack"] = cfg.len_ack;
  c["len_tries"] = len_tries;
  c["len_code"] = len_last_code;

  JsonObject s = d["sys"].to<JsonObject>();
  s["uptime_s"] = (uint32_t)(millis() / 1000);
  s["heap"]     = ESP.getFreeHeap();

  String out;
  serializeJson(d, out);
  sendJson(200, out);
}

// GET /api/config -- the saved values, never the stored password.
static void handleGetConfig() {
  JsonDocument d;
  d["id"] = ident.device_id;
  d["ssid"] = cfg.ssid;
  d["pass_set"] = cfg.pass.length() > 0;
  d["host"] = cfg.host;
  d["port"] = cfg.port;
  d["len_mm"] = cfg.len_mm;
  d["len_seq"] = cfg.len_seq;
  d["len_ack"] = cfg.len_ack;
  String out; serializeJson(d, out);
  sendJson(200, out);
}

// One incoming field: present or not, plus its text. Accepts a JSON body (what
// the web page sends) OR classic form/query args (what `curl -d ssid=..` sends),
// so the unit can be provisioned from a script without a browser.
struct InField { bool has; String val; };

static InField pickField(JsonDocument &d, bool json_ok, const char *key) {
  InField f = {false, String()};
  if (json_ok) {
    if (!d[key].isNull()) { f.has = true; f.val = d[key].as<String>(); }
  } else if (server.hasArg(key)) {
    f.has = true; f.val = server.arg(key);
  }
  return f;
}

// POST /api/config -- apply + persist. A key that is absent (or an empty string
// for ssid/pass/host) leaves that setting alone, so a partial update is fine:
//   curl -d '{"len_mm":4200}' -H 'Content-Type: application/json' http://192.168.4.1/api/config
static void handleSetConfig() {
  String body = server.hasArg("plain") ? server.arg("plain") : String();
  JsonDocument in;
  bool json_ok = body.length() && (deserializeJson(in, body) == DeserializationError::Ok);

  JsonDocument errs;
  JsonArray  err = errs.to<JsonArray>();
  bool wifi_changed = false, host_changed = false, len_changed = false;

  InField f = pickField(in, json_ok, "ssid");
  if (f.has && f.val.length()) {
    if (f.val.length() > 32) err.add("ssid longer than 32 characters");
    else if (f.val != cfg.ssid) { cfg.ssid = f.val; wifi_changed = true; }
  }

  f = pickField(in, json_ok, "pass");
  if (f.has && f.val.length()) {
    if (f.val.length() > 63) err.add("wi-fi password longer than 63 characters");
    else if (f.val != cfg.pass) { cfg.pass = f.val; wifi_changed = true; }
  }
  // Explicit opt-in for an OPEN field network (an empty "pass" means "keep").
  f = pickField(in, json_ok, "pass_clear");
  if (f.has && (f.val == "1" || f.val == "true")) {
    cfg.pass = ""; wifi_changed = true;
  }

  f = pickField(in, json_ok, "host");
  if (f.has && f.val.length()) {
    if (f.val.length() > 63) err.add("raspberry pi host too long");
    else if (f.val != cfg.host) { cfg.host = f.val; host_changed = true; }
  }

  f = pickField(in, json_ok, "port");
  if (f.has && f.val.length()) {
    long v = f.val.toInt();
    if (v < 1 || v > 65535) err.add("port must be 1-65535");
    else if ((uint16_t)v != cfg.port) { cfg.port = (uint16_t)v; host_changed = true; }
  }

  f = pickField(in, json_ok, "len_mm");
  if (f.has && f.val.length()) {
    long v = f.val.toInt();
    if (v < (long)FIELDCFG_LEN_MM_MIN || v > (long)FIELDCFG_LEN_MM_MAX)
      err.add("length must be 100-20000 mm");
    else if ((uint32_t)v != cfg.len_mm) { cfg.len_mm = (uint32_t)v; len_changed = true; }
  }

  if (err.size()) {
    JsonDocument out;
    out["ok"] = false;
    out["errors"] = err;
    String s; serializeJson(out, s);
    Serial.printf("[web] config REJECTED: %s\n", s.c_str());
    sendJson(400, s);
    return;
  }

  String note;
  // A new length starts a fresh one-shot hand-off. So does a new Pi address: the
  // machine at the other end has never heard our length before.
  if (len_changed) {
    cfg.len_seq++;
    cfg.len_ack = false;
    note += "length queued for the Pi (seq " + String(cfg.len_seq) + "). ";
  } else if (host_changed && cfg.len_seq > 0) {
    cfg.len_ack = false;
    note += "new Pi address - length will be re-sent. ";
  }
  if (len_changed || host_changed) { len_tried = false; len_tries = 0; }
  if (wifi_changed) {
    wifi_reapply = true;
    note += "re-joining wi-fi '" + cfg.ssid + "'. ";
  }
  if (note.length() == 0) note = "no change.";
  saveCfg();

  Serial.printf("[web] config saved: ssid='%s' pi=%s:%u len=%lu mm seq=%lu -- %s\n",
                cfg.ssid.c_str(), cfg.host.c_str(), cfg.port,
                (unsigned long)cfg.len_mm, (unsigned long)cfg.len_seq, note.c_str());

  JsonDocument out;
  out["ok"] = true;
  out["note"] = note;
  out["ssid"] = cfg.ssid;
  out["host"] = cfg.host;
  out["port"] = cfg.port;
  out["len_mm"] = cfg.len_mm;
  out["len_seq"] = cfg.len_seq;
  out["wifi_reapply"] = wifi_reapply;
  String s; serializeJson(out, s);
  sendJson(200, s);
}

// POST /api/resend -- clear the ack so the current length goes to the Pi again.
static void handleResend() {
  if (cfg.len_seq == 0) {
    sendJson(409, "{\"ok\":false,\"error\":\"no length has been assigned here yet\"}");
    return;
  }
  cfg.len_ack = false;
  len_tried = false;
  len_tries = 0;
  saveAck();
  Serial.printf("[len] operator asked for a re-send (%lu mm seq=%lu)\n",
                (unsigned long)cfg.len_mm, (unsigned long)cfg.len_seq);
  sendJson(200, "{\"ok\":true}");
}

static void handleReboot() {
  sendJson(200, "{\"ok\":true}");
  Serial.println("[web] reboot requested from the web page");
  delay(250);
  ESP.restart();
}

// GET /api/scan -- ASYNC scan, so a 2-3 s channel sweep never stalls the caller
// (a blocking scan inside the angle task would make the Pi flag this unit stale).
// scanComplete() is the whole state machine: -2 = idle/failed, -1 = running,
// >= 0 = that many results waiting.
static void handleScan() {
  int16_t n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    sendJson(200, "{\"scanning\":true}");
    return;
  }
  if (n == WIFI_SCAN_FAILED) {                 // nothing running -> start one
    WiFi.scanNetworks(true, false);
    sendJson(200, "{\"scanning\":true}");
    return;
  }
  JsonDocument d;
  d["scanning"] = false;
  JsonArray nets = d["nets"].to<JsonArray>();
  for (int i = 0; i < n && i < 20; i++) {
    JsonObject o = nets.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
  }
  WiFi.scanDelete();
  String out; serializeJson(d, out);
  sendJson(200, out);
}

// -------------------------------------------------- one-shot length hand-off --
static bool pushLength() {
  WiFiClient client;
  HTTPClient http;
  String url = String("http://") + cfg.host + ":" + cfg.port + "/length";
  if (!http.begin(client, url)) {
    len_last_code = -1000;
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(1200);
  http.setTimeout(1500);

  JsonDocument doc;
  doc["id"]     = ident.device_id;
  doc["len_mm"] = cfg.len_mm;
  doc["seq"]    = cfg.len_seq;
  doc["ip"]     = WiFi.localIP().toString();
  String body; serializeJson(doc, body);

  int code = http.POST(body);
  String reply = (code > 0) ? http.getString() : String();
  http.end();
  len_last_code = code;

  bool ok = false;
  if (code == 200) {
    JsonDocument r;
    if (deserializeJson(r, reply) == DeserializationError::Ok)
      ok = r["ok"] | false;
  }
  if (ok) {
    Serial.printf("[len] %lu mm (seq %lu) CONFIRMED by the Pi at %s:%u -- "
                  "will not be sent again\n",
                  (unsigned long)cfg.len_mm, (unsigned long)cfg.len_seq,
                  cfg.host.c_str(), cfg.port);
  } else {
    String why = reply.length() ? (", " + reply) : String();
    Serial.printf("[len] push of %lu mm (seq %lu) to %s:%u failed "
                  "(HTTP %d%s) -- retry in %lu s\n",
                  (unsigned long)cfg.len_mm, (unsigned long)cfg.len_seq,
                  cfg.host.c_str(), cfg.port, code, why.c_str(),
                  (unsigned long)(LEN_RETRY_MS / 1000));
  }
  return ok;
}

void fieldcfg_sync(uint32_t now_ms) {
  if (!begun) return;
  if (cfg.len_seq == 0 || cfg.len_ack) return;      // nothing to hand over
  if (WiFi.status() != WL_CONNECTED) return;
  if (len_tried && (uint32_t)(now_ms - len_last_try) < LEN_RETRY_MS) return;
  len_tried = true;
  len_last_try = now_ms;
  len_tries++;
  if (pushLength()) {
    cfg.len_ack = true;
    saveAck();
  }
}

void fieldcfg_check_ingest_reply(const String &body) {
  // Fast path: the normal reply is {"ok":true,...} with no such key.
  if (body.length() == 0 || body.indexOf("need_length") < 0) return;
  JsonDocument d;
  if (deserializeJson(d, body) != DeserializationError::Ok) return;
  if (!(d["need_length"] | false)) return;
  if (cfg.len_seq == 0 || !cfg.len_ack) return;     // nothing to do / already queued
  cfg.len_ack = false;
  len_tried = false;
  len_tries = 0;
  saveAck();
  Serial.printf("[len] the Pi does not know our length (seq %lu) -- re-sending\n",
                (unsigned long)cfg.len_seq);
}

// ------------------------------------------------------------------ lifecycle --
void fieldcfg_begin(const FieldCfgIdentity &id) {
  ident = id;
  loadCfg();

  // AP+STA: the config AP is ALWAYS up, so a wrong SSID/password typed into the
  // form can be corrected from the same page instead of needing a USB cable.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(ident.mdns_host);
  WiFi.setSleep(false);          // keep the AP + the 1 Hz POSTs responsive
  IPAddress ip(ident.ap_ip[0], ident.ap_ip[1], ident.ap_ip[2], ident.ap_ip[3]);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ident.ap_ssid, ident.ap_pass);

  server.on("/", HTTP_GET, handleIndex);
  server.on("/api/live", HTTP_GET, handleLive);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handleSetConfig);
  server.on("/api/scan", HTTP_GET, handleScan);
  server.on("/api/resend", HTTP_POST, handleResend);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.onNotFound([]() {                       // anything else -> the page
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();

  if (MDNS.begin(ident.mdns_host))
    MDNS.addService("http", "tcp", 80);

  begun = true;
  Serial.printf("[web] config portal up: SSID '%s' pass '%s' -> http://%s/ "
                "(also http://%s.local/ once on the field wi-fi)\n",
                ident.ap_ssid, ident.ap_pass,
                WiFi.softAPIP().toString().c_str(), ident.mdns_host);
  Serial.printf("[web] saved config: field ssid '%s'  pi %s:%u  "
                "arm length %lu mm (seq %lu, %s)\n",
                cfg.ssid.c_str(), cfg.host.c_str(), cfg.port,
                (unsigned long)cfg.len_mm, (unsigned long)cfg.len_seq,
                cfg.len_seq == 0 ? "never set on the web page"
                                 : (cfg.len_ack ? "confirmed by the Pi"
                                                : "NOT yet confirmed"));
}

void fieldcfg_handle() {
  if (!begun) return;
  server.handleClient();
}
