#!/usr/bin/env python3
"""
One-time setup: turn a Raspberry Pi 4 into a CONCURRENT WiFi client + Access Point.

The Pi has a single radio, so we cannot run two independent interfaces on two
channels. The trick is a second *virtual* interface on the same chip:

    wlan0  -> stays the WiFi CLIENT (managed by NetworkManager / dhcpcd as usual)
    uap0   -> a new virtual AP interface with a FIXED IP (192.168.50.1)

hostapd runs the access point on uap0, dnsmasq hands out DHCP leases to whoever
joins (the ESP32), and NetworkManager/dhcpcd is told to keep its hands off uap0.

    Run once, as root:   sudo python3 ap_sta_setup.py
    Then (recommended):  sudo reboot

Prereq: wlan0 should already be joined to your normal WiFi (nmtui / raspi-config)
so the Pi still has internet while it also serves the AP.

NOTE on the one-radio limit: the AP is forced onto the SAME channel wlan0 is
connected on. This script auto-detects that channel; if you later move the Pi to
a different router/channel, re-run it (or edit channel= in the hostapd conf).
"""
import os
import subprocess
import sys

# --- Access point settings (change SSID/pass here) --------------------------- #
AP_SSID    = "RPi_AP"
AP_PASS    = "raspberry123"        # >= 8 chars for WPA2
COUNTRY    = "US"                  # set your 2-letter regulatory domain (GB, DE, OM, ...)
PHY_IFACE  = "wlan0"              # the physical WiFi client interface
AP_IFACE   = "uap0"              # the virtual AP interface we create
AP_IP      = "192.168.50.1"      # <-- the FIXED IP of the Pi on its own AP
AP_CIDR    = "192.168.50.1/24"
DHCP_START = "192.168.50.10"
DHCP_END   = "192.168.50.100"
NETMASK    = "255.255.255.0"
FALLBACK_CHANNEL = 6              # used if wlan0's channel can't be detected

# --- File paths -------------------------------------------------------------- #
HOSTAPD_CONF   = "/etc/hostapd/hostapd.conf"
HOSTAPD_DEFAULT = "/etc/default/hostapd"
DNSMASQ_CONF   = "/etc/dnsmasq.d/uap0.conf"
UP_SCRIPT      = "/usr/local/sbin/ap-sta-up.sh"
AP_STA_SERVICE = "/etc/systemd/system/ap-sta.service"
HOSTAPD_OVERRIDE = "/etc/systemd/system/hostapd.service.d/override.conf"
DNSMASQ_OVERRIDE = "/etc/systemd/system/dnsmasq.service.d/override.conf"
NM_UNMANAGED   = "/etc/NetworkManager/conf.d/unmanaged-uap0.conf"


def run(cmd, check=True):
    """Run a command, echoing it first."""
    print("  $ " + " ".join(cmd))
    env = dict(os.environ, DEBIAN_FRONTEND="noninteractive")
    return subprocess.run(cmd, check=check, env=env)


def is_active(service):
    return subprocess.run(
        ["systemctl", "is-active", "--quiet", service]).returncode == 0


def write_file(path, content, executable=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)
    os.chmod(path, 0o755 if executable else 0o644)
    print(f"  wrote {path}")


def detect_channel():
    """Read wlan0's current channel so the AP lands on the SAME one (one radio)."""
    try:
        out = subprocess.check_output(["iw", "dev", PHY_IFACE, "info"], text=True)
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("channel "):        # "channel 6 (2437 MHz), ..."
                return int(line.split()[1])
    except Exception:
        pass
    print(f"  (!) could not detect {PHY_IFACE} channel; using {FALLBACK_CHANNEL}. "
          f"If the AP misbehaves, set channel= in {HOSTAPD_CONF} to match "
          f"`iw dev {PHY_IFACE} link`.")
    return FALLBACK_CHANNEL


def main():
    if os.geteuid() != 0:
        sys.exit("Run as root:  sudo python3 ap_sta_setup.py")

    print("[1/7] Installing packages (hostapd, dnsmasq, iw, rfkill) ...")
    run(["apt-get", "update"], check=False)
    run(["apt-get", "install", "-y", "hostapd", "dnsmasq", "iw", "rfkill"])

    print("[2/7] Detecting wlan0 channel ...")
    channel = detect_channel()
    print(f"      -> AP channel = {channel}")

    print("[3/7] Writing hostapd / dnsmasq config ...")
    write_file(HOSTAPD_CONF, f"""\
# Access point on the virtual interface uap0 (concurrent with the wlan0 client).
country_code={COUNTRY}
interface={AP_IFACE}
driver=nl80211
ssid={AP_SSID}
hw_mode=g
channel={channel}
ieee80211n=1
wmm_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase={AP_PASS}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
""")

    write_file(HOSTAPD_DEFAULT, 'DAEMON_CONF="/etc/hostapd/hostapd.conf"\n')

    write_file(DNSMASQ_CONF, f"""\
# DHCP for the uap0 access point only. port=0 disables dnsmasq's DNS server so it
# can't clash with systemd-resolved on :53 -- the ESP32 reaches the Pi by IP and
# needs no DNS. bind-dynamic tolerates uap0 appearing after dnsmasq starts.
interface={AP_IFACE}
bind-dynamic
port=0
dhcp-range={DHCP_START},{DHCP_END},{NETMASK},24h
dhcp-option={AP_IFACE},3,{AP_IP}
""")

    print("[4/7] Writing uap0 bring-up script + systemd units ...")
    write_file(UP_SCRIPT, f"""\
#!/bin/bash
# Create + configure the uap0 virtual AP interface (concurrent AP + WiFi client).
# Invoked by ap-sta.service. wlan0 stays the client; uap0 is the AP at {AP_IP}.
set -e
PATH=/usr/sbin:/usr/bin:/sbin:/bin

# Make sure the radio isn't soft-blocked.
rfkill unblock wlan || true

# Create the AP virtual interface on the SAME physical radio as {PHY_IFACE}.
if ! iw dev | grep -qw {AP_IFACE}; then
    iw dev {PHY_IFACE} interface add {AP_IFACE} type __ap
fi

# Bring it up with the FIXED IP.
ip link set {AP_IFACE} up
ip addr flush dev {AP_IFACE}
ip addr add {AP_CIDR} dev {AP_IFACE}
""", executable=True)

    write_file(AP_STA_SERVICE, f"""\
[Unit]
Description=Create {AP_IFACE} virtual AP interface (concurrent AP + WiFi client)
Wants=network.target
After=network.target
Before=hostapd.service dnsmasq.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart={UP_SCRIPT}
ExecStop=/usr/sbin/iw dev {AP_IFACE} del

[Install]
WantedBy=multi-user.target
""")

    # hostapd + dnsmasq must not start until uap0 exists and has its IP.
    write_file(HOSTAPD_OVERRIDE, """\
[Unit]
After=ap-sta.service
Requires=ap-sta.service
""")
    write_file(DNSMASQ_OVERRIDE, """\
[Unit]
After=ap-sta.service
Requires=ap-sta.service
""")

    print("[5/7] Telling the network manager to leave uap0 alone ...")
    if is_active("NetworkManager"):
        write_file(NM_UNMANAGED, f"""\
[keyfile]
unmanaged-devices=interface-name:{AP_IFACE}
""")
        run(["systemctl", "reload", "NetworkManager"], check=False)
    elif os.path.exists("/etc/dhcpcd.conf"):
        with open("/etc/dhcpcd.conf") as f:
            txt = f.read()
        if f"denyinterfaces {AP_IFACE}" not in txt:
            with open("/etc/dhcpcd.conf", "a") as f:
                f.write(f"\n# Concurrent AP+STA: ap-sta.service owns {AP_IFACE}\n"
                        f"denyinterfaces {AP_IFACE}\n")
            print(f"  appended 'denyinterfaces {AP_IFACE}' to /etc/dhcpcd.conf")

    print("[6/7] Enabling services ...")
    run(["rfkill", "unblock", "wlan"], check=False)
    run(["systemctl", "daemon-reload"])
    run(["systemctl", "unmask", "hostapd"])          # hostapd ships masked
    run(["systemctl", "enable", "ap-sta.service", "hostapd.service", "dnsmasq.service"])

    print("[7/7] Starting services ...")
    run(["systemctl", "restart", "ap-sta.service"])
    run(["systemctl", "restart", "hostapd.service"])
    run(["systemctl", "restart", "dnsmasq.service"])

    print("\n" + "=" * 64)
    print("DONE. Concurrent WiFi client + Access Point is up.")
    print(f"  SSID      : {AP_SSID}")
    print(f"  Password  : {AP_PASS}")
    print(f"  Pi AP IP  : {AP_IP}  (fixed)")
    print(f"  DHCP pool : {DHCP_START} - {DHCP_END}")
    print(f"  AP channel: {channel}  (forced to match {PHY_IFACE})")
    print("=" * 64)
    print("Verify:")
    print(f"  iw dev                       # should list {AP_IFACE} (type AP)")
    print(f"  ip addr show {AP_IFACE}             # should show {AP_IP}")
    print( "  sudo systemctl status hostapd dnsmasq ap-sta")
    print(f"  iw dev {PHY_IFACE} link            # confirm still connected as client")
    print("Then start the demo server:")
    print("  pip install flask && python3 ap_demo_server.py")
    print("A reboot is recommended to confirm everything comes up on boot:  sudo reboot")


if __name__ == "__main__":
    main()
