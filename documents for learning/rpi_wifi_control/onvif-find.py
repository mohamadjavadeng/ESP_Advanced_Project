#!/usr/bin/env python3
"""
onvif-find.py -- find IP cameras on the Pi's access point and print their NAMES.

This is the one discovery method that returns a camera's own advertised identity
rather than just a MAC address. ONVIF devices answer a WS-Discovery Probe sent to
UDP multicast 239.255.255.250:3702, and their reply carries "scopes" that normally
include the model and a friendly name -- which is where a label like "H8c-xxxxxx"
actually shows up on the wire.

    sudo python3 onvif-find.py                    # probe out of 192.168.50.1
    sudo python3 onvif-find.py 192.168.50.1       # explicit source address
    sudo python3 onvif-find.py 192.168.50.1 8     # ...and wait 8s instead of 5

Standard library only. Sends 3 probes (UDP is lossy and cheap cameras drop the
first one), then prints one block per responder.

If nothing answers: not every cheap camera implements ONVIF, and some only enable
it after you tick a box in their phone app. Fall back to
`sudo bash ap-clients.sh --find`, which identifies a device by power-cycling it
and is never wrong.
"""

import re
import socket
import struct
import sys
import uuid

MCAST_ADDR = "239.255.255.250"
MCAST_PORT = 3702
PROBES = 3

PROBE_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<e:Envelope xmlns:e="http://www.w3.org/2003/05/soap-envelope"
            xmlns:w="http://schemas.xmlsoap.org/ws/2004/08/addressing"
            xmlns:d="http://schemas.xmlsoap.org/ws/2005/04/discovery"
            xmlns:dn="http://www.onvif.org/ver10/network/wsdl">
  <e:Header>
    <w:MessageID>uuid:{msgid}</w:MessageID>
    <w:To e:mustUnderstand="true">urn:schemas-xmlsoap-org:ws:2005:04:discovery</w:To>
    <w:Action e:mustUnderstand="true">http://schemas.xmlsoap.org/ws/2005/04/discovery/Probe</w:Action>
  </e:Header>
  <e:Body>
    <d:Probe><d:Types>{types}</d:Types></d:Probe>
  </e:Body>
</e:Envelope>"""

# NetworkVideoTransmitter first (cameras), then an empty Types (anything ONVIF,
# including NVRs and doorbells that do not claim the video-transmitter type).
TYPE_SETS = ["dn:NetworkVideoTransmitter", ""]


def build_probe(types):
    return PROBE_TEMPLATE.format(msgid=uuid.uuid4(), types=types).encode()


def tag(xml, name):
    """All text values of <ns:name>...</ns:name>, namespace-agnostic."""
    return [m.strip() for m in
            re.findall(r"<[^:>]*:?%s[^>]*>(.*?)</[^:>]*:?%s>" % (name, name),
                       xml, re.S | re.I)]


def parse_scopes(scope_blob):
    """
    Scopes look like:
      onvif://www.onvif.org/name/IPCAM  onvif://www.onvif.org/hardware/H8c
      onvif://www.onvif.org/location/china  onvif://www.onvif.org/Profile/S
    Turn that into {"name": "IPCAM", "hardware": "H8c", ...}.
    """
    out = {}
    for s in scope_blob.split():
        m = re.match(r"onvif://[^/]+/([^/]+)/?(.*)", s.strip())
        if not m:
            continue
        k, v = m.group(1).lower(), m.group(2)
        if k in out and v:
            out[k] += "," + v
        else:
            out[k] = v
    return out


def main():
    # --map prints one tab-separated "ip<TAB>name<TAB>hardware" line per camera and
    # nothing else, so ap-clients.sh can read it. Everything else stays human-facing.
    argv = [a for a in sys.argv[1:] if a != "--map"]
    machine = "--map" in sys.argv

    src = argv[0] if len(argv) > 0 else "192.168.50.1"
    wait = float(argv[1]) if len(argv) > 1 else 5.0

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    try:
        # send the probe out of the AP interface specifically, not the default route
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                     socket.inet_aton(src))
        s.bind((src, 0))
    except OSError as e:
        if not machine:
            print(f"cannot bind to {src}: {e}")
            print("Pass the AP interface's own address, e.g.: sudo python3 onvif-find.py 192.168.50.1")
        return 1

    if not machine:
        print(f"probing {MCAST_ADDR}:{MCAST_PORT} from {src}, listening {wait:.0f}s ...")
    for types in TYPE_SETS:
        for _ in range(PROBES):
            try:
                s.sendto(build_probe(types), (MCAST_ADDR, MCAST_PORT))
            except OSError as e:
                if not machine:
                    print(f"send failed: {e}")
                return 1

    s.settimeout(wait)
    seen = {}
    while True:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            break
        xml = data.decode("utf-8", "replace")
        if "ProbeMatch" not in xml:
            continue
        ip = addr[0]
        xaddrs = " ".join(tag(xml, "XAddrs"))
        scopes = parse_scopes(" ".join(tag(xml, "Scopes")))
        urn = (tag(xml, "Address") or [""])[0]
        prev = seen.get(ip, {})
        # merge: the two probe types often return complementary detail
        prev.setdefault("xaddrs", xaddrs)
        prev.setdefault("urn", urn)
        prev["scopes"] = {**prev.get("scopes", {}), **scopes}
        seen[ip] = prev

    if machine:
        for ip, d in sorted(seen.items()):
            sc = d["scopes"]
            print("%s\t%s\t%s" % (ip, sc.get("name", ""), sc.get("hardware", "")))
        return 0 if seen else 2

    if not seen:
        print("\nno ONVIF responders.")
        print("Cheap cameras often ship with ONVIF off -- enable it in the vendor app,")
        print("or identify the device by power-cycling it:  sudo bash ap-clients.sh --find")
        return 2

    print(f"\n{len(seen)} ONVIF device(s):\n")
    for ip, d in sorted(seen.items()):
        sc = d["scopes"]
        name = sc.get("name") or "(no name advertised)"
        print(f"  {ip}")
        print(f"    name       : {name}")
        for key in ("hardware", "manufacturer", "location", "profile", "type"):
            if sc.get(key):
                print(f"    {key:<11}: {sc[key]}")
        if d.get("urn"):
            print(f"    urn        : {d['urn']}")
        if d.get("xaddrs"):
            print(f"    device svc : {d['xaddrs']}")
            print(f"    rtsp guess : rtsp://{ip}:554/  (ask the device: GetStreamUri)")
        leftover = {k: v for k, v in sc.items()
                    if k not in ("name", "hardware", "manufacturer", "location",
                                 "profile", "type")}
        if leftover:
            print(f"    other      : {leftover}")
        print()

    print("Match the 'name' or 'hardware' field against the label on your camera.")
    print("Then pair the IP with its MAC:  ip neigh show dev uap0 | grep <ip>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
