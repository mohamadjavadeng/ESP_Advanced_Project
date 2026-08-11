#!/usr/bin/env bash
# apsta-doctor.sh -- why can't a phone/ESP32 join the Pi's concurrent AP?
#
# READ-ONLY. Changes nothing, starts nothing, stops nothing. Safe to run on a
# live machine over SSH.
#
#   sudo bash apsta-doctor.sh
#
# Prints a PASS/WARN/FAIL line per check, then a ranked list of what to fix.
# Works for both routes: NetworkManager (Route A) and hostapd (Route B).

PATH=/usr/sbin:/usr/bin:/sbin:/bin
export LC_ALL=C

STA=wlan0
AP=uap0
CON=rpi-ap
[ -r /etc/default/rpi-ap-sta ] && . /etc/default/rpi-ap-sta
STA=${STA_IFACE:-$STA}; AP=${AP_IFACE:-$AP}; CON=${AP_CON:-$CON}

FIX=()
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$1"; FIX+=("WARN: $2"); }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FIX+=("FAIL: $2"); }
info() { printf '        %s\n' "$1"; }
head2() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

[ "$(id -u)" -eq 0 ] || { echo "run me with sudo -- some checks need root"; exit 1; }

head2 "0. what we are looking at"
info "station iface : $STA"
info "AP iface      : $AP"
info "NM profile    : $CON"
info "OS            : $(. /etc/os-release; echo "$PRETTY_NAME")"
info "kernel        : $(uname -r)"

# --------------------------------------------------------------------------- #
head2 "1. radio is legally and physically enabled"
reg=$(iw reg get | awk '/^country/{print $2; exit}')
case "$reg" in
    00:|"") fail "regulatory domain is unset ($reg)" \
                 "Set the country: sudo raspi-config nonint do_wifi_country US   (then reboot). An unset domain makes the AP refuse to start or beacon at minimum power your phone will not see." ;;
    *)      pass "regulatory domain = ${reg%:}" ;;
esac

if rfkill list wlan 2>/dev/null | grep -q 'blocked: yes'; then
    fail "the wlan radio is rfkill-blocked" \
         "sudo rfkill unblock wlan   (a soft block often comes back if the country is unset -- fix that too)"
else
    pass "radio not rfkill-blocked"
fi

# --------------------------------------------------------------------------- #
head2 "2. the chip admits it can be a station AND an AP"
if iw list | tr -d ' ' | grep -q 'managed}<=1,#{AP}<=1'; then
    pass "driver advertises a managed + AP combination"
    ch=$(iw list | tr -d ' ' | grep -A2 'managed}<=1,#{AP}<=1' | grep -o '#channels<=[0-9]' | head -1)
    info "the same combination line says ${ch:-#channels<=1} -- that is the shared-channel limit"
else
    fail "no interface combination allows managed + AP together" \
         "This radio/firmware cannot do concurrency. No software fix -- use a USB Wi-Fi dongle (Route C). Check with: iw list | grep -A8 'valid interface combinations'"
fi

# --------------------------------------------------------------------------- #
head2 "3. the AP interface exists and is distinct from the station"
if ip link show "$AP" &>/dev/null; then
    pass "$AP exists"
    t=$(iw dev "$AP" info 2>/dev/null | awk '/type/{print $2}')
    [ "$t" = "AP" ] && pass "$AP is type AP" \
        || warn "$AP is type '${t:-unknown}', not AP" \
                "Normal while NetworkManager is still bringing it up. If it stays this way: sudo systemctl restart $AP.service && sudo nmcli con up $CON"

    m1=$(cat "/sys/class/net/$STA/address" 2>/dev/null)
    m2=$(cat "/sys/class/net/$AP/address" 2>/dev/null)
    if [ "$m1" = "$m2" ]; then
        fail "$AP and $STA share one MAC ($m1)" \
             "Two netdevs on one MAC breaks ARP and DHCP for AP clients -- devices associate then fail. The old ap_sta_setup.py did not flip the MAC. Fix: sudo systemctl restart $AP.service (the guide's uap0-up.sh flips the locally-administered bit)."
    else
        pass "distinct MACs ($STA $m1 / $AP $m2)"
    fi

    ip -br link show "$AP" | grep -q 'UP' \
        && pass "$AP is UP" \
        || fail "$AP is DOWN" "sudo ip link set $AP up"

    v4=$(ip -4 -br addr show "$AP" | awk '{print $3}')
    [ -n "$v4" ] && pass "$AP has address $v4" \
        || fail "$AP has no IPv4 address" \
                "Route A: the NM profile assigns it -- see check 4. Route B: sudo ip addr add 192.168.50.1/24 dev $AP"
else
    fail "$AP does not exist" \
         "Nothing is on the air. Check the startup unit: systemctl status $AP.service ; journalctl -u $AP.service -b. If it logged 'Device or resource busy (-16)', the radio was busy with an association -- reboot so the unit runs before NetworkManager."
fi

# --------------------------------------------------------------------------- #
head2 "4. which stack is supposed to be driving the AP"
nm=no; hp=no
systemctl is-active --quiet NetworkManager && nm=yes
systemctl is-active --quiet hostapd && hp=yes
info "NetworkManager active: $nm    hostapd active: $hp"

if [ "$nm" = yes ] && [ "$hp" = yes ]; then
    warn "both NetworkManager and hostapd are running" \
         "Only one may own $AP. Pick a route: disable hostapd (Route A) with 'sudo systemctl disable --now hostapd dnsmasq', or mark $AP unmanaged in NM (Route B)."
fi

# the classic silent killer
u=$(grep -rl "interface-name:$AP" /etc/NetworkManager/conf.d/ 2>/dev/null)
if [ -n "$u" ] && [ "$hp" = no ]; then
    fail "NetworkManager is told to ignore $AP, but hostapd is not running" \
         "This is the #1 cause of a completely invisible SSID after migrating from ap_sta_setup.py. Nothing owns the interface. Fix: sudo rm $u && sudo systemctl reload NetworkManager && sudo nmcli con up $CON"
elif [ -n "$u" ]; then
    info "NM is set to ignore $AP (expected on Route B): $u"
fi

if [ "$nm" = yes ]; then
    st=$(nmcli -g GENERAL.STATE connection show "$CON" 2>/dev/null)
    if [ -z "$st" ]; then
        [ "$hp" = no ] && fail "no NetworkManager profile named '$CON'" \
            "You never created it, or it was deleted. Redo Route A steps 12-14."
    elif [ "$st" = activated ]; then
        pass "profile '$CON' is activated"
        info "mode=$(nmcli -g 802-11-wireless.mode con show "$CON")  ssid=$(nmcli -g 802-11-wireless.ssid con show "$CON")  band=$(nmcli -g 802-11-wireless.band con show "$CON")  ch=$(nmcli -g 802-11-wireless.channel con show "$CON")"
        d=$(nmcli -g GENERAL.DEVICES connection show "$CON")
        [ "$d" = "$AP" ] && pass "bound to $AP" \
            || fail "profile is on device '${d:-none}', not $AP" \
                    "A profile bound to $STA will destroy your station link. sudo nmcli con modify $CON connection.interface-name $AP"
        [ "$(nmcli -g ipv4.method con show "$CON")" = shared ] \
            && pass "ipv4.method=shared (DHCP + NAT provided by NM)" \
            || warn "ipv4.method is not 'shared'" \
                    "Without it AP clients get no DHCP lease and no internet. sudo nmcli con modify $CON ipv4.method shared ipv4.addresses 192.168.50.1/24"
        psk=$(nmcli -s -g 802-11-wireless-security.psk con show "$CON")
        [ ${#psk} -ge 8 ] && pass "PSK length ${#psk} (WPA2 needs 8-63)" \
            || fail "PSK length ${#psk} is invalid" "sudo nmcli con modify $CON wifi-sec.psk 'at-least-8-chars'"
    else
        fail "profile '$CON' state is '$st', not activated" \
             "sudo nmcli con up $CON   then read the reason: journalctl -u NetworkManager -b | tail -40"
    fi
fi

if [ "$hp" = yes ]; then
    pass "hostapd is running"
    c=$(awk -F= '/^channel=/{print $2}' /etc/hostapd/hostapd.conf 2>/dev/null)
    s=$(awk -F= '/^ssid=/{print $2}' /etc/hostapd/hostapd.conf 2>/dev/null)
    i=$(awk -F= '/^interface=/{print $2}' /etc/hostapd/hostapd.conf 2>/dev/null)
    info "hostapd.conf: interface=$i ssid=$s channel=$c"
    [ "$i" = "$AP" ] || fail "hostapd.conf points at '$i', not $AP" "Fix interface= in /etc/hostapd/hostapd.conf"
elif [ "$nm" = no ]; then
    fail "neither NetworkManager nor hostapd is running" "Nothing can serve an AP. Start one of them."
fi

# --------------------------------------------------------------------------- #
head2 "5. THE big one: both roles must share one channel"
sch=$(iw dev "$STA" info 2>/dev/null | awk '$1=="channel"{print $2}')
ach=$(iw dev "$AP"  info 2>/dev/null | awk '$1=="channel"{print $2}')
ssid=$(iw dev "$STA" link 2>/dev/null | awk '/SSID/{$1="";print substr($0,2)}')
info "station: ${ssid:-<not associated>} on channel ${sch:-none}"
info "AP     : channel ${ach:-none}"

if [ -z "$sch" ]; then
    warn "$STA is not associated with anything" \
         "Not fatal for the AP, but you lose internet and NAT has no upstream. sudo nmcli dev wifi connect 'SSID' password 'PW' ifname $STA"
elif [ "$sch" -gt 14 ] 2>/dev/null; then
    fail "$STA is on 5 GHz (channel $sch)" \
         "A concurrent 2.4 GHz AP cannot share a 5 GHz channel -- this alone can stop the AP appearing. Force your router or phone hotspot to 2.4 GHz, or use a USB dongle (Route C)."
elif [ -n "$ach" ] && [ "$sch" != "$ach" ]; then
    fail "channel mismatch: station on $sch, AP on $ach" \
         "One radio cannot hold two channels. Clients see a beacon and then fail to associate, with nothing logged. Fix now: sudo /usr/local/sbin/ap-channel-follow.sh ; and make sure its timer is enabled: systemctl is-enabled ap-channel-follow.timer"
elif [ -n "$ach" ]; then
    pass "station and AP are both on channel $sch"
fi

if [ -n "$ssid" ]; then
    info ""
    info "NOTE: if the phone you are testing with is ALSO the hotspot '$ssid'"
    info "      that $STA is joined to, it cannot be your upstream AP and a client"
    info "      of the Pi at the same time. Test with a different device."
fi

# --------------------------------------------------------------------------- #
head2 "6. is anything actually reaching the AP"
sd=$(iw dev "$AP" station dump 2>/dev/null | grep -c '^Station')
if [ "${sd:-0}" -gt 0 ]; then
    pass "$sd client(s) associated"
    iw dev "$AP" station dump | grep -E '^Station|signal:|rx bitrate' | sed 's/^/        /'
else
    info "no clients associated right now"
fi

echo
info "recent association / auth attempts (a phone trying and failing shows here):"
journalctl -b --no-pager -u NetworkManager -u hostapd -u wpa_supplicant 2>/dev/null \
  | grep -iE 'associat|authenticat|deauth|disassoc|AP-STA|handshake|WPA|psk' \
  | tail -12 | sed 's/^/        /' || true

# --------------------------------------------------------------------------- #
head2 "7. DHCP -- 'connected, no IP' lives here"
if pgrep -af "dnsmasq.*$AP" >/dev/null; then
    pass "a dnsmasq instance is bound to $AP"
    pgrep -af "dnsmasq.*$AP" | sed 's/^/        /'
else
    if [ "$nm" = yes ] && [ "$(nmcli -g ipv4.method con show "$CON" 2>/dev/null)" = shared ]; then
        fail "ipv4.method=shared but no dnsmasq is serving $AP" \
             "NM needs the dnsmasq-base package to run its internal DHCP. sudo apt install -y dnsmasq-base ; sudo nmcli con up $CON"
    else
        warn "nothing is serving DHCP on $AP" \
             "Route A: set ipv4.method shared. Route B: check /etc/dnsmasq.d/$AP.conf and 'systemctl status dnsmasq'."
    fi
fi

if systemctl is-active --quiet dnsmasq && [ "$hp" = no ]; then
    fail "the standalone dnsmasq.service is running alongside NetworkManager" \
         "Two DHCP/DNS daemons fight over :53 and $AP. On Route A: sudo systemctl disable --now dnsmasq"
fi

for f in /var/lib/NetworkManager/dnsmasq-$AP.leases /var/lib/misc/dnsmasq.leases; do
    [ -s "$f" ] && { info "leases in $f:"; sed 's/^/        /' "$f"; }
done

# --------------------------------------------------------------------------- #
head2 "8. internet for AP clients (only matters once they associate)"
[ "$(cat /proc/sys/net/ipv4/ip_forward)" = 1 ] \
    && pass "ip_forward is on" \
    || warn "ip_forward is off" "Route A sets this via ipv4.method=shared. Route B: echo 'net.ipv4.ip_forward=1' | sudo tee /etc/sysctl.d/99-ap-sta-forward.conf && sudo sysctl --system"

if iptables -t nat -S 2>/dev/null | grep -q MASQUERADE || nft list ruleset 2>/dev/null | grep -q masquerade; then
    pass "a masquerade/NAT rule exists"
else
    warn "no NAT rule found" "AP clients will get an IP but no internet. Route A: ipv4.method=shared installs this. Route B: see the NAT block in the guide."
fi

# --------------------------------------------------------------------------- #
head2 "9. will it survive a reboot"
for u in "$AP.service" ap-channel-follow.timer; do
    s=$(systemctl is-enabled "$u" 2>/dev/null)
    [ "$s" = enabled ] && pass "$u is enabled" \
        || warn "$u is '${s:-not-found}'" "sudo systemctl enable $u   -- otherwise this all disappears at the next power cut"
done

# --------------------------------------------------------------------------- #
printf '\n\033[1m== verdict\033[0m\n'
if [ ${#FIX[@]} -eq 0 ]; then
    echo "  Nothing wrong found on the Pi side."
    echo
    echo "  If a phone still will not join, in order of likelihood:"
    echo "   1. You are testing with the same phone whose hotspot $STA is using."
    echo "      One device cannot be both. Try another phone or a laptop."
    echo "   2. The phone cached a wrong password. Forget the network and retry."
    echo "   3. Android/iOS deprioritise a network with no internet -- confirm NAT"
    echo "      works: from a joined client, ping 1.1.1.1"
    echo "   4. Move the phone within a metre or two; a concurrent AP shares"
    echo "      airtime and beacons weaker than a dedicated router."
else
    printf '  %d issue(s), most important first:\n\n' "${#FIX[@]}"
    n=0
    for f in "${FIX[@]}"; do
        case $f in FAIL:*) n=$((n+1)); printf '  %d. %s\n\n' "$n" "${f#FAIL: }";; esac
    done
    for f in "${FIX[@]}"; do
        case $f in WARN:*) n=$((n+1)); printf '  %d. (minor) %s\n\n' "$n" "${f#WARN: }";; esac
    done
fi
