#!/usr/bin/env bash
# ap-clients.sh -- list every device on the Pi's access point.
#
# Joins four sources the kernel and DHCP server already keep, so nothing has to
# be guessed:
#   iw station dump   802.11 associations   -> MAC, signal, connected time
#   DHCP lease file   who asked for an IP   -> MAC, IP, hostname the client sent
#   ip neigh (ARP)    who is actually alive -> IP <-> MAC, incl. static-IP devices
#   IEEE OUI table    offline vendor lookup -> "Hangzhou ...", "Espressif", ...
#
#   sudo bash ap-clients.sh              # the table
#   sudo bash ap-clients.sh --ports      # + probe service ports, classify device type
#   sudo bash ap-clients.sh --find       # ** find one device by power-cycling it **
#   sudo bash ap-clients.sh --watch      # re-print every 3s
#
# Look a device up by name -- prints ONLY the IP, so it can be used in scripts:
#   sudo bash ap-clients.sh --ip H8c                 # -> 192.168.50.42
#   sudo bash ap-clients.sh --ip 'H8c-*' --onvif     # ask the cameras their names
#   sudo bash ap-clients.sh --show H8c               # the full row instead of just the IP
#   sudo bash ap-clients.sh --json                   # every device, machine-readable
#   sudo bash ap-clients.sh --alias 192.168.50.42 gate-cam    # label it permanently
#
# A name is matched, case-insensitively, against all of these -- most trustworthy
# first. '*' in the pattern is a wildcard; anything else is a plain substring.
#   1. your own alias from /etc/rpi-ap-sta-hosts   (you set it; it never lies)
#   2. the ONVIF name the camera advertises        (--onvif; adds ~5s)
#   3. the DHCP hostname the device asked for      (often blank on cheap gear)
#   4. the IEEE OUI vendor string
#   5. the MAC or the IP itself
#
# --find is the only method that is 100% certain about which IP is which physical
# box: it takes a baseline, you power-cycle the device, and it reports exactly
# which MAC/IP left and came back. Names and vendors can be absent or lie. Once
# --find has told you the MAC, record it with --alias and every later lookup is
# exact regardless of DHCP or ONVIF.

PATH=/usr/sbin:/usr/bin:/sbin:/bin
export LC_ALL=C

STA_IFACE=wlan0
AP_IFACE=uap0
AP_CON=rpi-ap
[ -r /etc/default/rpi-ap-sta ] && . /etc/default/rpi-ap-sta

ALIAS_FILE=/etc/rpi-ap-sta-hosts
declare -A ALIAS=() ONVIF_NAME=()

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \?//'
}

MODE=table; PATTERN=""; ALIAS_NAME=""; USE_ONVIF=0
while [ $# -gt 0 ]; do
    case $1 in
        --table)   MODE=table ;;
        --ports)   MODE=ports ;;
        --find)    MODE=find ;;
        --watch)   MODE=watch ;;
        --json)    MODE=json ;;
        --onvif)   USE_ONVIF=1 ;;
        --ip)      MODE=ip;    PATTERN=${2:-}; shift ;;
        --show)    MODE=show;  PATTERN=${2:-}; shift ;;
        --alias)   MODE=alias; PATTERN=${2:-}; ALIAS_NAME=${3:-}; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
    shift
done

case $MODE in
    ip|show) [ -n "$PATTERN" ] || { echo "$MODE needs a name pattern, e.g. --$MODE H8c" >&2; exit 1; } ;;
    alias)   [ -n "$PATTERN" ] && [ -n "$ALIAS_NAME" ] \
               || { echo "usage: --alias <mac|ip> <name>" >&2; exit 1; } ;;
esac

[ "$(id -u)" -eq 0 ] || { echo "run me with sudo (station dump and port probes need root)" >&2; exit 1; }
ip link show "$AP_IFACE" &>/dev/null || { echo "no such interface: $AP_IFACE" >&2; exit 1; }

SUBNET=$(ip -4 -o addr show "$AP_IFACE" | awk '{print $4}' | head -1)
PREFIX=${SUBNET%.*/*}                      # 192.168.50.1/24 -> 192.168.50

# --------------------------------------------------------------------------- #
# where the DHCP server keeps its leases -- NetworkManager's internal instance,
# or a standalone dnsmasq if you built the hostapd route
lease_file() {
    for f in "/var/lib/NetworkManager/dnsmasq-$AP_IFACE.leases" \
             /var/lib/misc/dnsmasq.leases \
             /var/lib/dhcp/dhcpd.leases; do
        [ -s "$f" ] && { echo "$f"; return; }
    done
}
LEASES=$(lease_file)

# offline MAC -> vendor. No internet lookup, no rate limits.
vendor_of() {
    local m p v b
    m=${1//:/}; p=$(printf '%s' "${m:0:6}" | tr 'a-f' 'A-F')
    v=""
    if [ -r /usr/share/nmap/nmap-mac-prefixes ]; then
        v=$(awk -v p="$p" '$1==p{$1="";sub(/^ /,"");print;exit}' /usr/share/nmap/nmap-mac-prefixes)
    fi
    if [ -z "$v" ] && [ -r /usr/share/ieee-data/oui.txt ]; then
        v=$(awk -v p="$p" '$1==p && $2=="(base"{for(i=4;i<=NF;i++)printf "%s%s",$i,(i<NF?" ":"");print "";exit}' \
              /usr/share/ieee-data/oui.txt)
    fi
    # A locally-administered MAC (bit 1 of the first octet) is not a real vendor
    # address -- modern phones randomise it per network for privacy.
    b=$(( 0x${p:0:2} ))
    [ -z "$v" ] && (( b & 2 )) && v="(randomised MAC)"
    printf '%s' "${v:-unknown}"
}

# Populate the ARP table so silent devices show up. Prefer a real scanner if the
# machine has one; otherwise fan out pings in bounded batches.
warm_arp() {
    if command -v arp-scan >/dev/null; then
        arp-scan -q -l -I "$AP_IFACE" &>/dev/null
    elif command -v nmap >/dev/null; then
        nmap -sn -n --host-timeout 2s "$PREFIX.0/24" &>/dev/null
    else
        local i n=0
        for i in $(seq 2 254); do
            ping -c1 -W1 -n -I "$AP_IFACE" "$PREFIX.$i" &>/dev/null &
            n=$((n+1)); (( n % 40 == 0 )) && wait
        done
        wait
    fi
}

port_open() { timeout 1 bash -c "exec 3<>/dev/tcp/$1/$2" &>/dev/null; }

# --------------------------------------------------------------------------- #
# Names you set yourself. One device per line:
#     aa:bb:cc:dd:ee:ff   gate-cam        # anything after '#' is a comment
load_alias() {
    ALIAS=()
    [ -r "$ALIAS_FILE" ] || return 0
    local m n
    while read -r m n _; do
        case ${m:-} in ''|\#*) continue ;; esac
        [ -n "${n:-}" ] || continue
        ALIAS[$(printf '%s' "$m" | tr 'A-F' 'a-f')]=$n
    done < "$ALIAS_FILE"
}

# Ask the cameras on this AP what they call themselves (ONVIF WS-Discovery).
# Opt-in because it costs ~5s of waiting for UDP replies.
load_onvif() {
    ONVIF_NAME=()
    local script; script="$(dirname "$(readlink -f "$0")")/onvif-find.py"
    if [ ! -r "$script" ]; then
        echo "note: onvif-find.py not found next to this script; skipping --onvif" >&2
        return 0
    fi
    local ip name hw
    while IFS=$'\t' read -r ip name hw; do
        [ -n "${ip:-}" ] || continue
        ONVIF_NAME[$ip]="${name:-} ${hw:-}"
    done < <(python3 "$script" "${PREFIX}.1" --map 2>/dev/null)
}

# Best available name for a device, most trustworthy source first.
name_of() {
    local m=$1 ip=$2 n
    n=${ALIAS[$m]:-}
    [ -z "$n" ] && n=$(printf '%s' "${ONVIF_NAME[$ip]:-}" | tr -s ' ' | sed 's/^ *//;s/ *$//')
    [ -z "$n" ] && n=${LNAME[$m]:-}
    printf '%s' "${n:--}"
}

# Does this device match the user's pattern? A '*' makes it a wildcard; without
# one it is a plain case-insensitive substring, which is what people expect when
# they type a device name.
matches() {
    local m=$1 ip=$2 pat=$3 hay
    hay="${ALIAS[$m]:-} ${ONVIF_NAME[$ip]:-} ${LNAME[$m]:-} $(vendor_of "$m") $m $ip"
    if [[ $pat == *'*'* ]]; then
        printf '%s' "$hay" | grep -qiE -- "${pat//\*/.*}"
    else
        printf '%s' "$hay" | grep -qiF -- "$pat"
    fi
}

# Escape for a JSON string. Pure bash -- the patterns are quoted so bash treats
# them literally instead of re-interpreting the backslashes. A device chooses its
# own DHCP hostname, so this is untrusted input and must not be able to break the
# output document.
json_esc() {
    local s=$1 bs='\' q='"'
    s=${s//"$bs"/"$bs$bs"}
    s=${s//"$q"/"$bs$q"}
    printf '%s' "$s"
}

# Guess what a device is from the ports it answers on. Ordered most-specific first.
classify() {
    local ip=$1 hits=$2
    case " $hits " in
        *" 554 "*)                        echo "IP CAMERA (RTSP)"; return ;;
        *" 34567 "*|*" 37777 "*|*" 8899 "*) echo "IP CAMERA / DVR (XM-Dahua proto)"; return ;;
        *" 8000 "*)                       echo "IP camera? (Hikvision SDK port)"; return ;;
        *" 62078 "*)                      echo "iPhone / iPad"; return ;;
        *" 22 "*)                         echo "Linux/SSH host"; return ;;
        *" 80 "*|*" 8080 "*)              echo "has a web UI"; return ;;
    esac
    [ -n "${hits// /}" ] && echo "open: $hits" || echo "-"
}

# --------------------------------------------------------------------------- #
collect() {
    # associated MACs with radio info
    declare -gA SIG=() UPTIME=()
    local mac=""
    while read -r k v _; do
        case $k in
            Station) mac=$(printf '%s' "$v" | tr 'A-F' 'a-f') ;;
            signal:) [ -n "$mac" ] && SIG[$mac]=$v ;;
            connected) : ;;
        esac
    done < <(iw dev "$AP_IFACE" station dump 2>/dev/null)

    # 'connected time' needs its own pass, the key is two words
    mac=""
    while IFS= read -r line; do
        case $line in
            Station*) mac=$(printf '%s' "$line" | awk '{print tolower($2)}') ;;
            # 'connected time:\t1234 seconds' -> field 3 is the number, not 4
            *"connected time"*) [ -n "$mac" ] && UPTIME[$mac]=$(printf '%s' "$line" | awk '{print $3}') ;;
        esac
    done < <(iw dev "$AP_IFACE" station dump 2>/dev/null)

    # DHCP leases: MAC -> IP and MAC -> hostname the client asked to be called
    declare -gA LIP=() LNAME=()
    if [ -n "$LEASES" ] && [ -r "$LEASES" ]; then
        while read -r _ m i n _; do
            [ -z "${m:-}" ] && continue
            m=$(printf '%s' "$m" | tr 'A-F' 'a-f')
            LIP[$m]=$i
            [ "${n:-*}" != "*" ] && LNAME[$m]=$n
        done < "$LEASES"
    fi

    # ARP: covers devices with a static IP that never asked for a lease
    #
    # Do not add `dev $AP_IFACE` here. With a dev filter iproute2 drops the
    # 'dev <iface>' pair from each line, so the MAC lands in field 3 rather than
    # field 5 -- the old positional read looked at field 5, matched nothing, and
    # left AIP permanently empty. Every address then came from the DHCP lease file
    # alone, which is exactly the source that cannot see a static-IP camera. Read
    # the unfiltered output and pick the interface out of it instead.
    declare -gA AIP=()
    while read -r i _ dev _ m _; do
        [ "$dev" = "$AP_IFACE" ] || continue
        case $m in ??:??:??:??:??:??) AIP[$(printf '%s' "$m" | tr 'A-F' 'a-f')]=$i ;; esac
    done < <(ip -4 neigh show 2>/dev/null)   # -4: a link-local IPv6 neighbour
                                             # shares the MAC and would clobber
                                             # the v4 address we actually want
}

print_table() {
    local n=0
    printf '\n\033[1m%-16s %-18s %-24s %-16s %7s %9s\033[0m\n' \
           IP MAC VENDOR NAME SIGNAL "UP(s)"
    printf '%s\n' "----------------------------------------------------------------------------------------------------"

    # union of every MAC we saw anywhere
    local macs
    macs=$(printf '%s\n' "${!SIG[@]}" "${!LIP[@]}" "${!AIP[@]}" | grep -v '^$' | sort -u)
    local m ip
    for m in $macs; do
        ip=${AIP[$m]:-${LIP[$m]:-}}
        n=$((n+1))
        printf '%-16s %-18s %-24s %-16s %7s %9s\n' \
               "${ip:-<no IP>}" "$m" "$(vendor_of "$m" | cut -c1-24)" \
               "$(name_of "$m" "$ip" | cut -c1-16)" "${SIG[$m]:--}" "${UPTIME[$m]:--}"
    done
    printf '%s\n' "----------------------------------------------------------------------------------------------------"
    printf '%d device(s) on %s (%s)   leases: %s\n' \
           "$n" "$AP_IFACE" "$SUBNET" "${LEASES:-none found}"
    printf 'NAME = your alias > ONVIF name (--onvif) > DHCP hostname.  Label one: --alias <ip> <name>\n'
}

# every MAC seen in any source, deduplicated
all_macs() {
    printf '%s\n' "${!SIG[@]}" "${!LIP[@]}" "${!AIP[@]}" | grep -v '^$' | sort -u
}

# --ip: print ONLY matching IPs, one per line, so it composes in scripts.
# Exit 0 if something matched, 1 if not -- so `if ip=$(... --ip cam); then` works.
print_ip() {
    local m ip found=0
    for m in $(all_macs); do
        ip=${AIP[$m]:-${LIP[$m]:-}}
        [ -n "$ip" ] || continue
        if matches "$m" "$ip" "$PATTERN"; then printf '%s\n' "$ip"; found=1; fi
    done
    if [ "$found" -eq 0 ]; then
        {
            echo "no device on $AP_IFACE matched '$PATTERN'."
            echo "Names come from: your alias file ($ALIAS_FILE), ONVIF (--onvif),"
            echo "the DHCP hostname, and the OUI vendor. Cheap cameras often send none"
            echo "of them -- identify it once with '--find', then label it with '--alias'."
        } >&2
        return 1
    fi
}

# --show: the same match, but the whole row, for when you want to eyeball it
print_show() {
    local m ip found=0
    printf '%-16s %-18s %-24s %-18s %7s\n' IP MAC VENDOR NAME SIGNAL
    for m in $(all_macs); do
        ip=${AIP[$m]:-${LIP[$m]:-}}
        matches "$m" "$ip" "$PATTERN" || continue
        found=1
        printf '%-16s %-18s %-24s %-18s %7s\n' \
               "${ip:-<no IP>}" "$m" "$(vendor_of "$m" | cut -c1-24)" \
               "$(name_of "$m" "$ip")" "${SIG[$m]:--}"
    done
    [ "$found" -eq 1 ] || { echo "nothing matched '$PATTERN'" >&2; return 1; }
}

print_json() {
    local m ip first=1
    printf '['
    for m in $(all_macs); do
        ip=${AIP[$m]:-${LIP[$m]:-}}
        [ $first -eq 1 ] && first=0 || printf ','
        printf '\n  {"ip":"%s","mac":"%s","vendor":"%s","name":"%s","dhcp_name":"%s","alias":"%s","onvif":"%s","signal_dbm":"%s","uptime_s":"%s"}' \
            "$(json_esc "$ip")" "$(json_esc "$m")" "$(json_esc "$(vendor_of "$m")")" \
            "$(json_esc "$(name_of "$m" "$ip")")" "$(json_esc "${LNAME[$m]:-}")" \
            "$(json_esc "${ALIAS[$m]:-}")" "$(json_esc "${ONVIF_NAME[$ip]:-}")" \
            "$(json_esc "${SIG[$m]:-}")" "$(json_esc "${UPTIME[$m]:-}")"
    done
    printf '\n]\n'
}

# --alias <mac|ip> <name>: write the label down so every later lookup is exact.
do_alias() {
    local target=$1 name=$2 mac="" m
    case $target in
        ??:??:??:??:??:??) mac=$(printf '%s' "$target" | tr 'A-F' 'a-f') ;;
        *.*.*.*)
            for m in $(all_macs); do
                [ "${AIP[$m]:-${LIP[$m]:-}}" = "$target" ] && { mac=$m; break; }
            done
            [ -n "$mac" ] || { echo "no device on $AP_IFACE currently has IP $target" >&2; return 1; } ;;
        *) echo "--alias takes a MAC or an IP, not '$target'" >&2; return 1 ;;
    esac

    touch "$ALIAS_FILE"
    if grep -qi "^$mac" "$ALIAS_FILE"; then
        sed -i "s|^$mac.*|$mac  $name|I" "$ALIAS_FILE"
        echo "updated: $mac -> $name"
    else
        [ -s "$ALIAS_FILE" ] || printf '# MAC                friendly-name\n' >> "$ALIAS_FILE"
        printf '%s  %s\n' "$mac" "$name" >> "$ALIAS_FILE"
        echo "added: $mac -> $name"
    fi
    echo "$ALIAS_FILE now:"; sed 's/^/    /' "$ALIAS_FILE"
    echo
    echo "From now on:  sudo bash $0 --ip $name"
}

probe_ports() {
    local ports="80 443 554 8000 8080 8081 8554 8899 9000 22 23 37777 34567 62078"
    printf '\n\033[1mservice probe (%s)\033[0m\n' "$ports"
    printf '%-16s %-18s %-34s %s\n' IP MAC "OPEN PORTS" "LIKELY"
    printf '%s\n' "----------------------------------------------------------------------------------------------------"
    local m ip p hits
    for m in $(printf '%s\n' "${!AIP[@]}" "${!LIP[@]}" | grep -v '^$' | sort -u); do
        ip=${AIP[$m]:-${LIP[$m]:-}}
        [ -z "$ip" ] && continue
        hits=""
        for p in $ports; do port_open "$ip" "$p" && hits="$hits $p"; done
        printf '%-16s %-18s %-34s %s\n' "$ip" "$m" "${hits:- -}" "$(classify "$ip" "$hits")"
    done
    printf '%s\n' "----------------------------------------------------------------------------------------------------"
    echo "RTSP on 554 is the strongest camera signal. Try it:"
    echo "  ffprobe -rtsp_transport tcp rtsp://user:pass@<ip>:554/  (or /stream1, /h264, /onvif1)"
}

# --------------------------------------------------------------------------- #
snapshot() {   # one line per device, sorted -- for diffing
    collect
    local m
    for m in $(printf '%s\n' "${!SIG[@]}" "${!LIP[@]}" "${!AIP[@]}" | grep -v '^$' | sort -u); do
        printf '%s %s\n' "$m" "${AIP[$m]:-${LIP[$m]:-<no IP>}}"
    done
}

find_device() {
    echo "Baseline -- every device currently on $AP_IFACE:"
    warm_arp
    local before after
    before=$(snapshot); printf '%s\n' "$before" | sed 's/^/    /'
    cat <<'MSG'

Now power-cycle ONLY the device you are looking for -- unplug the camera, count
to five, plug it back in. Do not touch anything else on the network.

Watching for up to 180s. Ctrl-C to stop.
MSG
    local i
    for i in $(seq 1 60); do
        sleep 3
        after=$(snapshot)
        local gone new
        gone=$(comm -23 <(printf '%s\n' "$before") <(printf '%s\n' "$after"))
        new=$(comm -13 <(printf '%s\n' "$before") <(printf '%s\n' "$after"))
        if [ -n "$gone" ]; then
            echo; echo ">>> LEFT the network (this is your device powering down):"
            printf '%s\n' "$gone" | while read -r m ip; do
                printf '    MAC %s   IP %s   vendor %s\n' "$m" "$ip" "$(vendor_of "$m")"
            done
            before=$after
        fi
        if [ -n "$new" ]; then
            echo; echo ">>> JOINED the network (this is your device coming back):"
            printf '%s\n' "$new" | while read -r m ip; do
                printf '    MAC %s   \033[1mIP %s\033[0m   vendor %s   dhcp-name %s\n' \
                       "$m" "$ip" "$(vendor_of "$m")" "${LNAME[$m]:--}"
            done
            echo
            echo "That IP belongs to the device you just power-cycled. Certain, not inferred."
            echo "Pin it so it never changes:"
            printf '%s\n' "$new" | while read -r m ip; do
                echo "  # NetworkManager reads extra dnsmasq config for shared connections from here:"
                echo "  sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d"
                echo "  echo 'dhcp-host=$m,${PREFIX}.90,ipcam' \\"
                echo "    | sudo tee /etc/NetworkManager/dnsmasq-shared.d/ipcam.conf"
                echo "  sudo nmcli connection down $AP_CON && sudo nmcli connection up $AP_CON"
                echo "  # -> from then on this device is always ${PREFIX}.90 and answers to 'ipcam'"
            done
            return 0
        fi
        printf '.'
    done
    echo; echo "Nothing changed in 180s. Either the device never dropped its association,"
    echo "or it is not on this AP at all. Check: sudo iw dev $AP_IFACE station dump"
}

# --------------------------------------------------------------------------- #
load_alias
[ "$USE_ONVIF" -eq 1 ] && load_onvif

case $MODE in
    table) warm_arp; collect; print_table ;;
    ports) warm_arp; collect; print_table; probe_ports ;;
    find)  find_device ;;
    ip)    warm_arp; collect; print_ip ;;
    show)  warm_arp; collect; print_show ;;
    json)  warm_arp; collect; print_json ;;
    alias) warm_arp; collect; do_alias "$PATTERN" "$ALIAS_NAME" ;;
    watch) while true; do warm_arp; collect; clear; date '+%H:%M:%S'; print_table; sleep 3; done ;;
esac
