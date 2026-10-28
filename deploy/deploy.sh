#!/usr/bin/env bash
#
# Upload the cross-compiled binaries in deploy/out/ to the rig's Raspberry Pi
# and restart whatever runs there as a service. Nothing is built on the Pi:
# run deploy/build-pi.sh (or `make pi`) first. Run it from the PC.
#
#   deploy/deploy.sh                          # upload, restart services
#   deploy/deploy.sh --autostart server       # tstand_server at boot
#   deploy/deploy.sh --autostart controller   # controller_v2 at boot
#   deploy/deploy.sh --autostart both         # both — see the warning below
#   deploy/deploy.sh --no-autostart           # stop launching both at boot
#   deploy/deploy.sh --no-autostart server    # ...or just one of them
#   HOST=rpi@other.local deploy/deploy.sh     # a different rig
#
# --install-service is the old name for --autostart server.
#
# Both at boot is a deliberate choice, not the default: controller_v2 opens the
# RS485 and CAN adapters as it starts and queues its initialisation sequence,
# which moves the pumps and valves, whenever it finds both adapters. Started
# alongside tstand_server, the two contend for the bus.
#
# On the Pi everything lives in ~/tstand: the two binaries, and the
# tstand_server.json / tstand_*.json state the server keeps next to itself.

set -euo pipefail

HOST="${HOST:-rpi@tonespi.local}"
DEST="${DEST:-tstand}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/deploy/out"

usage() { sed -n '3,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

enable_server=false
enable_controller=false
disable_server=false
disable_controller=false

while (($#)); do
    case "$1" in
        --autostart|--no-autostart)
            flag="$1"
            case "${2:-}" in
                server|controller|both) w="$2"; shift ;;
                *) if [[ "$flag" == --autostart ]]; then
                       echo "--autostart needs a choice: server, controller or both" >&2
                       exit 2
                   fi
                   w=both ;;
            esac
            if [[ "$flag" == --autostart ]]; then
                [[ "$w" != controller ]] && enable_server=true
                [[ "$w" != server ]] && enable_controller=true
            else
                [[ "$w" != controller ]] && disable_server=true
                [[ "$w" != server ]] && disable_controller=true
            fi ;;
        --install-service) enable_server=true ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if { $disable_server && $enable_server; } || { $disable_controller && $enable_controller; }; then
    echo "--autostart and --no-autostart contradict each other" >&2
    exit 2
fi

say() { printf '\n\033[1m==> %s\033[0m\n' "$1"; }
has_unit() { ssh "$HOST" "systemctl cat '$1' >/dev/null 2>&1"; }

# Upload what was built. A binary is replaced by rename, so a running service
# keeps its old file until it restarts rather than crashing mid-copy.
uploaded=0
for bin in tstand_server controller_v2; do
    [[ -f "$OUT/$bin" ]] || continue
    say "Uploading $bin ($(du -h "$OUT/$bin" | cut -f1))"
    ssh "$HOST" "mkdir -p '$DEST'"
    scp -q "$OUT/$bin" "$HOST:$DEST/$bin.new"
    ssh "$HOST" "chmod 755 '$DEST/$bin.new' && mv '$DEST/$bin.new' '$DEST/$bin'"
    uploaded=$((uploaded + 1))
done
if ((uploaded == 0)) && ! $disable_server && ! $disable_controller; then
    echo "Nothing in $OUT to upload. Build first: make pi" >&2
    exit 1
fi

need() {  # need <binary> — refuse to enable a service with nothing to run
    ssh "$HOST" "test -x '$DEST/$1'" || {
        echo "$1 is not on the Pi yet. Build it first: make pi" >&2
        exit 1
    }
}

install_unit() {
    local unit="$1"
    say "Installing $unit to launch at boot"
    scp -q "$REPO/deploy/$unit" "$HOST:/tmp/$unit"
    ssh "$HOST" "
        set -e
        sudo install -m 644 '/tmp/$unit' '/etc/systemd/system/$unit'
        rm -f '/tmp/$unit'
        sudo systemctl daemon-reload
        sudo systemctl enable '$unit'
    "
}

if $enable_server; then
    need tstand_server
    install_unit tstand-server.service
    ssh "$HOST" '
        if [ ! -f /etc/tstand-server.env ]; then
            echo "TSTAND_TOKEN=" | sudo tee /etc/tstand-server.env >/dev/null
            sudo chmod 600 /etc/tstand-server.env
            echo "Created an empty /etc/tstand-server.env — put a token in it."
        fi
    '
fi
if $enable_server && $enable_controller; then
    printf '\n\033[1;33mwarning:\033[0m controller_v2 initialises the rig as it starts (pumps and\n'
    printf '  valves move) whenever it finds the RS485 and CAN adapters, and then\n'
    printf '  contends with tstand_server for them. Both will now start at boot.\n'
fi
if $enable_controller; then
    need controller_v2
    install_unit controller-v2.service
fi

if $disable_server || $disable_controller; then
    say "Turning off launch at boot"
    units=()
    $disable_server && units+=(tstand-server.service)
    $disable_controller && units+=(controller-v2.service)
    for unit in "${units[@]}"; do
        if has_unit "$unit"; then
            ssh "$HOST" "sudo systemctl disable '$unit'"
            echo "  $unit: no longer starts at boot (still running until stopped or rebooted)"
        fi
    done
fi

enabled_now() {  # was this unit just switched on with --autostart?
    case "$1" in
        tstand-server.service) $enable_server ;;
        controller-v2.service) $enable_controller ;;
    esac
}

# Restart only what was already running, plus what was just enabled: a
# service someone stopped on purpose stays stopped. controller_v2 in
# particular initialises the rig as it starts.
restarted=false
for unit in controller-v2.service tstand-server.service; do
    if has_unit "$unit" && { enabled_now "$unit" || ssh "$HOST" "systemctl is-active --quiet '$unit'"; }; then
        say "Restarting ${unit%.service}"
        ssh "$HOST" "sudo systemctl restart '$unit' && sleep 2 && \
            systemctl --no-pager --lines=10 status '$unit'; \
            echo; echo \"  at boot: \$(systemctl is-enabled '$unit' 2>/dev/null || true)\""
        restarted=true
    fi
done

if ! $restarted && ! has_unit tstand-server.service && ! has_unit controller-v2.service; then
    say "No services installed; start them by hand"
    echo "  ssh $HOST"
    echo "  ~/$DEST/tstand_server --port /dev/ttyUSB0 --token <secret>"
    echo
    echo "Or launch at boot: make autostart-server (or -controller, -both)"
fi
