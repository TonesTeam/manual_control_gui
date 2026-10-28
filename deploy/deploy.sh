#!/usr/bin/env bash
#
# Copy this checkout (and optionally controller_v2) to the rig's computer,
# build there, and restart whatever is installed as a service. Run it from the
# PC, from anywhere in the repository.
#
#   deploy/deploy.sh                          # sync, build, restart services
#   deploy/deploy.sh --autostart              # launch both at boot
#   deploy/deploy.sh --autostart server       # only tstand_server at boot
#   deploy/deploy.sh --autostart controller   # only controller_v2 at boot
#   deploy/deploy.sh --no-autostart           # stop launching both at boot
#   deploy/deploy.sh --no-autostart server    # ...or just one of them
#   deploy/deploy.sh --controller             # also sync and build controller_v2
#   HOST=rpi@other.local deploy/deploy.sh     # a different rig
#
# --install-service is the old name for --autostart server.
#
# controller_v2 is taken from CONTROLLER_SRC (default: a controller_v2 checkout
# next to this one), or set CONTROLLER_BIN to a prebuilt aarch64 binary to
# upload that instead of building on the Pi. Building it there fetches the
# private optical-sensor-can crate over SSH, so this forwards your SSH agent
# for that one step — your GitHub key must be loaded (ssh-add -l).
#
# Rust must already be on the target (rustup.rs). The server is built with
# --no-default-features so it needs no display libraries, plus --features can
# for the Peltier temperature board, which needs libudev (libudev-dev on
# Debian; already present with systemd).

set -euo pipefail

HOST="${HOST:-rpi@tonespi.local}"
DIR="${DIR:-manual_control_gui}"
CONTROLLER_DIR="${CONTROLLER_DIR:-controller_v2}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTROLLER_SRC="${CONTROLLER_SRC:-$REPO/../controller_v2}"
CONTROLLER_BIN="${CONTROLLER_BIN:-}"

usage() { sed -n '3,27p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

enable_server=false
enable_controller=false
disable_server=false
disable_controller=false
with_controller=false

while (($#)); do
    case "$1" in
        --autostart)
            case "${2:-}" in
                server) enable_server=true; shift ;;
                controller) enable_controller=true; with_controller=true; shift ;;
                *) enable_server=true; enable_controller=true; with_controller=true ;;
            esac ;;
        --install-service) enable_server=true ;;
        --no-autostart)
            case "${2:-}" in
                server) disable_server=true; shift ;;
                controller) disable_controller=true; shift ;;
                *) disable_server=true; disable_controller=true ;;
            esac ;;
        --controller) with_controller=true ;;
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

# Sync and rebuild controller_v2 when asked to, or when it is already a
# service there — otherwise a restart would bring back a stale build.
if ! $with_controller && has_unit controller-v2.service; then
    with_controller=true
fi

say "Syncing $REPO to $HOST:~/$DIR"
# target/ stays on the far side so incremental builds survive; .git is not
# needed to build and is the bulk of the transfer.
rsync -az --delete \
    --exclude target \
    --exclude .git \
    --exclude 'docs/datasheets' \
    -e ssh "$REPO/" "$HOST:$DIR/"

say "Building tstand_server on $HOST"
# `cargo` is not on a non-interactive PATH by default after a rustup install.
ssh "$HOST" "cd '$DIR' && \
    export PATH=\"\$HOME/.cargo/bin:\$PATH\" && \
    cargo build --release --no-default-features --features can --bin tstand_server"

if $with_controller; then
    if [[ -n "$CONTROLLER_BIN" ]]; then
        say "Uploading prebuilt controller_v2 ($CONTROLLER_BIN)"
        ssh "$HOST" "mkdir -p '$CONTROLLER_DIR/target/release'"
        scp "$CONTROLLER_BIN" "$HOST:$CONTROLLER_DIR/target/release/controller_v2.new"
        ssh "$HOST" "cd '$CONTROLLER_DIR/target/release' && chmod 755 controller_v2.new && mv controller_v2.new controller_v2"
    else
        if [[ ! -f "$CONTROLLER_SRC/Cargo.toml" ]]; then
            echo "controller_v2 checkout not found at $CONTROLLER_SRC." >&2
            echo "Set CONTROLLER_SRC=/path/to/controller_v2, or CONTROLLER_BIN to a prebuilt binary." >&2
            exit 1
        fi
        CONTROLLER_SRC="$(cd "$CONTROLLER_SRC" && pwd)"
        say "Syncing $CONTROLLER_SRC to $HOST:~/$CONTROLLER_DIR"
        rsync -az --delete \
            --exclude target \
            --exclude .git \
            -e ssh "$CONTROLLER_SRC/" "$HOST:$CONTROLLER_DIR/"

        say "Building controller_v2 on $HOST"
        # -A lends the Pi your SSH agent so cargo can fetch the private
        # optical-sensor-can dependency from GitHub.
        ssh -A "$HOST" "cd '$CONTROLLER_DIR' && \
            export PATH=\"\$HOME/.cargo/bin:\$PATH\" && \
            export GIT_SSH_COMMAND='ssh -o StrictHostKeyChecking=accept-new' && \
            cargo build --release"
    fi
fi

install_unit() {
    local unit="$1"
    say "Installing $unit to launch at boot"
    scp "$REPO/deploy/$unit" "$HOST:/tmp/$unit"
    ssh "$HOST" "
        set -e
        sudo install -m 644 '/tmp/$unit' '/etc/systemd/system/$unit'
        rm -f '/tmp/$unit'
        sudo systemctl daemon-reload
        sudo systemctl enable '$unit'
    "
}

if $enable_server; then
    install_unit tstand-server.service
    ssh "$HOST" '
        if [ ! -f /etc/tstand-server.env ]; then
            echo "TSTAND_TOKEN=" | sudo tee /etc/tstand-server.env >/dev/null
            sudo chmod 600 /etc/tstand-server.env
            echo "Created an empty /etc/tstand-server.env — put a token in it."
        fi
    '
fi
$enable_controller && install_unit controller-v2.service

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

restarted=false
for unit in controller-v2.service tstand-server.service; do
    if has_unit "$unit"; then
        say "Restarting ${unit%.service}"
        ssh "$HOST" "sudo systemctl restart '$unit' && sleep 2 && \
            systemctl --no-pager --lines=10 status '$unit'; \
            echo; echo \"  at boot: \$(systemctl is-enabled '$unit' 2>/dev/null || true)\""
        restarted=true
    fi
done

if ! $restarted; then
    say "No services installed; start them by hand"
    echo "  ssh $HOST"
    echo "  cd $DIR && ./target/release/tstand_server --port /dev/ttyUSB0 --token <secret>"
    echo
    echo "Or launch both at boot: deploy/deploy.sh --autostart"
fi
