#!/usr/bin/env bash
#
# Copy this checkout to the rig's computer, build the control server there and
# restart it. Run it from the PC, from anywhere in the repository.
#
#   deploy/deploy.sh                      # sync, build, restart the service
#   deploy/deploy.sh --install-service    # also install the systemd unit
#   HOST=rpi@other.local deploy/deploy.sh # a different rig
#
# Rust must already be on the target (rustup.rs). The server is built with
# --no-default-features so it needs no display libraries, plus --features can
# for the Peltier temperature board, which needs libudev (libudev-dev on
# Debian; already present with systemd).

set -euo pipefail

HOST="${HOST:-rpi@tonespi.local}"
DIR="${DIR:-manual_control_gui}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

install_service=false
[[ "${1:-}" == "--install-service" ]] && install_service=true

say() { printf '\n\033[1m==> %s\033[0m\n' "$1"; }

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

if $install_service; then
    say "Installing the systemd unit"
    scp "$REPO/deploy/tstand-server.service" "$HOST:/tmp/tstand-server.service"
    ssh "$HOST" '
        set -e
        sudo install -m 644 /tmp/tstand-server.service /etc/systemd/system/tstand-server.service
        rm -f /tmp/tstand-server.service
        if [ ! -f /etc/tstand-server.env ]; then
            echo "TSTAND_TOKEN=" | sudo tee /etc/tstand-server.env >/dev/null
            sudo chmod 600 /etc/tstand-server.env
            echo "Created an empty /etc/tstand-server.env — put a token in it."
        fi
        sudo systemctl daemon-reload
        sudo systemctl enable tstand-server
    '
fi

if ssh "$HOST" 'systemctl list-unit-files tstand-server.service >/dev/null 2>&1'; then
    say "Restarting tstand-server"
    ssh "$HOST" 'sudo systemctl restart tstand-server && sleep 2 && systemctl --no-pager --lines=15 status tstand-server'
else
    say "No systemd unit installed; start it by hand"
    echo "  ssh $HOST"
    echo "  cd $DIR && ./target/release/tstand_server --port /dev/ttyUSB0 --token <secret>"
    echo
    echo "Or install the service: deploy/deploy.sh --install-service"
fi
