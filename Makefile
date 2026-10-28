# Everyday commands. The Pi only runs binaries: everything for it is
# cross-compiled here, in Docker, by deploy/build-pi.sh.
#
#   make pi                build tstand_server and controller_v2 for the Pi
#   make deploy            build, upload, restart the Pi's services
#   make autostart-server  ...and launch tstand_server at boot (-controller, -both)
#   make no-autostart      stop launching both at boot
#   make status | logs     what the Pi's services are doing
#   make gui | test        the desktop GUI, the test suite
#
# HOST=rpi@other.local picks another rig; CONTROLLER_SRC=path builds a local
# controller_v2 checkout instead of GitHub master.

HOST ?= rpi@tonespi.local
export HOST

.PHONY: help pi pi-server pi-controller deploy deploy-only \
        autostart-server autostart-controller autostart-both no-autostart \
        status logs gui server test clean-pi

help:
	@sed -n '1,13p' Makefile | sed 's/^# \{0,1\}//'

pi:
	deploy/build-pi.sh both
pi-server:
	deploy/build-pi.sh server
pi-controller:
	deploy/build-pi.sh controller

deploy: pi
	deploy/deploy.sh
# Upload whatever is already in deploy/out without rebuilding.
deploy-only:
	deploy/deploy.sh

autostart-server: pi-server
	deploy/deploy.sh --autostart server
autostart-controller: pi-controller
	deploy/deploy.sh --autostart controller
# Read the warning in deploy/deploy.sh first: controller_v2 moves the rig
# as it starts, and contends with tstand_server for the bus.
autostart-both: pi
	deploy/deploy.sh --autostart both
no-autostart:
	deploy/deploy.sh --no-autostart

status:
	@ssh $(HOST) 'for u in tstand-server controller-v2; do \
	    printf "%-14s %-9s at boot: %s\n" $$u "$$(systemctl is-active $$u)" "$$(systemctl is-enabled $$u 2>/dev/null || echo not-installed)"; \
	done; ls -lh ~/tstand 2>/dev/null; df -h / | tail -1'
logs:
	ssh $(HOST) 'journalctl -f -u tstand-server -u controller-v2'

gui:
	cargo build --release
# The server for this PC (not the Pi).
server:
	cargo build --release --no-default-features --features can --bin tstand_server
test:
	cargo test
	cargo test --no-default-features --features can

# Remove this PC's cross-build output (not anything on the Pi).
clean-pi:
	rm -rf target/pi deploy/out
