#!/usr/bin/env bash
#
# Cross-compile tstand_server and controller_v2 for the rig's Raspberry Pi,
# on this PC, inside Docker (or Podman). The Pi only ever receives binaries.
#
#   deploy/build-pi.sh                    # both, into deploy/out/
#   deploy/build-pi.sh server             # only tstand_server
#   deploy/build-pi.sh controller         # only controller_v2
#
# controller_v2 comes from GitHub master (cached in deploy/.cache), or from
# CONTROLLER_SRC=/path/to/controller_v2 to build a local checkout as it is.
#
# Private git dependencies are downloaded here, on the PC, with your own git
# credentials (gh, a credential helper or an SSH key). The container then
# builds offline, so it never needs a key of its own.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/deploy/out"
CACHE="$REPO/deploy/.cache"
IMAGE="tstand-pi-cross"
TARGET="aarch64-unknown-linux-gnu"
CONTROLLER_REPO="${CONTROLLER_REPO:-https://github.com/TonesTeam/controller_v2.git}"
CONTROLLER_REF="${CONTROLLER_REF:-master}"
CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"

what="${1:-both}"
case "$what" in
    both|server|controller) ;;
    -h|--help) sed -n '3,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "usage: $0 [both|server|controller]" >&2; exit 2 ;;
esac

say() { printf '\n\033[1m==> %s\033[0m\n' "$1"; }

if command -v docker >/dev/null && docker info >/dev/null 2>&1; then
    engine=docker
elif command -v podman >/dev/null; then
    engine=podman
else
    echo "Docker or Podman is needed to cross-compile." >&2
    exit 1
fi

# Fetch with the git CLI, so whatever credentials git already has on this PC
# apply, and send ssh:// GitHub URLs over HTTPS for this one command only —
# controller_v2 names its sensor crate by ssh://, which needs an SSH key.
fetch() {
    say "Downloading dependencies for $(basename "$1")"
    (cd "$1" && \
        CARGO_NET_GIT_FETCH_WITH_CLI=true \
        GIT_CONFIG_COUNT=1 \
        GIT_CONFIG_KEY_0='url.https://github.com/.insteadOf' \
        GIT_CONFIG_VALUE_0='ssh://git@github.com/' \
        cargo fetch --locked)
}

build() {  # build <source dir> <cargo args...>
    local src="$1"; shift
    mkdir -p "$CARGO_HOME/registry" "$CARGO_HOME/git"
    # Run as you, so target/ is not left owned by root. CARGO_HOME is a fresh
    # directory holding only your downloaded registry and git checkouts.
    # Size: optimise for size, link-time optimise across crates, drop symbols.
    # Applied here so controller_v2 gets it without editing its Cargo.toml.
    # panic=abort would save more, but would let a panic in one thread (one
    # client, say) take the whole server down, so it stays at unwind.
    "$engine" run --rm \
        --user "$(id -u):$(id -g)" \
        -e CARGO_HOME=/cargo -e HOME=/tmp \
        -e CARGO_PROFILE_RELEASE_OPT_LEVEL=s \
        -e CARGO_PROFILE_RELEASE_LTO=fat \
        -e CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1 \
        -e CARGO_PROFILE_RELEASE_STRIP=symbols \
        -v "$CARGO_HOME/registry:/cargo/registry" \
        -v "$CARGO_HOME/git:/cargo/git" \
        -v "$src:/src" -w /src \
        "$IMAGE" \
        cargo build --release --offline --locked --target "$TARGET" "$@"
}

say "Preparing the cross-compile image ($engine)"
"$engine" build -q -t "$IMAGE" "$REPO/deploy/pi" >/dev/null

mkdir -p "$OUT"

if [[ "$what" != controller ]]; then
    fetch "$REPO"
    say "Building tstand_server for the Pi"
    # target/pi keeps these apart from the PC's own builds.
    build "$REPO" --no-default-features --features can --bin tstand_server --target-dir target/pi
    install -m 755 "$REPO/target/pi/$TARGET/release/tstand_server" "$OUT/tstand_server"
fi

if [[ "$what" != server ]]; then
    if [[ -n "${CONTROLLER_SRC:-}" ]]; then
        src="$(cd "$CONTROLLER_SRC" && pwd)"
        say "Using controller_v2 from $src ($(git -C "$src" log --oneline -1 2>/dev/null || echo 'not a git checkout'))"
    else
        src="$CACHE/controller_v2"
        if [[ -d "$src/.git" ]]; then
            git -C "$src" fetch -q origin "$CONTROLLER_REF"
        else
            mkdir -p "$CACHE"
            git clone -q "$CONTROLLER_REPO" "$src"
        fi
        git -C "$src" checkout -q --detach FETCH_HEAD 2>/dev/null \
            || git -C "$src" checkout -q --detach "origin/$CONTROLLER_REF"
        say "Using controller_v2 $CONTROLLER_REF from GitHub ($(git -C "$src" log --oneline -1))"
    fi
    fetch "$src"
    say "Building controller_v2 for the Pi"
    build "$src"
    install -m 755 "$src/target/$TARGET/release/controller_v2" "$OUT/controller_v2"
fi

say "Built for the Pi"
ls -lh "$OUT"
