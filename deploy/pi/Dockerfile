# Cross-compiles for the rig's Raspberry Pi (64-bit Raspberry Pi OS, Debian 12
# "bookworm"). Building on bookworm keeps glibc at the Pi's 2.36, so the
# binaries run there without anything else installed.
FROM rust:1-bookworm

RUN dpkg --add-architecture arm64 \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        gcc-aarch64-linux-gnu libc6-dev-arm64-cross \
        pkg-config libudev-dev:arm64 \
 && rm -rf /var/lib/apt/lists/* \
 && rustup target add aarch64-unknown-linux-gnu

ENV CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
    PKG_CONFIG_ALLOW_CROSS=1 \
    PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig
