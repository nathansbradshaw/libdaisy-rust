# [Documentation](https://docs.rs/libdaisy)

# libdaisy

Hardware Abstraction Layer implementation for Daisy boards.

## Requirements

- Hardware target

```bash
rustup target add thumbv7em-none-eabihf
```

- [cargo-binutils][cargo-binutils-url]

```bash
cargo install cargo-binutils

rustup component add llvm-tools-preview
```

# A Flashing Utility

One of:

- [electro-smith web programmer](https://electro-smith.github.io/Programmer/)
- [dfu-util](http://dfu-util.sourceforge.net/)
- [probe.rs](https://probe.rs/)

`probe.rs` requires a debug probe of some sort (e.g. ST link) and allows for fast debugging messages via RTT.

```bash
cargo embed --features log-rtt --example passthru
```
**note** You will need to specify the board IE `--chip stm32h750v` for the daisy seed, or create an `Embed.toml` that specifies the chip.

## Build Examples

```bash
cargo objcopy --example blinky --release -- -O binary blinky.bin
```

```bash
cargo objcopy --example passthru --release -- -O binary passthru.bin
```

# Minimum supported Rust version

The Minimum Supported Rust Version (MSRV) at the moment is 1.68.2.

# Demos

[Looper](https://github.com/mtthw-meyer/daisy-looper) - Basic one button looper.

[cargo-binutils-url]: https://github.com/rust-embedded/cargo-binutils
