# Contributing

Thanks for considering a contribution. This document covers the legal terms
under which contributions are accepted, and points at the conventions this
repository shares with the rest of Phoxal.

## License of contributions (inbound = outbound)

This project is licensed under AGPL-3.0-only. See [LICENSE](LICENSE) for the
full license text.

Contributions you submit are accepted under the same license that already
applies to the file(s) you change - "inbound = outbound". You retain
copyright on your contribution; you grant the project and its users a
license to use your contribution under the file's declared license.

## Developer Certificate of Origin (DCO)

This project uses the [Developer Certificate of Origin](https://developercertificate.org/)
(DCO) to confirm that you have the right to submit each contribution under
the terms above. Every commit must include a `Signed-off-by` trailer
matching the author of the commit:

```
Signed-off-by: Your Name <your.email@example.com>
```

Add it automatically with `git commit -s`.

## Commit messages

Commit messages must follow
[Conventional Commits](https://www.conventionalcommits.org/).
The pull request title follows them too: it is what the release automation reads
to size the next version.

Unlike the framework, a change here does not need the breaking marker for
touching a wire contract: this controller declares none. It speaks the
framework's contracts, and the framework's train is the compatibility identity
peers match on.

## Code conventions

The conventions are the framework's, and they are stated there rather than
restated here: <https://github.com/phoxal/framework/blob/main/CONTRIBUTING.md>.
The gates that enforce them are in this repository's own configuration - edition
2024, `-D warnings` in `.cargo/config.toml`, and `unwrap_used`/`expect_used`
denied outside tests in `Cargo.toml` and `clippy.toml`.

Before pushing:

```
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test
```

## Building

Webots R2025a must be installed. `webots-rs` links Webots' `libController` at
build time, so this applies to `cargo check` and `cargo clippy` too, not only
`cargo test` or a release build, since build scripts run on check. Install
Webots R2025a and either use its default install location or set `WEBOTS_HOME`
to point at it.

## Getting started

Open an issue or draft PR early for non-trivial changes - alignment before
code is cheaper than alignment after code.
