//! `phoxal-simulator-webots-controller` - the Webots-owned controller process
//! that simulates one Phoxal robot's components onto the bus.

// The package that cannot support a target owns that truth. This controller
// links Webots' `libController` at build time, and Cyberbotics ships it only
// for the desktop platforms Webots itself runs on - so the unsupported targets
// fail here, legibly, instead of as a linker error or as release metadata the
// build never consults.
#[cfg(target_env = "musl")]
compile_error!(
    "phoxal-simulator-webots-controller does not support musl targets: it links Webots' \
     libController, which Cyberbotics ships only against glibc. Simulation runs on a \
     desktop host, not on a musl robot image."
);

#[cfg(all(target_arch = "aarch64", target_os = "linux", target_env = "gnu"))]
compile_error!(
    "phoxal-simulator-webots-controller does not support aarch64-unknown-linux-gnu: \
     Webots ships no aarch64 Linux distribution, so there is no libController to link \
     against. Run simulation on an x86_64 Linux or an Apple Silicon macOS host."
);

mod backend;
mod capabilities;
mod catalog;
mod channel;
mod controller;
mod runtime;

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use tracing_subscriber::EnvFilter;

/// The one tracing target this binary emits under. Everything it says about
/// itself is filterable as one name.
pub(crate) const LOG_TARGET: &str = "simulator_webots_controller";

const ABOUT: &str = "phoxal-simulator-webots-controller - the Phoxal Webots controller";

const LONG_ABOUT: &str = "\
phoxal-simulator-webots-controller - the Phoxal Webots controller

Webots launches this process for the robot it is the controller of, passing
these arguments through the robot's `controllerArgs`. It is an external
simulator host, not a Phoxal participant: it reads the robot out of the bundle's
manifest.json, attaches to the supervisor's execution, and simulates every
component the robot mounts - publishing their samples, subscribing their
setpoints, and publishing the authoritative world clock.

The execution id is not an argument. A router's session id IS the execution, so
this process asks the router at --connect rather than being told, and refuses to
start when that endpoint reports anything other than exactly one execution.

--version reports this package's own version. Compatibility with a bundle is the
framework contract train, never this diagnostic product version.";

/// The controller's complete launch contract. Everything else it needs is in
/// the bundle or on the bus.
#[derive(Debug, Parser)]
#[command(name = "phoxal-simulator-webots-controller")]
#[command(about = ABOUT, long_about = LONG_ABOUT)]
#[command(version)]
struct Cli {
    /// The installed bundle directory to read manifest.json from.
    #[arg(long, value_name = "DIR")]
    bundle_root: PathBuf,

    /// The supervisor router endpoint to connect to.
    #[arg(long, value_name = "ENDPOINT")]
    connect: String,
}

/// Multi-thread: Zenoh refuses to run on Tokio's current-thread scheduler, and
/// the Webots step loop runs on its own dedicated OS thread outside the
/// runtime, so the transport has to keep draining while that thread is blocked
/// inside Webots.
#[tokio::main]
async fn main() -> ExitCode {
    // Parse first: help, version, and misuse all end the process without ever
    // needing a subscriber installed.
    let cli = Cli::parse();
    init_tracing();
    match controller::run(&cli.bundle_root, &cli.connect).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            // Webots collects the controller's stderr into its console, so one
            // rendered error chain is what an operator actually reads. Not a
            // panic: the world is already parked by the time we get here.
            eprintln!("phoxal-simulator-webots-controller: {error:#}");
            ExitCode::from(1)
        }
    }
}

fn init_tracing() {
    // The same defaults as the supervisor: `RUST_LOG` decides, otherwise this
    // process at `info` and the transport at `warn`. `zenoh_link_unixsock_stream`
    // warns on every successful client connect over a Unix socket (the client
    // side has no named local address), so it is held to errors.
    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new("info,zenoh=warn,zenoh_link_unixsock_stream=error"));
    // Webots pipes a controller's stderr into its console when it launches one,
    // and that pipe renders no escape codes; a controller started by hand at a
    // terminal gets colour.
    let ansi = std::io::IsTerminal::is_terminal(&std::io::stderr());
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .with_ansi(ansi)
        .init();
}

/// Resolve when the host asks this controller to stop.
///
/// Webots stops a controller with SIGTERM, and an operator running one by hand
/// interrupts it; both mean the same thing here, so both resolve this.
pub(crate) async fn shutdown_signal() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{SignalKind, signal};

        // A controller that cannot install its signal handlers would otherwise
        // spin on an immediately-ready branch of the caller's `select!` and
        // shut down at once. Reporting the failure and waiting forever leaves
        // the world running; Webots can still stop the process outright.
        let mut terminate = match signal(SignalKind::terminate()) {
            Ok(terminate) => terminate,
            Err(error) => {
                tracing::error!(
                    target: LOG_TARGET,
                    error = %error,
                    "failed to install the SIGTERM handler; \
                     stop this controller from Webots instead"
                );
                return std::future::pending().await;
            }
        };
        let mut interrupt = match signal(SignalKind::interrupt()) {
            Ok(interrupt) => interrupt,
            Err(error) => {
                tracing::error!(
                    target: LOG_TARGET,
                    error = %error,
                    "failed to install the SIGINT handler; \
                     stop this controller from Webots instead"
                );
                return std::future::pending().await;
            }
        };
        tokio::select! {
            _ = terminate.recv() => {}
            _ = interrupt.recv() => {}
        }
    }
    #[cfg(not(unix))]
    {
        if let Err(error) = tokio::signal::ctrl_c().await {
            tracing::error!(
                target: LOG_TARGET,
                error = %error,
                "failed to listen for an interrupt; stop this controller from Webots instead"
            );
            return std::future::pending().await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Cli;
    use clap::{CommandFactory, Parser};

    #[test]
    fn the_launch_contract_is_exactly_bundle_root_and_connect() {
        let cli = Cli::try_parse_from([
            "phoxal-simulator-webots-controller",
            "--bundle-root",
            "/srv/robot/bundle",
            "--connect",
            "tcp/127.0.0.1:7447",
        ])
        .expect("the documented argv must parse");
        assert_eq!(cli.bundle_root.as_os_str(), "/srv/robot/bundle");
        assert_eq!(cli.connect, "tcp/127.0.0.1:7447");
    }

    #[test]
    fn both_arguments_are_required() {
        assert!(
            Cli::try_parse_from([
                "phoxal-simulator-webots-controller",
                "--bundle-root",
                "/srv/robot/bundle",
            ])
            .is_err()
        );
        assert!(
            Cli::try_parse_from([
                "phoxal-simulator-webots-controller",
                "--connect",
                "tcp/127.0.0.1:7447",
            ])
            .is_err()
        );
    }

    /// The launch contract this binary answers to is closed: no execution id,
    /// no participant id, no simulation flag. This process IS the simulation,
    /// and it learns its execution from the router, so accepting any of them
    /// would invite a launcher to state a fact that could disagree with the run
    /// actually in progress.
    #[test]
    fn the_participant_launch_arguments_are_refused() {
        for argument in [
            "--participant-id=front_left_drive",
            "--execution-id=1",
            "--execution-origin=0",
            "--simulation",
        ] {
            assert!(
                Cli::try_parse_from([
                    "phoxal-simulator-webots-controller",
                    "--bundle-root",
                    "/srv/robot/bundle",
                    "--connect",
                    "tcp/127.0.0.1:7447",
                    argument,
                ])
                .is_err(),
                "{argument} must be refused"
            );
        }
    }

    #[test]
    fn the_command_definition_is_internally_consistent() {
        Cli::command().debug_assert();
    }
}
