# phoxal-simulator-webots-controller

The Phoxal Webots controller: the process Webots runs for a simulated robot,
which puts that robot's components on the Phoxal bus.

It is a **plain bus client, not a Phoxal participant**.
There is no runner, no role attribute and no setup context.
It depends on `phoxal-bus`, `phoxal-protocol`, `phoxal-model` and
`phoxal-bundle`, and deliberately not on the `phoxal` facade, whose machinery
describes a supervised participant this process is not.

## What it does

1. Reads `manifest.json` from `--bundle-root` and takes the robot out of it -
   its components, their types, and how a world models each type.
2. Opens its Webots controller handle and resolves the world's `basicTimeStep`,
   which is what quantizes every capability's device refresh and therefore its
   effective publish cadence.
3. Learns its execution from the router at `--connect`.
   A router's session id **is** the execution, so the id is never an argument;
   an endpoint reporting zero or several executions is refused.
4. Opens one external bus session and binds every capability of every component
   instance to its Webots device: sensors publish samples, the battery publishes
   state, and motors subscribe setpoints under `phoxal-bus`'s fixed-source
   authority, which admits `drive` and nobody else.
5. Declares one liveliness Ready token **per component instance that declares a
   `driver` block**, and none for itself.
   That is the same set every launcher derives to decide which driver processes
   a real robot starts, and the set the supervisor expects: a driver's
   participant id is its component instance id, so this one process standing in
   for every driver is what makes a simulated robot read as complete.
   A component without a `driver` block runs no process on hardware either, so
   it is neither expected nor presented - it is still bound and simulated like
   any other.
6. Runs the Webots step loop: apply inputs, advance the world one step, publish
   everything that step produced, then publish the `Clock { step }` that closes
   it on `runtime/simulation/clock`.
   The order is the contract - a reader that has seen a step's clock has already
   seen that step's outputs.

Three of the capability kinds a component may declare are not simulated:

- `emergency_stop` is skipped and left unpublished. Webots has no button,
  switch or toggle node, so nothing in a simulated world could engage or release
  one, and asserting a state nobody can change would be worse than publishing
  nothing.
- `led` and `speaker` are refused at startup - the controller exits rather than
  running the world - because no participant owns those effects in the current
  graph, and applying them from any source would turn arrival order into
  authority.

Every other kind is simulated.

## Running it

```
phoxal-simulator-webots-controller --bundle-root <DIR> --connect <ENDPOINT>
```

That is the whole launch contract.
No execution id (learned from the router), no participant id (this process
stands for all of them), and no simulation flag (this binary **is** the
simulation).

You do not normally run it yourself.
`phoxal simulation webots run` stages this binary into
`webots/controllers/<name>/<name>` from the CLI's materialisation cache, and
Webots launches it with those arguments through the robot's `controllerArgs`.
Webots also stops it with `SIGTERM`; on that signal - or `SIGINT` - the
controller parks the world, drops its presence tokens, closes the bus session,
and exits 0.

Logging goes to stderr, which Webots collects into its console.
`RUST_LOG` sets the filter; the default is `info,phoxal.lease=warn`, because the
bus lease traces one line per motor command per motor and would otherwise bury
everything else. Raise them with `RUST_LOG=info,phoxal.lease=info`.

## Building it

Webots R2025a must be installed: `webots-rs` links Webots' `libController` at
build time, so `cargo check` and `cargo clippy` need it too - build scripts run
on check.
Use Webots' default install location or set `WEBOTS_HOME` to point at it.
At **run** time the dynamic loader has to find `libController` as well; Webots
sets that up for the controllers it launches, and a controller started by hand
needs `DYLD_LIBRARY_PATH` (macOS) or `LD_LIBRARY_PATH` (Linux) pointing at
`<Webots>/Contents/lib/controller` (macOS) or `<Webots>/lib/controller` (Linux).

musl and `aarch64-unknown-linux-gnu` are refused at compile time: Cyberbotics
ships no `libController` for them, and simulation runs on a desktop host rather
than on a robot image.

## Releases

The controller is published to the static `phoxal` registry, never to crates.io:
it is a binary, and robots compile it from source. It rides its own release
train, independent of the framework's.

## License

AGPL-3.0-only. See [LICENSE](LICENSE).
