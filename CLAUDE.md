# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Julia package providing discrete controllers for winches (motor/generator + drum, with or
without gearbox) reeling a tether connected to a load or kite. Part of the "Julia Kite
Power Tools" family. Two independent controller families live here, sharing only the drum
and a single settings struct/YAML file (`WCSettings` / `data/wc_settings.yaml`):

- **Speed-output family** (the original code): [`WinchController`](src/winchcontroller.jl)
  combines a [`SpeedController`](src/wc_components.jl), a
  [`LowerForceController`](src/wc_components.jl) and an
  [`UpperForceController`](src/wc_components.jl) through a `Mixer_3CH`, producing a
  reel-out speed setpoint for the motor/generator.
- **Torque-output family** ([`torque_controllers.jl`](src/torque_controllers.jl), moved
  here from V3Kite.jl): `WinchPosController` (cascaded length/speed loop) and
  `WinchForceController` (compliant force mode), deliberately plant-agnostic — they take
  plain scalars (force, speed, length, drum radius/gear ratio/friction), not a kite-model
  object, so this package needs no model dependency.

A run picks one family; nothing here needs a kite model or a simulator.

## Commands

```bash
julia --project -e 'using Pkg; Pkg.test()'   # full suite (careful: docs warn it can take up to 60 min)
```

In a live REPL, `include("test/runtests.jl")` for everything, or `include("test/test_winchcontroller.jl")`
(etc.) for one file's testset alone. `bin/run_julia` starts a REPL on the project and
defines `menu()`/`menu2()`, which `include` `examples/menu.jl`/`examples/menu2.jl` — an
interactive terminal menu (cursor up/down, Enter) to run the example scripts. `bin/install`
bootstraps Julia via juliaup, resolves/instantiates the root, `examples/`, `test/` and
`docs/` projects, and precompiles everything; run it once per machine, not per session.

Docs build/doctest (mirrors the CI `docs` job):

```bash
julia --project=docs -e 'using Pkg; Pkg.instantiate()'
julia --project=docs docs/make.jl
```

## Environments

Root `Project.toml` declares `[workspace] projects = ["examples", "docs", "test"]`, so
each subproject keeps its own `Project.toml` but resolves against the single
`Manifest-v1.11.toml`/`Manifest-v1.12.toml` at the root (Julia-version-specific; Pkg 1.11+
only — older Pkg resolves subprojects standalone). `examples/Project.toml` sources
`WinchControllers` itself via `[sources] path = ".."`.

## Architecture

`src/WinchControllers.jl` sets the include order — `wc_settings.jl`, `utils.jl`,
`components.jl`, `wc_components.jl`, `winchcontroller.jl`, `torque_controllers.jl`,
`logging.jl` — later files depend on structs/functions from earlier ones.

- **`wc_settings.jl`** — `WCSettings`, the single tuning struct for BOTH controller
  families (loaded from `data/wc_settings.yaml`). The speed-controller fields are the
  historical content; the `winch_pos_*`/`winch_speed_*`/`winch_ff_scale`/`winch_force_*`/
  `winch_len_kp`/`winch_damp` fields at the end tune the torque controllers. `dt` defaults
  to `NaN` — every caller must set it explicitly, so a missing timestep fails loudly
  instead of silently running at the wrong rate.
- **`components.jl`** — generic, reusable discrete control blocks used to assemble the
  speed-output family: `Integrator`, `UnitDelay`, `RateLimiter`, `LowPass`, `Mixer_2CH`,
  `Mixer_3CH`. Not winch-specific.
- **`wc_components.jl`** — the winch-specific pieces built from those blocks:
  `CalcVSetIn` (wraps `calc_vro`/`calc_vro_soft`, the force -> set-speed law), `Winch` (a
  simple plant model used by examples/tests), `SpeedController`,
  `LowerForceController`/`UpperForceController` (both `<: AbstractForceController`,
  sharing `set_v_act`/`set_force`/`set_reset`/`set_f_set`/`set_v_sw`/`set_tracking`).
  - `calc_vro(wcs, force)`: `WCSettings.mode` selects the law's shape — `"piecewise"`
    (default) uses `vf_max`/`f_low`/`f_high`, signed (reel-in below `f_low`); `"reelout"`
    is the unsigned `kv * sqrt(force)` law. `test = true` also forces `"reelout"`.
  - `calc_vro_soft(wcs, force, f_low; soft_lfc, use_awe_trim)`: continuous alternative,
    orthogonal to `mode`, selected by `WCSettings.force_limit = "soft"` (default
    `"hard"`). Requires `mode == "reelout"`; when active it replaces the
    `UpperForceController` as the upper limiter (which is then held in permanent reset)
    via softplus/softminus corners (`softplus_beta`/`softminus_beta`) and an optional
    asymmetric low-pass (`force_limit_tau`/`force_limit_tau_rise`). `soft_lfc` (defaults
    to `wcs.soft_lfc`) additionally replaces the `LowerForceController`: a straight
    line, SHIFTED DOWN by `log(2) / reel_in_beta`, through `(0, v_reel_in)` and
    `(f_low, 0)` — the whole physically valid range below `f_low`, since force is never
    negative, so no separate ramp-width setting exists, and the shift keeps its slope
    exact instead of eroding it — smoothly capped (`soft_min`, sharpness `reel_in_beta`)
    where the tension curve, HARD-clamped at `f_low` here (`softminus_beta` plays no
    part in this branch — only the line-less, `soft_lfc = false` branch above needs it,
    to avoid a dead band with no line to fall back on), overtakes it above `f_low`,
    rather than a hard `min` (kink-free handover; `soft_min` is the same log-sum-exp
    construction as `sp`/`sp_inv` above, just applied on the SPEED axis instead of
    force). `reel_in_beta` is the SOLE tunable sharpness for this whole handover.
    Requires `reel_in_beta * kv * sqrt(f_low) >= 8`, checked in `WinchController` — the
    tension-curve inverse jumps from `0` to `kv * sqrt(f_low)` almost immediately above
    `f_low` rather than ramping up gently, so a `reel_in_beta` that is too soft relative
    to that jump leaves a visible hump instead of a smooth corner. `use_awe_trim`
    (`[0, 1]`, default `0`) blends this curve towards AWETrim's own tension-curve
    constants — see its own docstring, particularly for why the blend is done on the
    two FULL forward (force-at-a-given-speed) curves rather than on the returned speed.
- **`winchcontroller.jl`** — `WinchController`, the top-level speed-output controller.
  `calc_v_set` drives one timestep: sets force/speed on the three sub-controllers, mixes
  their outputs through `Mixer_3CH` (channel selection = which sub-controller is active),
  and returns the winch set speed. `WinchControllerState` (`wcsLowerForceLimit` /
  `wcsSpeedControl` / `wcsUpperForceLimit`) reports which one is currently in control.
- **`torque_controllers.jl`** — the plant-agnostic torque-output family, see "What this
  is" above. `force_to_torque` converts tether force to holding torque; a model wraps
  these in a one-line method taking its own state (e.g. V3Kite.jl's
  `winch_position_torque!(s::V3KITE, ...)`). Carries its own `SPDX-License-Identifier:
  MPL-2.0` header (different from this repo's overall MIT license) because it was moved
  here verbatim from V3Kite.jl — don't overwrite it with MIT.
- **`utils.jl`** — `saturate`, the `@limit` macro, `merge_angles`, `moving_average`, and
  synthetic test-signal generators (`get_startup`, `get_triangle_wind`) used by examples
  and tests.
- **`logging.jl`** — `WCLogger` and the performance indicators computed from a run's log:
  `f_err`, `v_err`, `damage`, `gamma`.

Block-diagram docs for the speed-output family live in `docs/src/` (`winchcontroller.md`,
`components.md`, `settings.md`, `performance_indicators.md`, `autotuning.md`,
`tests.md`, `functions.md`); `WinchController`'s own docstring links to
`docs/src/assets/winch_controller.png`.

## Plotting

Use the functions of the package MakieControlPlots.jl for plotting — never GLMakie/Plots
directly. It is a dependency of `examples/`, `test/` and `mwes/`, not of the library
(`src/` never plots). Every script that plots opens with this guard, since `test/` and
`mwes/` scripts are sometimes run with the root project active instead of their own:

```julia
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using WinchControllers, MakieControlPlots
```

Four plotting entry points cover everything in this repo:

- `plot(x, y; xlabel, ylabel, labels, fig)` — a single-panel line plot; `y` can be a vector
  of series for one panel with a `labels` vector, `fig` names the window. Always
  `display(...)` it — scripts run via `include`, so the return value is otherwise dropped.
  All series share one `x`; when they don't (e.g. two curves that diverge along the axis
  being compared, not the shared one) use `plotxy` instead.
- `plotxy(xs, ys; xlabel, ylabel, legend, fig)` — like `plot`, but each series gets its OWN
  x AND y vector (`xs`/`ys` are vectors of vectors); `legend` labels them. Used by
  `examples/plot_winch_curve.jl` to compare `calc_vro_soft`'s `soft_lfc` branches on one
  speed/force datasheet axis, since the two curves differ in speed at a given force.
- `plotx(x, sig1, sig2, ...; title, ylabels, ysize, labels, fig)` — the stacked-panel form
  used for full controller runs (one panel per signal, shared x-axis); see
  `examples/plot_power.jl`'s `plot(lg::WCLogger)` for the canonical multi-panel layout
  logging a `WCLogger` run (wind, speed, acceleration, jerk, force, power, controller
  state).
- `bode_plot(sys; from, to, title)` — frequency-response plots for the linearized-system
  stability examples (`stability_lfc.jl`, `stability_ufc.jl`, `mwes/mwe_02..06.jl`), used
  together with `ControlSystemsBase`/`RobustAndOptimalControl`.

`examples/plot_power.jl` and `examples/autotune.jl` both `import MakieControlPlots: plot`
and add a `plot(lg::WCLogger)` method — follow that pattern (extend `plot`, don't invent a
new function name) when a script needs a repeatable, run-specific plot.

## Use Kaimon

Kaimon exposes the live Julia REPL and code-intelligence tools; prefer it over a shell
subprocess or ad-hoc `grep`/`find`.

- **Executing code** — `ex` evaluates in the REPL the user is watching; `src/` edits are
  picked up automatically via Revise, so never call `Revise.revise()` yourself. Run tests
  by `include`-ing the test file into that REPL, not `Pkg.test()` in a subprocess — a fresh
  subprocess can't reuse the compiled V3 model or `examples/cache/settled_*.bin`, so a full
  run there pays the multi-minute first-run cost every time.
- **Finding code** — semantic search when exploring or when you only know what the code
  does, not its name; grep when you already have an exact symbol, call site, or string to
  match.

## Conventions

- Tunables belong in `data/wc_settings.yaml` through `WCSettings`, never hardcoded in a
  script or source file. Unknown YAML keys error; missing ones fall back to the struct
  default (`StructTypes.constructfrom!`). New fields are added at the *end* of the struct
  so the positional `WCSettings(update; dt)` constructor other packages use keeps working.
- **Terminology: "reel-out"/"reel-in", never "pay-out"/"pay-in"** — mirrors the sibling
  packages in Julia Kite Power Tools.
- Every exported type/function needs a docstring (`"""`, not `#`); CI runs `Documenter`
  doctests, so a docstring's example code must actually work.
- Inline comments are ONLY allowed when stating a very non-obvious fact, and
  then keep them to 1 line at most. Give every type/function a docstring ("""
  not #) instead, but not too verbose, people won't reed it if your docstring is
  too long, and explain how the code works in the docstring, but not the whole
  story behind it.
- Remove or make inline comments 1 line where you see them.
- In YAML files, consider the comments at the top of the file as docstring where you can add multiline comments.
