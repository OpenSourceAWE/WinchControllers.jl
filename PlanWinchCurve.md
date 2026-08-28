# Improve winch curve

Scope: Only investigate and improve the soft mode.

The winch set speed is following a function that depends on the current force.

Q1: Where is this function defined?

The force -> set-speed law lives in WinchControllers.jl:

- `calc_vro(wcs::WCSettings, force)` (WinchControllers/src/wc_components.jl:78) —
  the nominal law. In `"reelout"` mode it's the unsigned square-root law
  `v_set = kv * sqrt(force)`; in the default `"piecewise"` mode it's signed:
  `v_set = ±vf_max * sqrt(|force - f_low| / (f_high - f_low))`.
- `calc_vro_soft(wcs::WCSettings, force, f_low)` (WinchControllers/src/wc_components.jl:122) —
  a continuous, soft-saturating variant (softplus/softminus near `f_low`/`f_high`).
- `calc_v_set(wc::WinchController, v_act, force, f_low, ...)` (WinchControllers/src/winchcontroller.jl:95) —
  top-level entry point; dispatches between `LowerForceController`/`SpeedController`/
  `UpperForceController` states and calls into the law above.

The wind-speed-dependent gain feeding it, `kv`, comes from `winch_kv(v_wind; project)`
in this repo (src/winch_kv_table.jl:38).

Config: gains/thresholds (`kv`, `f_low`, `f_high`, `vf_max`, `v_sat`, `force_limit`) are
in `data/wc_settings.yaml`; wind-speed overrides are in `data/winch_kv_table.yaml`.

## Plot current winch curve
Add a script `plot_winch_curve.jl` to the `examples` folder that plots the winch curve
computed by `calc_vro_soft()`, in the style of a motor/winch datasheet torque-speed
curve: **speed on the x-axis, force on the y-axis**.

- Follow this repo's plotting conventions (see CLAUDE.md "Plotting"): only
  `MakieControlPlots.plot(x, y; xlabel, ylabel, labels, fig)`, never GLMakie/Plots
  directly, behind the standard activation guard, and `display(...)` the result.
- Settings: `wcs = WCSettings(dt=0.02); update(wcs)`, matching
  `examples/plot_power.jl`/`examples/test_winchcontroller.jl`.
- Sweep `force` from `0` to somewhat past `wcs.f_high` (e.g. `1.1 * wcs.f_high`) so the
  plot shows all three regions of the curve: flat zero below `f_low`, the rising
  soft-saturated section, and the `v_sat` plateau above `f_high`.
- `speed = calc_vro_soft.(Ref(wcs), force)`, then `plot(speed, force; xlabel="speed [m/s]", ylabel="force [N]", fig="winch_curve")`.
