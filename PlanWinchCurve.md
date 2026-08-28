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

## Plot current winch curve [DONE]
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

## Extend calc_vro_soft()

Give `calc_vro_soft` a second branch that commands reel-in below `f_low`, so that in
`force_limit = "soft"` the static law becomes the LOWER limiter as well as the upper
one and the `LowerForceController` is switched off — the same treatment the
`UpperForceController` already gets.

### Why the anchor has to move

The obvious spec ("zero at `f_low`, −0.3 m/s at `f_low - 75`") cannot be bolted onto the
current curve, because the current curve is not zero *at* `f_low` — it is zero over a
wide band above it. Measured with the defaults (`f_low=350`, `f_high=3800`, `kv=0.06`,
`softminus_beta=1e-3`):

```text
F [N]:   350   500   700   850   870   875   880   900  1000
v [m/s]: 0.0   0.0   0.0   0.0   0.0  0.080 0.226 0.477 1.024
```

The softminus corner spans `1/softminus_beta = 1000 N`, ~3x `f_low`, so the forward
tension curve has `T(0) = 874 N`: below that force no reel-out speed exists and the
inverse clamps to 0. Adding a reel-in ramp under `f_low` without touching this would
leave a 524 N dead band of exactly-zero speed between reel-in and reel-out — and with the
`LowerForceController` gone, nothing else would act inside it.

This is a `softminus_beta` artefact, not a defect of the construction, exactly as the
setting's own docstring warns (src/wc_settings.jl:177-183). Make the corner narrow enough
to fit inside the reel-in band and the EXISTING inversion already lands on zero at
`f_low`, with no formula change at all:

```text
beta-    1/beta     zero up to    dead band    v(1000)   v(3000)
1e-3     1000 N       874 N         524 N       1.024     3.533
1e-2      100 N       352 N           2 N       1.956     3.598
2e-2       50 N       350 N           0 N       1.956     3.598
5e-2       20 N       350 N           0 N       1.956     3.598
```

So `soft_lfc = true` requires a sharper lower corner rather than new maths. Validate
`softminus_beta * f_reel_in_band >= 2` (corner at most half the band, i.e. `>= 0.0267` at
75 N) and fail loudly — the default `1e-3` misses it by a factor of 27. Note the price:
the over-soft floor was dragging the whole lower half of the curve down, so the reel-out
branch changes a lot at low force (1.024 -> 1.956 m/s at 1000 N) and barely at all near
`f_high` (3.533 -> 3.598 at 3000 N). `softminus_beta` "must match the value the
trajectory optimizer applies to the same curve" (src/wc_settings.jl:169-175), so this is
a coordinated change with AWETrim's tension curve — but that curve has no reel-in branch
either, so the two modes need separate agreed values regardless.

Do NOT try to anchor the softminus analytically instead (subtracting `sp(-beta*f_low)`
from the corner so it hits `f_low` by construction): tried and rejected. The upper
softplus inversion carries its own offset (`sp_cap(0) = -22 N`), so the anchored curve
still leaves `v(f_low) = 0.285` m/s, and the shifted `t` can exceed `f_high`, which makes
the second `sp_inv` throw a `DomainError`.

### The law

A LINE through the two anchor points, cut off where the reel-out curve overtakes it. No
new setting: the slope is fixed by the anchors, and the meeting point falls out of `min`.

```text
soft_lfc = false:  unchanged, bit for bit.
soft_lfc = true:   m       = -v_reel_in / f_reel_in_band          [m/s per N]
                  line(F) = m * (F - f_low)
                  sqrt(F) = the CURRENT double inversion, unchanged

  F <= f_low : max(line(F), v_reel_in)
  F >  f_low : min(line(F), sqrt(F))
```

`min`/`max` is the whole implementation. Both arms are monotone and cross once, so the
line governs the low-force region and the tension curve takes over above it — and the
`min` states the safety property directly: never command more speed than the tension
curve allows.

The linear segment IS the `LowerForceController`'s replacement, and it is a proportional
force controller with gain `m` — 0.004 m/s per N at the default anchors.

Measured at `softminus_beta = 3e-2`, `f_reel_in_band = 75`, `v_reel_in = -0.3`:

```text
F [N]:   200   275   300   325   350   400   500   700   800   1000  1500  3000
v [m/s]: -0.3  -0.3  -0.2  -0.1   0.0   0.2   0.6   1.4  1.750 1.956 2.404 3.598
```

- both anchors exact: `v(275) = -0.3`, `v(350) = 0`;
- monotone over the whole range, no dead band;
- `max |dv/dF| = 0.00405` m/s per N, against ~1.85 (formally infinite) for the √ variant
  — a ~450x drop in force-noise sensitivity at the crossing;
- the line governs up to `F_x = 783 N` (v = 1.73 m/s), where it meets the tension curve.
  The linear regime is therefore ~508 N wide, much wider than `f_reel_in_band`; that is a
  consequence of the anchors and `kv`, not a free parameter.
- `C0` only: at `F_x` the slope steps 0.004 -> 0.00109, a factor 3.66. Mild and finite,
  unlike the vertical tangent it replaces.

The `softminus_beta` constraint above is still required, not made redundant by the line:
at the shipped `1e-3` the √ arm is 0 up to 874 N, so `min(line, 0) = 0` and the 524 N dead
band comes straight back.

### Settings, not literals

75 N and −0.3 m/s go into `WCSettings`/`data/wc_settings.yaml`, since `f_low` is a
per-call argument that varies at runtime:

- `soft_lfc::Bool = false` — selects the branch. Requires `force_limit == "soft"`.
- `f_reel_in_band = 75.0` — width of the reel-in ramp below `f_low` [N], absolute.
- `v_reel_in = -0.3` — reel-in speed at the bottom of the ramp [m/s]. Must satisfy
  `-v_ri_max <= v_reel_in < 0`; `v_ri_max = 8.0` already exists and keeps its meaning.

Signature: `calc_vro_soft(wcs, force, f_low=wcs.f_low; soft_lfc=wcs.soft_lfc)`. The
keyword is what lets the example draw both curves from one settings struct; the
production path through `set_vset_pc` (src/wc_components.jl:144) picks up the setting.
Validate in `WinchController(wcs)`, next to the existing `force_limit`/`mode` checks
(src/winchcontroller.jl:59-64): `force_limit == "soft"`, `-v_ri_max <= v_reel_in < 0`,
`f_reel_in_band > 0`, `f_low - f_reel_in_band >= 0`, and
`softminus_beta * f_reel_in_band >= 2` — the last one is what keeps the dead band closed,
and the shipped default fails it.

### Switching the LowerForceController off

Without this the change is cosmetic: below `f_low` the LFC activates
(src/wc_components.jl:751-753) and `Mixer_3CH` selects its channel
(src/winchcontroller.jl:129), so the new negative values would never reach the winch.

- Constructor: hold `wc.lfc` in permanent reset when `soft_lfc`, mirroring
  `wcs.force_limit == "soft" || set_reset(wc.ufc, false)` (src/winchcontroller.jl:72).
- `calc_v_set`: leave `set_reset(wc.lfc, reset)` (src/winchcontroller.jl:111) at `true`,
  and bypass `get_v_set_out(wc.lfc)` the way the UFC's nlsolve is bypassed at
  src/winchcontroller.jl:135 — otherwise it is pure waste.
- Consequence: with both force controllers dead the `SpeedController` is always active
  and `Mixer_3CH` always selects channel A, so `calc_v_set` degenerates to
  `calc_vro_soft -> SpeedController -> output`. Say so in the docs; the three-channel
  mixer is dead weight in this mode.
- `get_state` can then only ever return `wcsSpeedControl`, and `get_f_err`
  (src/winchcontroller.jl:251-259) only ever `NaN`.

**`f_err(logger)` crashes in this mode** (src/logging.jl:146-149): every logged `f_err`
is `NaN`, `filter(!isnan, ...)` is empty, and `maximum` over an empty collection throws.
`gamma` goes with it. Fix it to return `0.0` on an empty collection.

No change is needed in the `SpeedController`: it already saturates at
`[-v_ri_max, v_sat]` (src/wc_components.jl:466), so a negative setpoint passes through.

### Plot

`plot(X, Ys; labels)` (MakieControlPlots/src/plot.jl:84) takes ONE shared X and several
Ys. The two curves share the force grid and differ in speed, so they cannot share an
x-axis in the datasheet orientation via `plot`. Use `plotxy(Xs, Ys; legend)`
(MakieControlPlots/src/plotxy.jl:81) instead, which takes an independent X per series —
still the sanctioned plotting package, just a different entry point, so it does not
violate "never GLMakie/Plots directly". One figure, `winch_curve`: `Xs = [speed_false,
speed_true]`, `Ys = [force, force]` (the same grid twice), `legend=["soft_lfc = false",
"soft_lfc = true"]`, sweeping `f_low - f_reel_in_band - 50 … 1.1*f_high`.

### Tests

Extend `test/test_soft_limit.jl`:

- regression: `soft_lfc = false` reproduces the current values exactly;
- anchors: `v(f_low) == 0`, `v(f_low - f_reel_in_band) ≈ v_reel_in`, clamp below it;
- monotonicity of the whole curve and no dead band above `f_low`;
- the slope bound `max |dv/dF| <= 1.05 * m` below `F_x` — this is the point of the whole
  design and the test that catches a regression to the vertical tangent;
- `min` handover: the curve equals `line` below `F_x` and the tension curve above it,
  and never exceeds the tension curve anywhere;
- the `softminus_beta * f_reel_in_band >= 2` validation throws on the shipped default;
- `get_state(wc) == Int(wcsSpeedControl)` in a low-force run, and `gamma(logger)` /
  `f_err(logger)` do not throw.

The existing assertions at test/test_soft_limit.jl:174-181 ("The LowerForceController
stays") must be split by mode rather than deleted — they still hold for
`soft_lfc = false`.

### Docs to update

The "never negative, so reeling in stays the `LowerForceController`'s job" promise
(src/wc_components.jl:119-120), `calc_vro`'s claim that the soft law "returns exactly 0
at `f_low`" (src/wc_components.jl:66-69), the `force_limit` setting docstring's "the
`LowerForceController` stays" (src/wc_settings.jl:165-166), the `calc_vro_soft` bullet in
CLAUDE.md, and `docs/src/settings.md` for the three new settings.

### Unrelated, but found while reading

`force >= f_high && return v_sat` is a genuine jump — 5.096 m/s at 3799.9 N, 8.0 m/s at
3800 N — contradicting the "continuous everywhere" claim, independently of this change.

### Revision: floor moved to zero force, `f_reel_in_band` removed

Implemented as above with `v_reel_in = -0.3`, `f_reel_in_band = 75`, then asked to deepen
the floor to `-2 m/s` "without changing the slope". Since `m = -v_reel_in / f_reel_in_band`,
holding `m` fixed while `v_reel_in -> -2.0` forces `f_reel_in_band -> 500`, which exceeds
`f_low = 350` — the ramp would need negative force to reach the floor. Flagged this before
touching anything; the resolution, on request, was to anchor the SLOPE at zero force
instead: line through `(0, v_reel_in)` and `(f_low, 0)`, spanning the whole physically
valid range below `f_low` rather than a separately-configured sub-band.

This removes `f_reel_in_band` entirely — it was doing double duty as both "how far the
ramp is wide" and "where the clamp sits", and anchoring at `F=0` collapses both onto
`f_low`, which is already a setting. `m` is now `-v_reel_in / f_low` (per-call `f_low`,
consistent with the rest of the function). New default `v_reel_in = -2.0`.

The `softminus_beta` corner-sharpness requirement carries over with `f_low` as the
reference length instead of `f_reel_in_band`: analytically, the residual dead band above
`f_low` is `sp(-beta*f_low)/beta`, which decays like `exp(-beta*f_low)/beta` — negligible
(< 0.05 N) once `beta*f_low >= 7`. Validate `softminus_beta * f_low >= 8` (one round
number above that, matching the old check's actual strength: it worked out to
`beta*f_low ≈ 9.3` at the original `beta=3e-2`/`f_low=350`). The shipped `softminus_beta`
default (`1e-3`) now misses by roughly 8x instead of 27x — still fails, still needs
sharpening for `soft_lfc`.

Everything downstream (`min`/`max` handover, the `LowerForceController` switch-off, the
`f_err(logger)` fix, the crossing-point kink) is unchanged in kind, only in the specific
numbers: `F_x` (where the line meets the tension curve) moves to ~512 N at the test
settings (was ~600 N), since the slope itself changed.

The plot merge (single `winch_curve` figure via `plotxy`, described above) also changed
its sweep domain: since the line now spans exactly `[0, f_low]` with no separate band,
the sweep starts at `0` rather than `f_low - f_reel_in_band - 50`.

### Revision: soften the line/tension-curve crossing

The `min(line, sqrt)` handover above `f_low` is a genuine kink (`C0`, not `C1`) at `F_x`
where the two curves cross — asked to make it soft, matching the softplus/softminus
treatment already used at the `f_low`/`f_high` corners on the FORCE side.

Replace `min` with a smooth minimum, same log-sum-exp construction as `sp`/`sp_inv`
elsewhere in this file, numerically stable by subtracting out the hard min first:

```julia
soft_min(a, b, beta) = min(a, b) - log1p(exp(-beta * abs(a - b))) / beta
```

`beta` [s/m — the domain here is SPEED, not force] sets the sharpness; `soft_min <= min`
always, exact at `beta -> Inf`, and sits exactly `log(2)/beta` below the hard min right at
the crossing. New setting `reel_in_beta = 20.0`. Only the `F > f_low` branch changes,
`soft_min(line, sqrt, reel_in_beta)` in place of `min(line, sqrt)`; the `F <= f_low`
branch (`max(line, v_reel_in)`) is untouched, so the `(0, v_reel_in)`/`(f_low, 0)` anchors
stay exact.

Checked numerically before committing to it: near `f_low` the tension-curve inverse
already jumps to a sizeable value within a fraction of a newton (the same steep corner
behaviour documented under "the vertical tangent" earlier), while the line is still tiny
there — so `|line - sqrt|` is large near `f_low` regardless of `reel_in_beta`, and
`soft_min` reduces to the exact `min`. The softening is therefore invisible near the
`(f_low, 0)` anchor and only visibly rounds the corner near the actual crossing (~512 N at
the test settings, matched `hard_min` to float precision away from it, within `log(2)/beta`
of it at `F_x` itself — verified against an independent naive log-sum-exp reference).

No change to the `softminus_beta * f_low >= 8` requirement — that fixes a DEAD BAND
(a real defect), `reel_in_beta` only rounds a kink (cosmetic/dynamic-response, not a
correctness defect); `reel_in_beta > 0` is the only new invariant, validated alongside
the others in `WinchController`.
