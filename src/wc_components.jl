"""
A collection of control functions and components for winch control

Components:

- CalcVSetIn   calculate the set speed of the speed controller, using soft switching
- Winch        model of 20 kW winch, 4000 N max force, 8 m/s max speed
- SpeedController
- LowerForceController
- UpperForceController

Implemented as described in the PhD thesis of Uwe Fechner.
"""

"""
    mutable struct CalcVSetIn

Component for calculation `v_set_in`, using soft switching.

## Fields
- wcs::[WCSettings](@ref)
- mixer2::[Mixer_2CH](@ref): mixer component. Default: `Mixer_2CH(wcs.dt, wcs.t_blend)`
- filter::[LowPass](@ref): low-pass on the force, `"soft"` force limiting only
- `f_low`: lower force limit of the soft law; tracks `calc_v_set`'s argument
- input_a: Default: 0
- input_b: Default: 0
"""
@with_kw mutable struct CalcVSetIn @deftype Float64
    wcs::WCSettings
    mixer2::Mixer_2CH = Mixer_2CH(wcs.dt, wcs.t_blend)
    filter::LowPass   = LowPass(wcs.dt, wcs.force_limit_tau,
                                isnan(wcs.force_limit_tau_rise) ? wcs.force_limit_tau :
                                                                  wcs.force_limit_tau_rise)
    f_low       = wcs.f_low
    input_a     = 0
    input_b     = 0
end

"""
    function CalcVSetIn(wcs::WCSettings)

Constructor for component for calculation `v_set_in`, using soft switching.

## Parameters
- wcs:: [WCSettings](@ref): settings struct with the winch controller settings

## Returns
- a new struct of type [CalcVSetIn](@ref)
"""
function CalcVSetIn(wcs::WCSettings)
    CalcVSetIn(wcs=wcs)
end

"""
    calc_vro(wcs::WCSettings, force)

Calculate the optimal reel-out speed for a given force.

In `wcs.mode == "reelout"` (or `wcs.test == true`), the unsigned square-root law
`kv * sqrt(force)` is used; it never commands reel-in. Below `f_low` it is the
`LowerForceController` that takes over and pulls in at `v_sw` — that hand-over
is REEL_OUT mode's reel-in behaviour, not a gap in the law. Otherwise (the
default `"piecewise"` mode) the signed law below applies, which reels in
whenever `force < f_low`.

This is the NOMINAL law, unaffected by `wcs.force_limit`: the force controllers'
hand-over speeds `v_sw` are read off it under both settings, and the soft law of
[`calc_vro_soft`](@ref) returns exactly 0 at `f_low` (negative below it under
`wcs.soft_lfc`) and `v_sat` at `f_high` — the two worst possible values for
that purpose.

## Parameters
- wcs::[WCSettings](@ref): the settings struct
- force: the tether force at the winch

## Returns
- the optimal reel-out speed
"""
function calc_vro(wcs::WCSettings, force)
    if wcs.test || wcs.mode == "reelout"
        return sqrt(force) * wcs.kv
    else
        if force >= wcs.f_low
            return wcs.vf_max * sqrt((force - wcs.f_low) / (wcs.f_high - wcs.f_low))
        else
            return -wcs.vf_max * sqrt((wcs.f_low - force) / (wcs.f_high - wcs.f_low))
        end
    end
end

"""
    sp_inv(y)

Inverse of the softplus `sp(x) = ln(1 + eˣ)`, i.e. `ln(eʸ - 1)`. Written with
`expm1` so it stays accurate for small `y`, where `eʸ - 1` cancels. Defined for
`y > 0` only; `y -> 0` gives `-Inf`, which the callers turn into a zero speed.
"""
sp_inv(y) = y + log(-expm1(-y))

"""
    soft_min(a, b, beta)

Smooth approximation to `min(a, b)`: exact as `a == b`, and converges to `min(a, b)`
away from the crossing or as `beta -> Inf`. Always `<= min(a, b)`, by up to
`log(2) / beta` right at the crossing. `beta` sets the sharpness, in units of `1/[a]`.

Numerically stable log-sum-exp form: `log(exp(-beta*a) + exp(-beta*b)) =
-beta*min(a,b) + log1p(exp(-beta*|a-b|))`, so no term ever overflows.
"""
soft_min(a, b, beta) = min(a, b) - log1p(exp(-beta * abs(a - b))) / beta

# shared by calc_vro_soft and WinchController; see calc_vro_soft's docstring.
function _check_reel_in_beta(wcs::WCSettings, kv, f_low)
    wcs.reel_in_beta * kv * sqrt(f_low) >= 8 ||
        error("WCSettings.reel_in_beta = $(wcs.reel_in_beta) is too soft for " *
              "kv = $kv, f_low = $f_low: need reel_in_beta * kv * sqrt(f_low) >= 8 " *
              "(reel_in_beta >= $(8 / (kv * sqrt(f_low))) here), or the reel-in " *
              "line-to-tension-curve handover shows a visible hump instead of a smooth corner.")
end

"""
    calc_vro_soft(wcs::WCSettings, force, f_low=wcs.f_low; soft_lfc=wcs.soft_lfc,
                  use_awe_trim=wcs.use_awe_trim)

Reel-out speed under SOFT force limiting: the exact inverse of the tension curve
`T = (v/kv)²` soft-saturated at `f_high` and then at `f_low`, which is the curve
the trajectory optimizer plans against. Continuous everywhere, no threshold and
no state — the upper force limit is the law itself rather than a controller that
switches in.

The saturations are undone in the reverse of the order they are applied: the
lower one first, then the upper one. `wcs.v_sat` is load-bearing, not a guard —
the inverse diverges as `force` approaches `f_high`.

With `soft_lfc`, the law also replaces the `LowerForceController`: below
`f_low` it follows a straight line, SHIFTED DOWN by `log(2) / reel_in_beta`
from the one through `(0, wcs.v_reel_in)` and `(f_low, 0)` — the whole
physically valid range, since force is never negative, so no separate
ramp-width setting is needed — clamped at `wcs.v_reel_in` near `force = 0`.
Above `f_low` the returned speed is the [`soft_min`](@ref) of the UNSHIFTED
line and the tension-curve inverse — HARD-clamped at `f_low` from below (no
`softminus_beta` softening here; that parameter only smooths the non-`soft_lfc`
branch above, where there is no line to fall back on and a soft corner is
needed to avoid a dead band) — so the line governs near `f_low` and the
tension curve takes back over once it exceeds the line, with a smooth corner
rather than a kink where they cross — this also guarantees the command never
exceeds what the tension curve allows (`soft_min <= min`, always). The shift
matters only right at the boundary: AT `force = f_low`, the (unshifted) line
is `0` and the (hard-clamped) tension-curve inverse is `kv * sqrt(f_low)`, and
`soft_min(a, b, beta) = min(a, b) - log1p(exp(-beta*|a-b|))/beta`, so unless
the line below is shifted down to match, the two pieces disagree there by up
to `log(2) / reel_in_beta` (a genuine jump, not a kink, when `a == b`; here
the offset is smaller since `kv * sqrt(f_low) > 0`, but still real). Shifting
the WHOLE line, rather than blending it into `soft_min` across the whole
domain, is what keeps its slope exactly `m = -wcs.v_reel_in / f_low`
throughout the reel-in region instead of only asymptotically. `reel_in_beta`
is the SOLE tunable sharpness for this whole handover — changing it can never
un-straighten the line below `f_low` (only the constant shift moves), and
above `f_low` it purely controls how quickly `soft_min` settles onto the
tension curve. It must be sharp enough that the transition is actually
invisible rather than merely continuous: requires `wcs.reel_in_beta * wcs.kv
* sqrt(f_low) >= 8`, checked HERE (not just in [`WinchController`](@ref)'s
constructor, which this function bypasses whenever it is called directly, as
`CalcVSetIn` and `examples/plot_winch_curve.jl` do) — `kv * sqrt(f_low)` is
the tension-curve inverse's value right at `f_low` (its smallest possible
separation from the line, which starts at `0` there), so this is the same
"residual `<= exp(-8)`" margin `wcs.softminus_beta * f_low >= 8` uses for the
non-`soft_lfc` corner, applied to this one instead.

`use_awe_trim` blends this curve towards the curve that uses AWETrim's own
tension-curve constants (`k_v = 0.0408`, `f_min = 350`, `f_max = 8000`,
`softplus_beta = softminus_beta = 1e-3`, see `awetrim_tension` in
`examples/plot_winch_curve.jl`) instead of `wcs`'s and `f_low`'s. At `0.0`
(default) the law is exactly as before; at `1.0` it exactly reproduces
AWETrim's curve; both by the closed-form inverse, [`_calc_vro_soft`](@ref).
In between, the blend is done on the two FULL forward curves — force as a
function of SPEED, `(1 - use_awe_trim) * F_own(v) + use_awe_trim *
F_awe_trim(v)`, each obtained by numerically inverting [`_calc_vro_soft`](@ref)
(no closed form exists for that inverse once `soft_lfc` mixes in the reel-in
line) — and the blended forward curve is then itself inverted numerically to
get the speed for the given `force`. Blending forward (force at a given
speed), rather than the returned speed at a given force, is what makes the
result sit the same distance from both curves on a speed-vs-force plot: the
two curves are shaped very differently, so at a given FORCE their speeds can
be close while at a given SPEED their forces are far apart (or the reverse),
and it is the latter that a plot like `examples/plot_winch_curve.jl` shows.
Both the outer and the two inner root-finds are plain bisections (the
functions being inverted are monotonic by construction), 60 iterations each —
overkill for Float64 precision, but cheap next to the `nlsolve` calls
elsewhere in this file, even nested three deep.

## Parameters
- wcs::[WCSettings](@ref): the settings struct; reads `kv`, `f_high`, `v_sat`,
  both `beta`s and, under `soft_lfc`, `v_reel_in`/`reel_in_beta`
- force: the tether force at the winch [N], filtered by the caller
- `f_low`: the lower force limit [N]; per-call, since `calc_v_set` takes one
  too, and it also sets the reel-in line's slope under `soft_lfc`
- `soft_lfc`: selects the branch below `f_low`; defaults to `wcs.soft_lfc`
- `use_awe_trim`: blend factor in `[0, 1]` towards AWETrim's curve; defaults
  to `wcs.use_awe_trim` (itself `0.0`, i.e. current behaviour, unless set)

## Returns
- the reel-out speed [m/s]. With `soft_lfc = false` (the default), in
  `[0, wcs.v_sat]` — never negative, so reeling in stays the
  `LowerForceController`'s job. With `soft_lfc = true`, in
  `[wcs.v_reel_in, wcs.v_sat]`, and the `LowerForceController` must be held in
  reset (`WCSettings.soft_lfc` does this in [`WinchController`](@ref)).
"""
function calc_vro_soft(wcs::WCSettings, force, f_low=wcs.f_low; soft_lfc=wcs.soft_lfc,
                        use_awe_trim=wcs.use_awe_trim)
    0.0 <= use_awe_trim <= 1.0 || throw(ArgumentError("use_awe_trim must be in [0, 1], got $use_awe_trim"))
    use_awe_trim == 1.0 && return _calc_vro_soft(wcs, force, 350.0, soft_lfc, 0.0408, 8000.0, 1e-3, 1e-3)
    soft_lfc && _check_reel_in_beta(wcs, wcs.kv, f_low)
    own_softminus_beta = soft_lfc ? Inf : wcs.softminus_beta
    use_awe_trim == 0.0 && return _calc_vro_soft(wcs, force, f_low, soft_lfc, wcs.kv, wcs.f_high,
                                                  own_softminus_beta, wcs.softplus_beta)
    v_lo = soft_lfc ? wcs.v_reel_in : 0.0
    blended_force(v) = (1 - use_awe_trim) * _force_at_speed(wcs, v, f_low, soft_lfc, wcs.kv, wcs.f_high,
                                                              own_softminus_beta, wcs.softplus_beta) +
                        use_awe_trim * _force_at_speed(wcs, v, 350.0, soft_lfc, 0.0408, 8000.0, 1e-3, 1e-3)
    _bisect_increasing(blended_force, force, v_lo, wcs.v_sat)
end

# closed-form v(F) behind calc_vro_soft's endpoints and _force_at_speed's inner root-find.
# softminus_beta = Inf selects a HARD clamp at f_low (wcs's own soft_lfc = true curve,
# where reel_in_beta alone governs the handover sharpness, see calc_vro_soft's
# docstring); any finite value keeps the original soft corner there (AWETrim's OWN
# curve genuinely uses softminus_beta = 1e-3 -- an externally fixed fact about AWETrim,
# unrelated to wcs's redesign -- so its use_awe_trim = 1.0 endpoint must keep it).
function _calc_vro_soft(wcs::WCSettings, force, f_low, soft_lfc, kv, f_high,
                         softminus_beta, softplus_beta)
    if !soft_lfc
        force <= f_low && return 0.0
        force >= f_high && return wcs.v_sat
        t = f_low + sp_inv(softminus_beta * (force - f_low)) / softminus_beta
        t = f_high - sp_inv(softplus_beta * (f_high - t)) / softplus_beta
        return min(kv * sqrt(max(t, 0.0)), wcs.v_sat)
    end
    line = -wcs.v_reel_in / f_low * (force - f_low)
    if force <= f_low
        return max(line - log(2) / wcs.reel_in_beta, wcs.v_reel_in)
    end
    v_sqrt = if force >= f_high
        wcs.v_sat
    else
        t = isinf(softminus_beta) ? force :
            f_low + sp_inv(softminus_beta * (force - f_low)) / softminus_beta
        t = f_high - sp_inv(softplus_beta * (f_high - t)) / softplus_beta
        min(kv * sqrt(max(t, 0.0)), wcs.v_sat)
    end
    max(soft_min(line, v_sqrt, wcs.reel_in_beta), wcs.v_reel_in)
end

# bisection root-finder for a monotonic increasing scalar function `f`, such
# that `f(result) ≈ target`, searched over `v ∈ [lo, hi]`.
function _bisect_increasing(f, target, lo, hi; iters=60)
    for _ in 1:iters
        mid = 0.5 * (lo + hi)
        if f(mid) < target
            lo = mid
        else
            hi = mid
        end
    end
    0.5 * (lo + hi)
end

# forward inverse of `_calc_vro_soft`: the force that produces speed `v` under
# the given (kv, f_low, f_high, beta) parametrization. No closed form exists
# once `soft_lfc` mixes the reel-in line into the tension-curve inverse via
# `soft_min`, so `_calc_vro_soft` itself (monotonic increasing in force) is
# inverted numerically. `2 * f_high` comfortably covers the plateau where
# `_calc_vro_soft` already saturates at `wcs.v_sat`.
function _force_at_speed(wcs::WCSettings, v, f_low, soft_lfc, kv, f_high, softminus_beta, softplus_beta)
    _bisect_increasing(F -> _calc_vro_soft(wcs, F, f_low, soft_lfc, kv, f_high, softminus_beta, softplus_beta),
                        v, 0.0, 2 * f_high)
end

"""
    set_vset_pc(cvi::CalcVSetIn, v_set_pc, force)

## Parameters:
- force:      measured tether force [N]
- `v_set_pc`: only used during manual operation or park-at-length. If it is `nothing`,
              `v_set_in` is calculated as function of the force.

## Returns:
- nothing
"""
function set_vset_pc(cvi::CalcVSetIn, v_set_pc, force=nothing)
    if isnothing(v_set_pc)
        if cvi.wcs.force_limit == "soft"
            cvi.input_a = calc_vro_soft(cvi.wcs, calc_output(cvi.filter, force), cvi.f_low)
        else
            cvi.input_a = calc_vro(cvi.wcs, force)
        end
        select_b(cvi.mixer2, false)
    else
        cvi.input_b = v_set_pc
        select_b(cvi.mixer2, true)
    end
    nothing
end

"""
    calc_output(cvi::CalcVSetIn)

## Parameters
- cvi::CalcVSetIn: A struct of type CalcVSetIn

## Returns 
- `v_set_in`: Either `v_set`, or a value, proportional to the square root of the force.
"""
function calc_output(cvi::CalcVSetIn)
    calc_output(cvi.mixer2, cvi.input_a, cvi.input_b)
end

"""
    on_timer(cvi::CalcVSetIn)

Update the mixer. Must be called once per time-step.

## Parameters
- cvi::CalcVSetIn: Reference to the [CalcVSetIn](@ref) component

## Returns
- nothing
"""
function on_timer(cvi::CalcVSetIn)
    on_timer(cvi.mixer2)
    on_timer(cvi.filter)
    nothing
end

"""
    mutable struct Winch

Component, that calculates the acceleration of the tether based on the tether force
and the set speed (= synchronous speed). Asynchronous motor model and drum inertia
are taken into account. Used for testing of the winch controller.

## Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct Winch @deftype Float64
    wcs::WCSettings
    set::Settings
    wm::AsyncMachine = AsyncMachine(set)
    inertia = set.inertia_total * set.gear_ratio^2
    last_omega = 0.0 # last angular velocity of the asynchronous motor
    v_set     = 0 # input
    force     = 0 # input
    acc       = 0 # output
    jerk      = 0 # output; jerk of the winch; derivative of the acceleration
    speed     = 0 # output; reel-out speed; only state of this model
    p_dyn     = 0 # output; mechanical, dynamic power of the winch
end

# calculate the angular velocity of the winch drum
function calc_omega(set, v_ro)
    return v_ro / set.drum_radius
end
# calculate the angular acceleration of the winch drum
function calc_alpha(last_omega, omega, dt)
    return (omega - last_omega) / dt
end

function calc_dynamic_power(set, inertia, v_ro, last_omega, dt)
    omega = calc_omega(set, v_ro)
    alpha = calc_alpha(last_omega, omega, dt)
    return inertia * alpha * omega, omega
end

"""
    function Winch(wcs::WCSettings, set::Settings)

Constructor for a Winch struct, using the winch-controller settings and the general settings
as parameters.

## Parameters
- wcs::[WCSettings](@ref): settings of the winch controller
- set::[Settings](https://ufechner7.github.io/KiteUtils.jl/dev/types/#KiteUtils.Settings): general settings

## Returns
- a struct of type [Winch](@ref)
"""
function Winch(wcs::WCSettings, set::Settings)
    Winch(wcs=wcs, set=set)
end

"""
    function set_v_set(w::Winch, v_set)

Set the reel-out speed of the winch.

## Parameters
- w::[Winch](@ref): struct of type Winch
- v_set: new set value of the reel-out speed [m/s]

## Returns
- nothing
"""
function set_v_set(w::Winch, v_set)
    w.v_set = v_set
    nothing
end

"""
    function set_force(w::Winch, force)

Set the tether force at the winch.

## Parameters
- w::[Winch](@ref): struct of type Winch
- force: new set value of the tether force [N]

## Returns
- nothing
"""
function set_force(w::Winch, force)
    w.force = force
    nothing
end

"""
    function get_speed(w::Winch)

Read the tether speed of the winch.

## Parameters
- w::[Winch](@ref): struct of type Winch

## Returns
- the reel-out speed of the winch in m/s
"""
function get_speed(w::Winch) w.speed end

"""
    function get_acc(w::Winch)

Determine the current acceleration of the winch.

## Parameters
- w::[Winch](@ref): struct of type Winch

## Returns
- acceleration of the winch in m/s²
"""
function get_acc(w::Winch) w.acc end

"""
    on_timer(w::Winch)

Update the winch. Must be called once per time-step. calculates
and updates the winch acceleration `w.acc` using a loop.

## Parameters
- w::Winch: Reference to the [Winch](@ref) component

## Returns
- nothing
"""
function on_timer(w::Winch)
    last_acc = w.acc
    acc = 0.0
    for _ in 1:w.wcs.winch_iter
        w.acc = calc_acceleration(w.wm, w.speed, w.force; set_speed = w.v_set)
        acc += w.acc
        w.speed += w.acc * w.wcs.dt/w.wcs.winch_iter
    end
    w.acc = acc/w.wcs.winch_iter
    w.jerk = (w.acc - last_acc) / w.wcs.dt
    w.p_dyn, w.last_omega = calc_dynamic_power(w.set, w. inertia, w.speed, w.last_omega, w.wcs.dt)
    nothing
end

"""
    mutable struct SpeedController

PI controller for the reel-out speed of the winch in speed control mode.
While inactive, it tracks the value from the tracking input.
Back-calculation is used as anti-windup method and for tracking. The constant for
anti-windup is `K_b`, the constant for tracking `K_t`
Implements the following block diagram: ![speed_controller](assets/speed_controller.png)

## Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct SpeedController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    v_act = 0
    v_set_in = 0
    inactive::Bool = true
    tracking = 0
    v_err = 0         # output, calculated by solve
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{2, Float64} = zeros(2)
end
"""
    SpeedController(wcs::WCSettings)

Constructor for a SpeedController, based on the winch controller settings.

## Parameters
- wcs::WCSettings: the winch controller settings struct

## Returns
- a struct of type [SpeedController](@ref)
"""
function SpeedController(wcs::WCSettings)
    SpeedController(wcs=wcs)
end

"""
    set_inactive(sc::SpeedController, inactive::Bool)

De-activate the speed controller if the parameter inactive is true, otherwise
activate it and reset the integrator and the limiter.

## Parameters
- sc::[SpeedController](@ref): the speed controller to de-activate or activate

## Returns
- nothing
"""
function set_inactive(sc::SpeedController, inactive::Bool)
    # when it gets activated
    if sc.inactive && ! inactive
        reset(sc.integrator, sc.tracking)
        reset(sc.limiter, sc.tracking)
        sc.v_set_out = sc.tracking
    end
    sc.inactive = inactive
    nothing
end

"""
    set_v_act(sc::SpeedController, v_act)

Set the actual reel-out speed of the speed controller `sc` to `v_act`.

## Parameters
- sc::[SpeedController](@ref): the speed controller
- `v_act`: the actual reel-out speed

## Returns
- nothing
"""
function set_v_act(sc::SpeedController, v_act)
    sc.v_act = v_act
end

"""
    set_v_set(sc::SpeedController, v_set)

Set the set speed of the speed controller `sc` to `v_set`.

## Parameters
- sc::[SpeedController](@ref): the speed controller
- `v_set`: the set value of the reel-out speed

## Returns
- nothing
"""
function set_v_set(sc::SpeedController, v_set)
    reset(sc.integrator, v_set)
    nothing
end

"""
    set_v_set_in(sc::SpeedController, v_set_in)

Set the signal `v_set_in` of the speed controller to `v_set_in`.

## Parameters
- sc::[SpeedController](@ref): the speed controller
- `v_set_in`: the value to assign to the signal `v_set_in`

## Returns
- nothing
"""
function set_v_set_in(sc::SpeedController, v_set_in)
    sc.v_set_in = v_set_in
    nothing
end

"""
    set_tracking(sc::SpeedController, tracking)

Set the signal `tracking` of the speed controller to `tracking`.

## Parameters
- sc::[SpeedController](@ref): the speed controller
- `tracking`: the value to assign to the signal `tracking`

## Returns
- nothing
"""
function set_tracking(sc::SpeedController, tracking)
    sc.tracking = tracking
end

function calc_sat2in_sat2out_rateout_intin(sc::SpeedController, x)
    kb_in = x[begin]
    kt_in = x[begin+1]
    int_in = sc.wcs.i_speed * sc.sat_out + sc.wcs.kb_speed * kb_in + sc.wcs.kt_speed * kt_in * sc.inactive
    int_out = calc_output(sc.integrator, int_in)
    sat2_in = int_out + sc.wcs.p_speed * calc_output(sc.delay, sc.sat_out)
    sat2_out = saturate(sat2_in, -sc.wcs.v_ri_max, sc.wcs.v_sat)
    rate_out = calc_output(sc.limiter, sat2_out)
    sat2_in, sat2_out, rate_out, int_in
end

function solve(sc::SpeedController)
    # Function, that calculates the residual for the given kb_in and kt_in estimates
    # of the feed-back loop of the integrator.
    function calc_residual!(F, x)
        sat2_in, sat2_out, rate_out, int_in = calc_sat2in_sat2out_rateout_intin(sc, x)
        kt_in = sc.tracking - sat2_out
        kb_in = sat2_out - sat2_in
        sc.res[begin]   = kb_in - x[begin]
        sc.res[begin+1] = kt_in - x[begin+1]
        F .= sc.res 
    end

    err = sc.v_set_in - sc.v_act
    if sc.inactive
        sc.v_err = NaN
    else
        sc.v_err = err
    end
    sc.sat_out = saturate(err, -sc.wcs.v_sat_error, sc.wcs.v_sat_error)
    # begin iterate
    sol = nlsolve(calc_residual!, [ 0.0; 0.0], iterations=sc.wcs.max_iter)
    @assert sol.f_converged
    sc.wcs.iter = max(sol.iterations, sc.wcs.iter)
    sat2_in, sat2_out, rate_out, _ = calc_sat2in_sat2out_rateout_intin(sc, sol.zero)
    sc.v_set_out = rate_out
end

"""
    on_timer(sc::SpeedController)

Update the SpeedController. Must be called once per time-step. 

## Parameters
- sc::SpeedController Reference to the [SpeedController](@ref) component

## Returns
- nothing
"""
function on_timer(sc::SpeedController)
    on_timer(sc.limiter)
    on_timer(sc.integrator)
    on_timer(sc.delay)
end

"""
    get_v_set_out(sc::SpeedController)

Calculate the output value of the controller by using a non-linear solver.

## Parameters
- sc::SpeedController Reference to the [SpeedController](@ref) component

## Returns
- `v_set_out`: the synchronous speed, calculated by the controller
"""
function get_v_set_out(sc::SpeedController)
    solve(sc)
end

"""
    get_v_err(sc::SpeedController)

Compute and return the velocity error for the given `SpeedController` instance `sc`.

# Arguments
- sc::[SpeedController](@ref): The speed controller object for which the velocity error is to be calculated.

# Returns
- The velocity error `v_err` [m/s]. 
  If the controller is inactive, it returns `NaN`.
"""
function get_v_err(sc::SpeedController)
    if sc.inactive
        return NaN
    else
        return sc.v_err
    end
end

"""
    mutable struct LowerForceController <: AbstractForceController

PI controller for the lower force of the tether.
While inactive, it tracks the value from the tracking input.
Back-calculation is used as anti-windup method and for tracking. The constant for
anti-windup is `K_b`, the constant for tracking `K_t`
Implements the following block diagram: ![lower_force_controller](assets/lower_force_controller.png)

## Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct LowerForceController <: AbstractForceController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    int2::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    reset::Bool = false
    active::Bool = false
    force = 0
    f_set = 0
    v_sw = 0
    v_act = 0
    tracking = 0
    f_err = 0         # output, calculated by solve
    last_err = 0
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{3, Float64} = zeros(3)
end

"""
    LowerForceController(wcs::WCSettings)

Constructor for a LowerForceController, based on the winch controller settings.

## Parameters
- wcs::[WCSettings](@ref): the winch controller settings struct

## Returns
- a struct of type [LowerForceController](@ref)
"""
function LowerForceController(wcs::WCSettings)
    LowerForceController(wcs=wcs)
end

# internal method to set the SR flip-flop and activate the force controller
function _set(lfc::LowerForceController)
    if lfc.reset return end
    if ! lfc.active
       reset(lfc.integrator, lfc.tracking)
       reset(lfc.int2, 0.0)
       reset(lfc.limiter, lfc.tracking)
       lfc.v_set_out = lfc.tracking
    end
    lfc.active = true
end

function _update_reset(fc::LowerForceController)
    if (fc.v_act - fc.v_sw) >= 0.0 || fc.reset
        if (fc.force - fc.f_set) > 0.0 || fc.reset
            fc.active = false
        end
    end
end

"""
    set_v_act(fc::AFC, v_act)

Set the signal `v_act` of the force controller to `v_act`.

## Parameters
- sc::AFC: abstract force controller
- `v_act`: the value to assign to the signal `v_act`

## Returns
- nothing
"""
function set_v_act(fc::AFC, v_act)
    fc.v_act = v_act
    nothing
end

"""
    set_force(fc::AFC, force)

Set the signal `force` of the force controller to `force`.

## Parameters
- sc::AFC: abstract force controller
- `force`: the value to assign to the signal `force`

## Returns
- nothing
"""
function set_force(fc::AFC, force)
    fc.force = force
    nothing
end

"""
    set_reset(fc::AFC, reset)

Set the signal `reset` of the force controller to `reset` and activate
or de-activate the controller.

## Parameters
- sc::AFC: abstract force controller
- `reset`: the value to assign to the signal `reset`

## Returns
- nothing
"""
function set_reset(fc::AFC, reset)
    fc.reset = reset
    _update_reset(fc)
    nothing
end

"""
    set_f_set(fc::AFC, f_set)

Set the set force of the force controller to `f_set`.

## Parameters
- sc::AFC: abstract force controller
- `f_set`: the value to assign to the signal `f_set`

## Returns
- nothing
"""
function set_f_set(fc::AFC, f_set)
    fc.f_set = f_set
    nothing
end

"""
    set_v_sw(fc::AFC, v_sw)

## Parameters
- sc::AFC: abstract force controller
- `v_sw`: the value to assign to the signal `v_sw`

## Returns
- nothing
"""
function set_v_sw(fc::AFC, v_sw)
    fc.v_sw = v_sw
    nothing
end

"""
    set_tracking(fc::AFC, tracking)

## Set the signal `tracking` of the force controller to `tracking`.

## Parameters
- sc::AFC: abstract force controller
- `tracking`: the value to assign to the signal `tracking`

## Returns
- nothing
"""
function set_tracking(fc::AFC, tracking)
    fc.tracking = tracking
end

function calc_sat2in_sat2out_rateout_intin(lfc::LowerForceController, x)
    kb_in = x[begin]
    kt_in = x[begin+1]
    int2_in = x[begin+2]
    int_in = if_low_scaled(lfc.wcs) * lfc.f_err + lfc.wcs.kbf_low * kb_in + lfc.wcs.ktf_low * kt_in * (! lfc.active)
    int_out = calc_output(lfc.integrator, int_in)
    int2_out = calc_output(lfc.int2, int2_in)
    sat2_in = int_out + pf_low_scaled(lfc.wcs) * calc_output(lfc.delay, lfc.f_err*abs(lfc.f_err/150)^0.7) +
              (lfc.f_err- lfc.last_err)/ lfc.wcs.dt * lfc.wcs.df_low
            #lfc.wcs.nf_low * ((lfc.f_err- lfc.last_err)/ lfc.wcs.dt * lfc.wcs.df_low - int2_out)
    sat2_out = saturate(sat2_in, -lfc.wcs.v_ri_max, lfc.wcs.v_sat)
    rate_out = calc_output(lfc.limiter, sat2_out)
    sat2_in, sat2_out, rate_out, int_in, int2_in
end

function solve(lfc::LowerForceController)
    # Function, that calculates the residual for the given kb_in and kt_in estimates
    # of the feed-back loop of the integrator.
    function calc_residual!(F, x)
        sat2_in, sat2_out, rate_out, int_in, int2_in = calc_sat2in_sat2out_rateout_intin(lfc, x)
        kt_in = lfc.tracking - sat2_out
        kb_in = rate_out - sat2_in
        lfc.res[begin]   = kb_in - x[begin]
        lfc.res[begin+1] = kt_in - x[begin+1]
        lfc.res[begin+2] = int2_in - x[begin+2]
        F .= lfc.res 
    end

    _update_reset(lfc::LowerForceController)
    lfc.last_err = lfc.f_err
    err = lfc.force - lfc.f_set
    if ! lfc.active
        # activate the force controller if the force drops below the set force
        if err < 0.0
            _set(lfc)
        end
        lfc.f_err = 0.0
    else
        lfc.f_err = err
    end
    sol = nlsolve(calc_residual!, [0.0; 0.0; 0.0], iterations=lfc.wcs.max_iter)
    @assert sol.f_converged
    lfc.wcs.iter = max(sol.iterations, lfc.wcs.iter)
    sat2_in, sat2_out, rate_out, int_in, int2_in = calc_sat2in_sat2out_rateout_intin(lfc, sol.zero)
    lfc.v_set_out = rate_out
end

"""
    get_v_set_out(fc::AFC)

Calculate the output value of the controller by using a non-linear solver.

## Parameters
- fc::AFC: abstract force controller

## Returns
- `v_set_out`: the synchronous speed, calculated by the controller
"""
function get_v_set_out(fc::AFC)
    solve(fc)
    fc.v_set_out
end

"""
    get_f_err(fc::AFC)

Get the error of the force controller.

## Parameters
- fc::AFC: abstract force controller

## Returns
- `f_err`: the error of the force controller
"""
function get_f_err(fc::AFC)
    fc.f_err
end

"""
    get_f_set_low(lfc::LowerForceController)

Returns the lower force setpoint for the given `LowerForceController` instance `lfc`.

# Arguments
- `lfc::LowerForceController`: The lower force controller object from which to retrieve the setpoint.

# Returns
- The lower force setpoint value associated with the controller.

"""
function get_f_set_low(lfc::LowerForceController)
    lfc.active * lfc.f_set
end

"""
    on_timer(lfc::LowerForceController)

Callback function that is triggered on a timer event for a `LowerForceController` instance.
This function handles periodic updates or control logic that needs to be executed
at regular intervals for the lower force controller.

# Arguments
- `lfc::LowerForceController`: The instance of `LowerForceController` for which the timer event is handled.

# Returns
- nothing

"""
function on_timer(lfc::LowerForceController)
    on_timer(lfc.limiter)
    on_timer(lfc.integrator)
    on_timer(lfc.int2)
    on_timer(lfc.delay)
    nothing
end


"""
    mutable struct UpperForceController <: AbstractForceController 

PID controller for the upper force of the tether.
While inactive, it tracks the value from the tracking input.
Back-calculation is used as anti-windup method and for tracking. The constant for
anti-windup is `K_b`, the constant for tracking `K_t`
Implements the following block diagram: ![upper_force_controller](assets/upper_force_controller.png)


# Fields
$(TYPEDFIELDS)

# Usage
Create an instance to control the upper force limit in a winch system.
"""
@with_kw mutable struct UpperForceController <: AbstractForceController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    int2::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    reset::Bool = false
    active::Bool = false
    f_set = 0
    v_sw = 0
    v_act = 0
    force = 0
    tracking = 0
    f_err = 0         # output, calculated by solve
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{3, Float64} = zeros(3)
end

"""
    UpperForceController(wcs::WCSettings)

Creates and returns an upper force controller using the provided `WCSettings`.

# Arguments
- wcs::[WCSettings](@ref): The settings structure containing configuration parameters for the winch controller.

# Returns
- An instance of the upper force controller configured according to the provided settings.

"""
function UpperForceController(wcs::WCSettings)
    UpperForceController(wcs=wcs, f_set=wcs.f_high)
end

# internal method to set the SR flip-flop and activate the force controller
function _set(ufc::UpperForceController)
    if ufc.reset return end
    if ! ufc.active
       reset(ufc.integrator, ufc.tracking)
       reset(ufc.int2, 0.0)
       reset(ufc.limiter, ufc.tracking)
       ufc.v_set_out = ufc.tracking
    end
    ufc.active = true
end

function _update_reset(fc::UpperForceController)
    if (fc.v_act - fc.v_sw) <= 0.0 || fc.reset
        if (fc.force - fc.f_set) < 0.0 || fc.reset
            fc.active = false
        end
    end
end

function calc_sat2in_sat2out_rateout_intin(ufc::UpperForceController, x)
    kb_in   = x[begin]
    kt_in   = x[begin+1]
    int2_in = x[begin+2]
    int_in = ufc.wcs.if_high * ufc.f_err + ufc.wcs.kbf_high * kb_in + ufc.wcs.ktf_high * kt_in * (! ufc.active)
    int_out = calc_output(ufc.integrator, int_in)
    int2_out = calc_output(ufc.int2, int2_in)
    sat2_in = int_out + ufc.wcs.pf_high * calc_output(ufc.delay, ufc.f_err) +
              ufc.wcs.nf_high * (ufc.f_err * ufc.wcs.df_high - int2_out)
    sat2_out = saturate(sat2_in, -ufc.wcs.v_ri_max, ufc.wcs.v_sat)
    rate_out = calc_output(ufc.limiter, sat2_out)
    sat2_in, sat2_out, rate_out, int_in, int2_in
end

function solve(ufc::UpperForceController)
    # Function, that calculates the residual for the given kb_in and kt_in estimates
    # of the feed-back loop of the integrator.
    function calc_residual!(F, x)
        sat2_in, sat2_out, rate_out, int_in, int2_in = calc_sat2in_sat2out_rateout_intin(ufc, x)
        kt_in = ufc.tracking - sat2_out
        kb_in = rate_out - sat2_in
        ufc.res[begin]   = kb_in   - x[begin]
        ufc.res[begin+1] = kt_in   - x[begin+1]
        ufc.res[begin+2] = int2_in - x[begin+2]
        F .= ufc.res 
    end

    _update_reset(ufc)
    err = ufc.force - ufc.f_set
    if ! ufc.active
        # activate the force controller if the force rises above the set force
        if err >= 0.0
            _set(ufc)
        end
        ufc.f_err = 0.0
    else
        ufc.f_err = err
    end
    sol = nlsolve(calc_residual!, [0.0; 0.0; 0.0], iterations=ufc.wcs.max_iter)
    @assert sol.f_converged
    ufc.wcs.iter = max(sol.iterations, ufc.wcs.iter)
    sat2_in, sat2_out, rate_out, int_in, int2_in = calc_sat2in_sat2out_rateout_intin(ufc, sol.zero)
    ufc.v_set_out = rate_out
end

"""
    get_f_set_upper(ufc::UpperForceController)

Returns the setpoint force for the given `UpperForceController` instance `ufc`.

# Arguments
- ufc::[UpperForceController](@ref): The upper force controller object from which to retrieve the force setpoint.

# Returns
- The setpoint force of the upper force controller [N].

"""
function get_f_set_upper(ufc::UpperForceController)
    ufc.active * ufc.f_set
end

"""
    on_timer(ufc::UpperForceController)

Callback function that is triggered on a timer event for an `UpperForceController` instance.
This function is used to perform periodic updates and checks related to the controller's operation.

# Arguments
- `ufc::UpperForceController`: The upper force controller instance on which the timer event is triggered.

# Returns
- nothing

"""
function on_timer(ufc::UpperForceController)
    on_timer(ufc.limiter)
    on_timer(ufc.integrator)
    on_timer(ufc.int2)
    on_timer(ufc.delay)
    nothing
end
