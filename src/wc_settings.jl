"""
    mutable struct WCSettings

Settings of the WinchController. See also: [Winchcontroller Settings](@ref).

Covers BOTH winch control families in one struct, because a run configures one
file, not one per controller:

- The speed/force controller of [`WinchController`](@ref) — the historical
  content of this struct, everything except the `winch_*` block at the end.
  Its output is a reel-out SPEED.
- The cascaded length controller ([`winch_position_torque!`](@ref)) and force
  mode ([`winch_force_torque!`](@ref)) — the `winch_pos_*`/`winch_speed_*`/
  `winch_ff_scale` and `winch_force_*`/`winch_len_kp`/`winch_damp` fields.
  Their output is a winch TORQUE. 

A run uses one mode; they share only the drum.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct WCSettings @deftype Float64
    """
    Timestep of the winch controller [s]. Defaults to `NaN` — every caller must
    overwrite it with the plant's own timestep, and a `NaN` makes a missing one
    fail loudly (e.g. `DiscretePID(; Ts=NaN, ...)`) instead of silently running
    at the wrong rate. The field exists at all so that `WCSettings()` works for
    the torque controllers below, which read their gains from a
    default-constructed instance without touching `dt`.
    """
    dt = NaN
    "set to true for running the unit tests"
    test::Bool = false
    "`\"piecewise\"` (default) uses `vf_max`/`f_low`/`f_high`; `\"reelout\"` uses the unsigned `kv * sqrt(force)` law. `test = true` also selects `\"reelout\"`, kept for the existing tests"
    mode::String = "piecewise"
    "factor for I and P of lower force controller"
    fac = 0.25
    "max iterations limit for the PID solvers"
    max_iter::Int64 = 100
    "actual max iterations of the PID solvers"
    iter::Int64 = 0
    "startup time for soft start"
    t_startup = 0.25
    "blending time of the mixers in seconds"
    t_blend = 0.25
    "limitation of the reel-out speed error, used by the input saturation block of the speed controller"
    v_sat_error = 1.0
    "limitation of the reel-out speed , used by the output saturation block of the speed controller"
    v_sat = 8.0 # was: 8.0
    "maximal reel-in speed [m/s]"
    v_ri_max = 8.0
    "P value of the speed controller"
    p_speed = 0.125
    "I value of the speed controller"
    i_speed = 4.0
    "back calculation constant for the anti-windup loop of the speed controller"
    kb_speed = 4.0
    "tracking constant of the speed controller"
    kt_speed = 5.0
    "reel-out velocity where the set force should reach it's maximum"
    vf_max = 2.75
    "P constant of the lower force controller"
    pf_low = 1.44e-4
    "I constant of the lower force controller"
    if_low = 7.5e-3 * 1.5
    "D constant of lower force controller"
    df_low =  2e-5*1.7
    "filter constant n of upper force controller"
    nf_low = 7.0
    "back calculation constant for the anti-windup loop of the lower force controller"
    kbf_low = 1.0
    "tracking constant of the lower force controller"
    ktf_low = 8.0
    "lower force limit [N]"
    f_low = 350
    "set force for reel-in phase [N]"
    f_reelin = 700
    "upper force limit [N]"
    f_high = 3800
    "P constant of upper force controller"
    pf_high = 1.44e-4*1.6
    "I constant of upper force controller"
    if_high = 7.5e-3*1.6
    "D constant of upper force controller"
    df_high =  2e-5*1.7
    "filter constant n of upper force controller"
    nf_high = 15.0
    "back calculation constant for the anti-windup loop of the upper force controller"
    kbf_high = 1.0
    "tracking constant of the upper force controller"
    ktf_high = 10.0
    "interations of the winch model"
    winch_iter = 10
    "maximal acceleration of the winch (derivative of the set value of the reel-out speed)"
    max_acc = 8.0
    "damage at max acceleration"
    damage_factor = 0.2 
    "jerk factor for damage calculation"
    jerk_factor = 0.9
    "proportional factor of the square root law, see function calc_vro"
    kv = 0.06

    # ---- Torque controllers: the cascaded length loop and force mode -------- #
    # Moved from V3Kite.jl's own `WC_Settings` struct. These are new parameters
    # with no historical counterpart; tune on a constant-length parking run and
    # change them by at most 10 % per iteration.
    "Outer proportional gain [1/s]: tether length error [m] → reel-out speed setpoint [m/s]"
    winch_pos_kp = 0.5
    "Inner speed-loop proportional gain [N·m·s/m]: speed error [m/s] → winch torque [N·m]"
    winch_speed_k = 30.0
    "Inner speed-loop integral time [s] (larger = weaker integral action)"
    winch_speed_ti = 2.0
    "Saturation of the inner speed loop's torque correction [N·m]"
    winch_torque_limit = 500.0
    """
    Scale on the load term of the force feed-forward,
    `τ = -ff_scale·r/G·F + friction + Δτ`. The friction compensation is never
    scaled: it cancels the drum's own friction, not the load, and flips sign with
    the reel-out direction.

    `1.0` (the default) cancels the measured tether force exactly, making the drum
    perfectly stiff at any PI gain; below 1.0 the holding torque falls short and
    the drum pays out. The compliance is transient unless `winch_speed_ti` is well
    above the timescale you want the winch to yield on, since the inner PI
    integrates the resulting speed error away.
    """
    winch_ff_scale = 1.0
    """
    Force mode only ([`winch_force_torque!`](@ref)): time constant of the low-pass
    that turns the measured winch force into the reference force [s]. The drum
    yields to everything faster than this and holds everything slower, so this is
    the winch's compliance timescale — it should sit above the lap rate to let the
    kite trade line length for speed within a lap.
    """
    winch_force_tau = 10.0
    "Force mode only: length-trim gain [N/m], length error [m] → reference force [N]"
    winch_len_kp = 100.0
    """
    Force mode only: viscous damping on the drum [N·s/m], reel-out speed [m/s] →
    reference force [N]. Without it force mode has no velocity feedback at all — the drum
    is a free mass on the `winch_len_kp` spring and its speed runs away (measured:
    3.5 m/s of payout, `v_app` 49.6 m/s, run lost at t = 19.8 s). This is what makes
    the winch compliant rather than free-wheeling; it opposes fast reel-out without
    pinning the length the way the position cascade does.
    """
    winch_damp = 500.0
    "Force mode only: floor on the reference force, keeps the tether taut [N]"
    winch_force_min = 100.0

    # ---- Soft force limiting (REEL_OUT mode) -------------------------------- #
    # New fields go HERE, at the end: the positional `WCSettings(update; dt)`
    # constructor other packages use keeps working, and YAML falls back to the
    # struct defaults for every key a file does not carry.
    """
    How the force limits are enforced, orthogonal to `mode`, which selects the
    law's shape. `"hard"` (the default) is the historical behaviour: the bare
    law of [`calc_vro`](@ref) plus the `LowerForceController` and the
    `UpperForceController` switching in at the limits.

    `"soft"` replaces the UPPER limiter with a static, continuous law
    ([`calc_vro_soft`](@ref)) that inverts a tension curve saturating smoothly at
    `f_low` and `f_high` — no threshold, no hand-over, no integrator. The
    `UpperForceController` is then held in reset for the whole run, because the
    law itself is the upper limiter. The `LowerForceController` stays UNLESS
    `soft_reel_in` is also set, in which case it is held in reset too and the
    same law commands reel-in below `f_low`. Requires `mode == "reelout"`.
    """
    force_limit::String = "hard"
    """
    `"soft"` only: if `true`, [`calc_vro_soft`](@ref) also replaces the
    `LowerForceController` — a straight line through `(f_low, 0)` and
    `(f_low - f_reel_in_band, v_reel_in)`, capped where the reel-out tension
    curve overtakes it. Requires `softminus_beta * f_reel_in_band >= 2`: the
    lower corner must be sharp enough that the tension curve itself reaches 0
    at `f_low`, rather than somewhere above it — a wide corner otherwise
    leaves a dead band of exactly-zero speed between the reel-in ramp and the
    reel-out curve, with nothing left to bridge it once the
    `LowerForceController` is gone. The shipped `softminus_beta` default fails
    this check by a factor of ~27.
    """
    soft_reel_in::Bool = false
    """
    `soft_reel_in` only: width of the linear reel-in ramp below `f_low` [N].
    Together with `v_reel_in` this fixes the ramp's slope; the ramp then
    governs the commanded speed until the reel-out tension curve overtakes it,
    which on this winch's numbers is well above `f_low + f_reel_in_band`.
    """
    f_reel_in_band = 75.0
    """
    `soft_reel_in` only: reel-in speed [m/s] at `f_low - f_reel_in_band`, and
    the clamp for every force below it. Must satisfy
    `-v_ri_max <= v_reel_in < 0`.
    """
    v_reel_in = -0.3
    """
    `"soft"` only: corner sharpness of the UPPER limit [1/N]. The transition
    spans a tension band of order `1/softplus_beta`, so LARGER is sharper; at
    `f_high` itself the curve sits `ln(2)/softplus_beta` below the limit. Must
    match the value the trajectory optimizer applies to the same curve — both
    sides default to 1e-3.
    """
    softplus_beta = 1e-3
    """
    `"soft"` only: corner sharpness of the LOWER limit [1/N], as
    `softplus_beta`. Watch this one: the effective floor is
    `sp(beta*f_low)/beta`, so a `1/beta` larger than `f_low` itself dominates the
    limit it smooths — at 1e-3 an `f_low` of 700 N acts as 1103 N. Under
    `soft_reel_in` this must also satisfy `softminus_beta * f_reel_in_band >= 2`
    (validated in [`WinchController`](@ref)), or the same effect leaves a dead
    band of zero speed above `f_low`.
    """
    softminus_beta = 1e-3
    """
    `"soft"` only: time constant of the low-pass on the measured force before it
    is inverted [s]; `0` disables the filter. The inverse is near-vertical close
    to `f_high` — `dv/dF` rises by a factor 50 over the last kN — so the force
    ripple of a single figure-eight lap would otherwise be amplified straight
    into the speed command.
    """
    force_limit_tau = 1.0
    """
    `"soft"` only: time constant of the same low-pass while the force is RISING
    [s]. `NaN` — the default — means "the same as `force_limit_tau`", i.e. a
    symmetric filter, and is resolved where the filter is BUILT rather than here:
    a plain default of `force_limit_tau` would freeze the struct default 1.0 and
    leave a YAML that sets only `force_limit_tau` silently asymmetric.

    Setting it BELOW `force_limit_tau` gives a fast attack with a slow release: the limiter follows
    a force spike up promptly but gives the speed back slowly, so the noise
    attenuation that keeps the loop stable is retained on the way down.

    This makes the filter non-linear. Its output is biased toward the PEAKS of an
    oscillating force rather than its mean, so the law commands a higher speed for
    the same mean force — better peak limiting, less power.
    """
    force_limit_tau_rise = NaN
end

function pf_low_scaled(wcs::WCSettings)
   wcs.pf_low * wcs.fac
end

function if_low_scaled(wcs::WCSettings)
   wcs.if_low * wcs.fac
end

StructTypes.StructType(::Type{WCSettings}) = StructTypes.Mutable()

function update(wcs::WCSettings)
    config_file = joinpath(get_data_path(), wc_settings())
    if Sys.iswindows()
        config_file = replace(config_file, "/" => "\\")
    end
    if ! isfile(config_file)
        println("Warning: $config_file not found, using default settings.")
        return
    end
    dict = YAML.load_file(config_file)
    sec_dict = Dict(Symbol(k) => v for (k, v) in dict["wc_settings"])
    StructTypes.constructfrom!(wcs, sec_dict)
end

function WCSettings(update; dt)
    wcs = WCSettings(; dt)
    if update
        WinchControllers.update(wcs)
    end
    wcs.dt = dt
    wcs
end
