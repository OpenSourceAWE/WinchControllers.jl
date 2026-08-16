# Copyright (c) 2025 Jelle Poland, Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

# Winch controllers whose output is a winch TORQUE, as opposed to the reel-out
# SPEED that `WinchController` (winchcontroller.jl) produces. Moved here from
# V3Kite.jl so that all winch control lives in one package.
#
# Everything here is deliberately PLANT-AGNOSTIC: the functions take plain
# scalars (force, speed, length, and the drum's radius/gear ratio/friction)
# rather than a kite-model object, so this package needs no dependency on any
# particular model. A model wraps them in a one-line method of its own; see
# V3Kite.jl's `winch_position_torque!(s::V3KITE, ...)`.

"""
    force_to_torque(force, drum_radius, gear_ratio, friction; ff_scale=1.0)

Convert tether force [N] to winch torque [N·m], `τ = -ff_scale * r/G * F + friction`.

`ff_scale` scales the load-canceling term only; the friction term is always
applied in full. Friction compensation cancels the drum's own coulomb/viscous
torque and is not part of the load being held, and `friction` flips sign with
the reel-out direction — scaling it would make the intended compliance
direction- and speed-dependent instead of a plain fraction of the tether force.
"""
function force_to_torque(force, drum_radius, gear_ratio, friction; ff_scale = 1.0)
    return -ff_scale * drum_radius / gear_ratio * force + friction
end

"""
    winch_acc_limit(max_acc) -> Float64

The winch acceleration limit [m/s²] in the form the rate limiter of
[`winch_position_torque!`](@ref) wants. A non-positive `max_acc` — means
*unlimited*, not a frozen drum, and maps to `Inf`.
"""
winch_acc_limit(max_acc) = max_acc > 0 ? Float64(max_acc) : Inf

"""
    WinchPosController(wcs::WCSettings; dt=wcs.dt)

Runtime state of the cascaded winch length controller: an outer proportional
loop on the tether length error produces a speed setpoint (saturated by
`speed_limit` and rate-limited by `acceleration_limit`), added to the optional
speed feed-forward `v_ff`, and an inner PI loop (`speed_pid`) on the speed error
produces a winch-torque correction added to a force feed-forward. `v_sp_prev`
carries the rate-limited speed setpoint between steps.

Gains come from a [`WCSettings`](@ref) struct (`winch_pos_kp`, `winch_speed_k`,
`winch_speed_ti`, `winch_torque_limit`, `winch_ff_scale`).

# Fields

$(TYPEDFIELDS)
"""
Base.@kwdef mutable struct WinchPosController
    "Inner speed PI controller; output is a winch torque correction [N·m]"
    speed_pid::DiscretePID
    "Outer proportional gain, length error [m] → speed setpoint [m/s]"
    kp_pos::Float64 = WCSettings().winch_pos_kp
    "Scale on the force feed-forward; < 1 makes the drum pay out under load"
    ff_scale::Float64 = WCSettings().winch_ff_scale
    "Rate-limited speed setpoint carried between steps [m/s]"
    v_sp_prev::Float64 = 0.0
end

function WinchPosController(wcs::WCSettings; dt = wcs.dt)
    speed_pid = DiscretePID(; K = wcs.winch_speed_k, Ti = wcs.winch_speed_ti,
                            Ts = dt, umin = -wcs.winch_torque_limit,
                            umax = wcs.winch_torque_limit)
    WinchPosController(; speed_pid, kp_pos = wcs.winch_pos_kp,
                       ff_scale = wcs.winch_ff_scale)
end

"""
    WinchForceController(; force_tau, len_kp, damp, force_min)
    WinchForceController(wcs::WCSettings)

Runtime state and gains of the force-mode winch (see
[`winch_force_torque!`](@ref)). Deliberately a separate object from
[`WinchPosController`](@ref): a run holds either a length or a force, and the
two modes share no gain — `winch_force_torque!` never reads `kp_pos`,
`speed_pid` or `ff_scale`.

`f_lpf` is the low-passed reference force, `NaN` until the first step, which is
what makes engaging force mode from a settled state produce no torque step.

# Fields

$(TYPEDFIELDS)
"""
Base.@kwdef mutable struct WinchForceController
    "Time constant of the reference-force low-pass [s]"
    force_tau::Float64 = WCSettings().winch_force_tau
    "Length-trim gain, length error [m] → reference force [N/m]"
    len_kp::Float64 = WCSettings().winch_len_kp
    "Viscous damping on the drum, reel-out speed [m/s] → reference force [N·s/m]"
    damp::Float64 = WCSettings().winch_damp
    "Floor on the reference force, keeps the tether taut [N]"
    force_min::Float64 = WCSettings().winch_force_min
    "Low-passed reference force, `NaN` before the first force-mode step [N]"
    f_lpf::Float64 = NaN
end

WinchForceController(wcs::WCSettings) = WinchForceController(
    force_tau = wcs.winch_force_tau, len_kp = wcs.winch_len_kp,
    damp = wcs.winch_damp, force_min = wcs.winch_force_min)

"""
    winch_position_torque!(wpc::WinchPosController, set_length, l_actual, speed,
                            force, drum_radius, gear_ratio, friction, dt,
                            speed_limit, acceleration_limit; v_ff=0.0) -> torque

Cascaded winch length controller. Outer proportional loop on the tether
length error (`set_length - l_actual`) yields a reel-out speed setpoint, added to
the speed feed-forward `v_ff` [m/s], then clamped to `±speed_limit` [m/s] and
rate-limited by `acceleration_limit` [m/s²]. The inner PI loop (`wpc.speed_pid`)
on the speed error yields a winch-torque correction, added to the force
feed-forward `force_to_torque(force, drum_radius, gear_ratio, friction;
ff_scale)` — the holding torque for the measured winch force, whose load term
`winch_ff_scale` scales (see [`WCSettings`](@ref)). Returns the winch set torque
[N·m], applied directly: at parking equilibrium with `ff_scale = 1` the
correction vanishes and the output is the holding torque.

`v_ff` is the speed the caller is *commanding*, not a measured one, and it
belongs here whenever `set_length` is the running integral of a speed setpoint
(reel-out). Without it the outer P loop has to rediscover that speed from a
length error, and integrator-then-P is a first-order lag of `1/kp_pos` — 2 s at
the default `winch_pos_kp = 0.5`, which halves the amplitude of a 6 s reel-out
oscillation and delays it by over a second, on top of a standing length error of
`v/kp_pos`. Passing `v_ff = v_set` leaves the outer loop only the length
*error* to correct and removes both. The default `0.0` reproduces the
pure-feedback behaviour exactly, so a caller holding a constant length is
unaffected.
"""
function winch_position_torque!(wpc::WinchPosController, set_length, l_actual, speed,
                                 force, drum_radius, gear_ratio, friction, dt,
                                 speed_limit, acceleration_limit; v_ff = 0.0)
    # Outer P loop: length error → speed setpoint, on top of the commanded speed.
    v_sp = v_ff + wpc.kp_pos * (set_length - l_actual)
    # Both limits act on the TOTAL setpoint: they are the drum's, not the loop's.
    v_sp = clamp(v_sp, -speed_limit, speed_limit)
    dv_max = acceleration_limit * dt
    v_sp = clamp(v_sp, wpc.v_sp_prev - dv_max, wpc.v_sp_prev + dv_max)
    wpc.v_sp_prev = v_sp
    # Inner PI loop: speed error → torque correction.
    dtau = wpc.speed_pid(v_sp, speed, 0.0)
    # Only the load term is scaled; friction compensation stays at full size.
    tau_ff = force_to_torque(force, drum_radius, gear_ratio, friction;
                             ff_scale = wpc.ff_scale)
    return tau_ff + dtau
end

"""
    winch_force_torque!(wfc::WinchForceController, set_length, l_actual, speed,
                         force, drum_radius, gear_ratio, friction, dt) -> torque

Force-mode winch controller: the drum holds a *force*, not a length, so it pays
out whenever the tether pulls harder than the reference. Returns a torque to
apply to the drum.

The reference force is a first-order low-pass of the measured winch force
(`force_tau`), plus a length trim `len_kp * (l_actual - set_length)`, plus
viscous damping `damp * speed`. Damping is not optional: force control gives the
drum no velocity feedback, so trim alone leaves it a free mass on a spring.
Tracking only the *mean* force is what makes the drum yield to everything faster
than `force_tau` while still holding the mean length. Raise `force_tau` for a
softer winch, `len_kp` for tighter length keeping; they trade against each other.
It is initialised to the measured force on the first call, so engaging force mode
from a settled state steps no torque.

Contrast [`winch_position_torque!`](@ref), which holds a length; the two modes
share only the drum and no gains.
"""
function winch_force_torque!(wfc::WinchForceController, set_length, l_actual, speed,
                              force, drum_radius, gear_ratio, friction, dt)
    isnan(wfc.f_lpf) && (wfc.f_lpf = force)
    alpha = dt / (wfc.force_tau + dt)
    wfc.f_lpf += alpha * (force - wfc.f_lpf)
    f_set = wfc.f_lpf + wfc.len_kp * (l_actual - set_length) + wfc.damp * speed
    return force_to_torque(max(f_set, wfc.force_min), drum_radius, gear_ratio,
                           friction)
end
