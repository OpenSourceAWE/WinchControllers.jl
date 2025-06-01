
"""
    mutable struct FFWinchController

Feedforward winch controller that combines feedforward force and speed control.

# Fields
$(TYPEDFIELDS)
"""
@with_kw mutable struct FFWinchController <: AbstractWinchController @deftype Float64
    wcs::WCSettings
    set::Settings
    fc::FeedforwardForceController = FeedforwardForceController(; set, wcs) # Initialize with default settings
    sc::FeedforwardSpeedController = FeedforwardSpeedController(; set, wcs) # Initialize with default settings
    traj_lim::TrajectoryLimiters.TrajectoryLimiter = TrajectoryLimiters.TrajectoryLimiter(wcs.dt, 0.01, 0.01)
    traj_state::TrajectoryLimiters.State = TrajectoryLimiters.State(0.0, 0.0, 0.0, 0.0)
    rate_lim::RateLimiter = RateLimiter(wcs.dt, 1000.0, 0.0) # Rate limiter
    state::WinchControllerState = wcsSpeedControl
    t = 0.0
    v_set = 0.0
    F̂::Float64 = 0.0
    ω̂ = 0.0
    α̂ = 0.0
    τ_ff = 0.0
    τ_set = 0.0
    τ_last = 0.0
    ∫e = 0.0
    last_switch = -Inf
end

function FFWinchController(wcs::WCSettings, set::Settings)
    return FFWinchController(; wcs, set)
end

"""
    calculate_torque(wc::FFWinchController, v_set::Float64, F̂::Union{Float64, Nothing}, τ̂::Float64, ω̂::Float64, α̂::Float64)

Calculates the feedforward torque for the winch based on desired speed, measured force (optional),
actual motor torque, current angular velocity, and current angular acceleration.

## Parameters
- `wc::FFWinchController`: The feedforward winch controller instance.
- `v_set::Float64`: The target tether speed [m/s].
- `F̂::Union{Float64, Nothing}`: Measured tether force [N]. If `nothing`, force will be estimated.
- `ω̂::Float64`: The current angular velocity of the winch drum [rad/s].
- `α̂::Float64`: The current angular acceleration of the winch drum [rad/s²].

## Returns
- `Float64`: The calculated feedforward torque [N·m].
"""
function calc_τ_set(wc::FFWinchController, ω̂, α̂, F̂=nothing)
    if isnothing(F̂)
        F̂ = calc_F_est(wc, ω̂, α̂, wc.τ_last)
    end
    wcs = wc.wcs
    wc.F̂ = F̂
    wc.ω̂ = ω̂
    wc.α̂ = α̂
    
    v_set = calc_vro(wcs, abs(F̂))
    wc.v_set = v_set
    ω_set = v_set / wc.set.drum_radius
    
    # α_set = limit_acceleration(wc, ω_set)

    if wc.t - wc.last_switch > 0.1
        if wc.state == wcsSpeedControl && wc.F̂ ≤ wcs.f_low
            wc.state = wcsLowerForceLimit
            wc.last_switch = wc.t
        elseif wc.state == wcsSpeedControl && wc.F̂ ≥ wcs.f_high # || wc.state == wcsUpperForceLimit
            wc.state = wcsUpperForceLimit
            wc.last_switch = wc.t
        elseif wc.state == wcsLowerForceLimit && ω_set < ω̂
            wc.state = wcsSpeedControl
            wc.last_switch = wc.t
        elseif wc.state == wcsUpperForceLimit && ω_set > ω̂
            wc.state = wcsSpeedControl
            wc.last_switch = wc.t
        end
    end
    # wc.state = wcsSpeedControl
    
    if wc.state == wcsLowerForceLimit
        wc.τ_ff = calc_τ_ff(wc.fc, wcs.f_low, ω̂, α̂, F̂)
        v_set = NaN
    elseif wc.state == wcsUpperForceLimit
        wc.τ_ff = calc_τ_ff(wc.fc, wcs.f_high, ω̂, α̂, F̂)
        v_set = NaN
    elseif wc.state == wcsSpeedControl
        wc.τ_ff = calc_τ_ff(wc.sc, v_set, ω̂, α̂, F̂)
    end
    # _, d_τ_ff = wc.limiter(wc.state, wc.τ_ff)
    # wc.τ_set = calc_output(wc.rate_lim, wc.τ_ff)
    wc.τ_set = wc.τ_ff
    wc.τ_last = wc.τ_set
    return wc.τ_set, v_set
end

"""
    on_timer(wc::FFWinchController)

Callback function that is triggered periodically by a timer event. This function is responsible for handling time-based 
updates or actions for the given `FFWinchController` instance `wc`.

## Arguments
- `wc::FFWinchController`: The winch controller instance to be updated.

## Returns
- Nothing. This function is called for its side effects.
"""
function on_timer(wc::FFWinchController)
    wc.t += wc.wcs.dt
    on_timer(wc.rate_lim)
    nothing
end

"""
    get_status(wc::FFWinchController)

Retrieve the current status of the given `FFWinchController` instance for logging and debugging purposes.

## Arguments
- `wc::FFWinchController`: The winch controller object whose status is to be retrieved.

## Returns
- The current status of the winch controller, an array containing:
    - `reset`: A boolean indicating whether a reset has occurred (using a dummy value for now).
    - `active`: A boolean indicating whether the controller is active (always true for now).
    - `force`: The measured force [N] or 0.0 if nothing.
    - `f_set`: The set force (using a dummy value for now).
    - `v_set_out`: The output set speed [m/s].
    - `v_set_out_lfc`: The output set speed from a lower force controller (using a dummy value for now).
    - `v_set_out_ufc`: The output set speed from an upper force controller (using a dummy value for now).
"""
function get_status(wc::FFWinchController)
    F̂ = wc.F̂
    if isnothing(F̂)
        F̂ = 0.0
    end
    result = [false, true, F̂, 0.0, wc.v_set, 0.0, 0.0]
    return result
end

function get_state(wc::FFWinchController)
    return Int(wc.state)
end

function get_v_err(wc::FFWinchController)
    return NaN
end

function get_f_err(wc::FFWinchController)
    return NaN
end

function get_v_set(wc::FFWinchController)
    return NaN
end

function get_v_set_in(wc::FFWinchController)
    return NaN
end

