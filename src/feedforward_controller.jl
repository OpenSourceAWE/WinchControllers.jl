
"""
    struct FeedforwardForceController

Component to calculate the feedforward input torque for a desired tether force.
Assumes a torque-controlled winch. The torque is calculated based on the formula:
\$\\tau = F_{desired} \\cdot r + b \\cdot \\ω̂ + I_{eff} \\cdot \alpha\$

Where:
- \$F_{desired}\$ is the desired tether force.
- \$r\$ is the drum radius.
- \$b\$ is the viscous friction coefficient at the drum.
- \$\\ω̂\$ is the current angular velocity of the drum.
- \$I_{eff}\$ is the effective moment of inertia at the drum (\$I_{motor\\_{side}} \\cdot gear\\_{ratio}^2\$).
- \$\alpha\$ is the current angular acceleration of the drum.

## Fields
$(TYPEDFIELDS)
"""
@with_kw struct FeedforwardForceController
    """General settings containing physical parameters of the winch."""
    set::Settings
end

function calc_coulomb_friction(set::Settings)
    return set.f_coulomb * set.drum_radius / set.gear_ratio
end

function calc_viscous_friction(set::Settings, ω̂_m)
    set.c_vf * ω̂_m * set.drum_radius^2 / set.gear_ratio^2     
end

function calc_inertia(set::Settings)
    set.inertia_total
end

function calc_τ_friction(set::Settings, ω̂)
    ω̂_m = set.gear_ratio * ω̂
    return calc_coulomb_friction(set) * WinchModels.smooth_sign(ω̂_m) + calc_viscous_friction(set, ω̂_m)
end

function calc_τ_force(set::Settings, F)
    return (F / set.gear_ratio) * set.drum_radius
end

function calc_τ_I(set::Settings, α̂)
    α̂_m = set.gear_ratio * α̂
    return set.inertia_total * α̂_m
end

"""
    calc_ff_τ(fc::FeedforwardForceController, desired_force::Float64, ω̂::Float64, α̂::Float64)

Calculates the feedforward torque required to achieve a `desired_force`.

## Parameters
- `fc::FeedforwardForceController`: The feedforward force controller instance.
- `desired_force::Float64`: The target tether force [N].
- `ω̂::Float64`: The current angular velocity of the winch drum [rad/s].
- `α̂::Float64`: The current angular acceleration of the winch drum [rad/s²].

## Returns
- `Float64`: The calculated feedforward torque [N·m].
"""
function calc_ff_τ(fc::FeedforwardForceController, F_set, ω̂, α̂)
    set = fc.set
    I = calc_inertia(set)
    
    τ_force = calc_τ_force(set, F_set)
    τ = τ_force + calc_τ_friction(set, ω̂) + calc_τ_I(set, α̂)
    return τ
end

"""
    struct FeedforwardSpeedController

Component to calculate the feedforward input torque for a desired tether speed.
Assumes a torque-controlled winch. The torque is calculated based on the formula:
\$\\tau = F_{eff} \\cdot r + b \\cdot (v_{desired}/r) + I_{eff} \\cdot α\$

Where:
- \$F_{eff}\$ is the effective tether force (either measured or estimated).
- \$r\$ is the drum radius.
- \$b\$ is the viscous friction coefficient at the drum.
- \$v_{desired}\$ is the desired tether speed.
- \$I_{eff}\$ is the effective moment of inertia at the drum.
- \$\alpha\$ is the current angular acceleration of the drum.

If the tether force is not measured, it can be estimated using:
\$F_{est} = (\\tau_{actual\\_motor} - b \\cdot \\ω̂ - I_{eff} \\cdot α) / r\$

## Fields
$(TYPEDFIELDS)
"""
@with_kw mutable struct FeedforwardSpeedController
    """General settings containing physical parameters of the winch."""
    set::Settings
    τ_last::Float64 = 0.0
end

"""
    calc_F_est(sc::FeedforwardSpeedController, ω̂::Float64, α̂::Float64, τ̂::Float64)

Estimates the tether force based on the actual motor torque, angular velocity, and angular acceleration.
Formula: \$F_{est} = (\\tau_{actual\\_motor} - b \\cdot \\ω̂ - I_{eff} \\cdot α) / r\$

## Parameters
- `sc::FeedforwardSpeedController`: The feedforward speed controller instance.
- `ω̂::Float64`: Current angular velocity of the winch drum [rad/s].
- `α̂::Float64`: Current angular acceleration of the winch drum [rad/s²].
- `τ̂::Float64`: The actual torque being applied by the motor [N·m].

## Returns
- `Float64`: The estimated tether force [N].
"""
function calc_F_est(sc::FeedforwardSpeedController, ω̂::Float64, α̂::Float64, τ̂::Float64)
    set = sc.set
    r = set.drum_radius
    I = calc_inertia(set)
    τ_friction = calc_τ_friction(set, ω̂)

    estimated_force = (-τ̂ + τ_friction + I * α̂) * set.gear_ratio / r
    return estimated_force
end


"""
    α_m * I = τ_set + drum_radius / gear_ratio * F  - τ_friction(ω)

    F = (α_m * I - τ_set + τ_friction(ω)) * gear_ratio / drum_radius

    τ_set = α_m * I - drum_radius / gear_ratio * F + τ_friction(ω)
"""

"""
    calc_ff_τ(sc::FeedforwardSpeedController, v_set::Float64, ω̂::Float64, α̂::Float64; F̂::Union{Float64, Nothing}=nothing)

Calculates the feedforward torque required to achieve a `v_set`.

## Parameters
- `sc::FeedforwardSpeedController`: The feedforward speed controller instance.
- `v_set::Float64`: The target tether speed [m/s].
- `α̂::Float64`: The current angular acceleration of the winch drum [rad/s²].
- `F̂::Union{Float64, Nothing}` (optional): Measured tether force [N]. If `nothing`, force will be estimated.
  The actual torque being applied by the motor [N·m], used for force estimation.

## Returns
- `Float64`: The calculated feedforward torque [N·m].
"""
function calc_ff_τ(sc::FeedforwardSpeedController, v_set, ω̂, α̂, F̂=nothing)
    set = sc.set
    r = set.drum_radius
    I = calc_inertia(set)
    
    F_eff = 0.0
    if !isnothing(F̂)
        F_eff = F̂
    else
        F_eff = calc_F_est(sc, ω̂, α̂, sc.τ_last)
    end

    # Desired angular velocity at the drum
    ω_set = v_set / r
    τ = -calc_τ_force(set, F_eff) + calc_τ_friction(set, ω_set) + calc_τ_I(set, α̂)

    sc.τ_last = τ
    return τ
end

"""
    mutable struct FFWinchController

Feedforward winch controller that combines feedforward force and speed control.

# Fields
$(TYPEDFIELDS)
"""
@with_kw mutable struct FFWinchController @deftype Float64
    wcs::WCSettings
    set::Settings
    fc::FeedforwardForceController = FeedforwardForceController(; set) # Initialize with default settings
    sc::FeedforwardSpeedController = FeedforwardSpeedController(; set) # Initialize with default settings
    limiter::TrajectoryLimiters.TrajectoryLimiter = TrajectoryLimiters.TrajectoryLimiter(wcs.dt, set.max_acc, Inf)
    state::TrajectoryLimiters.State = TrajectoryLimiters.State(0.0, 0.0, 0.0, 0.0)
    t = 0.0
    v_set = 0.0
    F̂::Union{Float64, Nothing} = nothing
    ω̂ = 0.0
    α̂ = 0.0
    τ_ff = 0.0
    τ_set = 0.0
    ∫e = 0.0
end

function FFWinchController(wcs::WCSettings, set::Settings)
    return FFWinchController(; wcs, set)
end

"""
    limit_acceleration(wc::FFWinchController, ω_set::Float64, dt::Float64)

Calculates the limited angular acceleration based on the desired angular velocity and the rate limiter.

## Parameters
- `wc::FFWinchController`: The feedforward winch controller instance.
- `ω_set::Float64`: The target angular velocity [rad/s].
- `dt::Float64`: The time step [s].

## Returns
- `Float64`: The limited angular acceleration [rad/s²].
"""
function limit_acceleration(wc::FFWinchController, ω_set::Float64)
    α_lim = wc.limiter(wc.state, ω_set)
    return α_lim
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
function calc_τ_set(wc::FFWinchController, v_set, ω̂, α̂, F̂=nothing)
    wc.v_set = v_set
    wc.F̂ = F̂
    wc.ω̂ = ω̂
    wc.α̂ = α̂

    ω_set = v_set / wc.set.drum_radius
    α_set = limit_acceleration(wc, ω_set)
    wc.τ_ff = calc_ff_τ(
        wc.sc,
        v_set,
        ω̂,
        α̂,
        F̂
    )
    return wc.τ_ff
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
    return 1
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

