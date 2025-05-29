
"""
    struct FeedforwardForceController

Component to calculate the feedforward input torque for a desired tether force.
Assumes a torque-controlled winch. The torque is calculated based on the formula:
\$\\tau = F_{desired} \\cdot r + b \\cdot \\omega_{current} + I_{eff} \\cdot \alpha_{current}\$

Where:
- \$F_{desired}\$ is the desired tether force.
- \$r\$ is the drum radius.
- \$b\$ is the viscous friction coefficient at the drum.
- \$\\omega_{current}\$ is the current angular velocity of the drum.
- \$I_{eff}\$ is the effective moment of inertia at the drum (\$I_{motor\\_{side}} \\cdot gear\\_{ratio}^2\$).
- \$\alpha_{current}\$ is the current angular acceleration of the drum.

## Fields
$(TYPEDFIELDS)
"""
@with_kw struct FeedforwardForceController
    """General settings containing physical parameters of the winch."""
    set::Settings
end

"""
    calculate_feedforward_torque(fffc::FeedforwardForceController, desired_force::Float64, current_omega::Float64, current_alpha::Float64)

Calculates the feedforward torque required to achieve a `desired_force`.

## Parameters
- `fffc::FeedforwardForceController`: The feedforward force controller instance.
- `desired_force::Float64`: The target tether force [N].
- `current_omega::Float64`: The current angular velocity of the winch drum [rad/s].
- `current_alpha::Float64`: The current angular acceleration of the winch drum [rad/s²].

## Returns
- `Float64`: The calculated feedforward torque [N·m].
"""
function calculate_feedforward_torque(fffc::FeedforwardForceController, desired_force::Float64, current_omega::Float64, current_alpha::Float64)
    r = fffc.set.drum_radius
    # Effective inertia at the drum side
    # Assuming set.inertia_total is the inertia on the motor side before the gearbox
    eff_inertia = fffc.set.inertia_total * (fffc.set.gear_ratio^2)
    b = fffc.set.b_friction # Viscous friction coefficient at the drum

    # Torque due to desired force
    torque_force = desired_force * r
    # Torque due to viscous friction
    torque_friction = b * current_omega
    # Torque due to inertia
    torque_inertia = eff_inertia * current_alpha

    # Total feedforward torque
    total_torque = torque_force + torque_friction + torque_inertia
    return total_torque
end

"""
    struct FeedforwardSpeedController

Component to calculate the feedforward input torque for a desired tether speed.
Assumes a torque-controlled winch. The torque is calculated based on the formula:
\$\\tau = F_{eff} \\cdot r + b \\cdot (v_{desired}/r) + I_{eff} \\cdot \\alpha_{current}\$

Where:
- \$F_{eff}\$ is the effective tether force (either measured or estimated).
- \$r\$ is the drum radius.
- \$b\$ is the viscous friction coefficient at the drum.
- \$v_{desired}\$ is the desired tether speed.
- \$I_{eff}\$ is the effective moment of inertia at the drum.
- \$\alpha_{current}\$ is the current angular acceleration of the drum.

If the tether force is not measured, it can be estimated using:
\$F_{est} = (\\tau_{actual\\_motor} - b \\cdot \\omega_{current} - I_{eff} \\cdot \\alpha_{current}) / r\$

## Fields
$(TYPEDFIELDS)
"""
@with_kw struct FeedforwardSpeedController
    """General settings containing physical parameters of the winch."""
    set::Settings
end

"""
    estimate_tether_force(ffsc::FeedforwardSpeedController, current_omega::Float64, current_alpha::Float64, actual_motor_torque::Float64)

Estimates the tether force based on the actual motor torque, angular velocity, and angular acceleration.
Formula: \$F_{est} = (\\tau_{actual\\_motor} - b \\cdot \\omega_{current} - I_{eff} \\cdot \\alpha_{current}) / r\$

## Parameters
- `ffsc::FeedforwardSpeedController`: The feedforward speed controller instance.
- `current_omega::Float64`: Current angular velocity of the winch drum [rad/s].
- `current_alpha::Float64`: Current angular acceleration of the winch drum [rad/s²].
- `actual_motor_torque::Float64`: The actual torque being applied by the motor [N·m].

## Returns
- `Float64`: The estimated tether force [N].
"""
function estimate_tether_force(ffsc::FeedforwardSpeedController, current_omega::Float64, current_alpha::Float64, actual_motor_torque::Float64)
    r = ffsc.set.drum_radius
    eff_inertia = ffsc.set.inertia_total * (ffsc.set.gear_ratio^2)
    b = ffsc.set.b_friction

    # Ensure drum radius is not zero to avoid division by zero
    if r == 0.0
        # Or handle error appropriately, e.g., throw ArgumentError
        return 0.0 
    end

    estimated_force = (actual_motor_torque - b * current_omega - eff_inertia * current_alpha) / r
    return estimated_force
end

"""
    calculate_feedforward_torque(ffsc::FeedforwardSpeedController, desired_speed::Float64, current_omega::Float64, current_alpha::Float64; measured_force::Union{Float64, Nothing}=nothing, actual_motor_torque::Float64=0.0)

Calculates the feedforward torque required to achieve a `desired_speed`.

## Parameters
- `ffsc::FeedforwardSpeedController`: The feedforward speed controller instance.
- `desired_speed::Float64`: The target tether speed [m/s].
- `current_omega::Float64`: The current angular velocity of the winch drum [rad/s].
- `current_alpha::Float64`: The current angular acceleration of the winch drum [rad/s²].
- `measured_force::Union{Float64, Nothing}` (optional): Measured tether force [N]. If `nothing`, force will be estimated.
- `actual_motor_torque::Float64` (optional, required if `measured_force` is `nothing`): 
  The actual torque being applied by the motor [N·m], used for force estimation.

## Returns
- `Float64`: The calculated feedforward torque [N·m].
"""
function calculate_feedforward_torque(ffsc::FeedforwardSpeedController, desired_speed::Float64, current_omega::Float64, current_alpha::Float64; measured_force::Union{Float64, Nothing}=nothing, actual_motor_torque::Float64=0.0)
    r = ffsc.set.drum_radius
    eff_inertia = ffsc.set.inertia_total * (ffsc.set.gear_ratio^2)
    b = ffsc.set.b_friction

    # Ensure drum radius is not zero for calculations involving 1/r
    if r == 0.0
        # Or handle error appropriately
        return 0.0 
    end

    F_eff::Float64 = 0.0
    if !isnothing(measured_force)
        F_eff = measured_force
    else
        if actual_motor_torque == 0.0 && desired_speed != 0.0 # Warn if trying to estimate force with no motor torque info for active control
            @warn "actual_motor_torque is 0.0 for force estimation in FeedforwardSpeedController. Estimated force might be inaccurate."
        end
        F_eff = estimate_tether_force(ffsc, current_omega, current_alpha, actual_motor_torque)
    end

    # Desired angular velocity at the drum
    omega_desired = desired_speed / r

    # Torque due to effective force
    torque_force = F_eff * r
    # Torque due to viscous friction at desired speed
    torque_friction = b * omega_desired 
    # Torque due to current inertia
    torque_inertia = eff_inertia * current_alpha

    # Total feedforward torque
    total_torque = torque_force + torque_friction + torque_inertia
    return total_torque
end

