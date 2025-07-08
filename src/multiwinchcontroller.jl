using TrajectoryLimiters
using Parameters
using DiscretePIDs
using Test

# ----------------------------------------
# Reference Line Length Control Settings
# ----------------------------------------
@with_kw struct ReferenceLineLengthControllerSettings
    max_setpoint_speed::Real          # Maximum reel in/out speed allowed [m/s]
    max_setpoint_acceleration::Real   # Max variation of reel speed [m/s²]
    sampling_time::Real               # Sampling time for discrete controller [s]
end

function create_trajectory_limiter(settings::ReferenceLineLengthControllerSettings)
    return TrajectoryLimiter(settings.sampling_time, settings.max_setpoint_speed, settings.max_setpoint_acceleration)
end

function update_line_length_feedback(
    limiter::TrajectoryLimiter, 
    feedback::Real, feedback_speed::Real, 
    order::Real
)
    state = TrajectoryLimiters.State(feedback, feedback_speed, order, 0.0)
    state, _ = limiter(state, order)
    return state.x  # New line length setpoint
end

# ----------------------------------------
# Differential Line Length Control Settings
# ----------------------------------------
@with_kw struct DifferentialLineLengthControllerSettings
    max_differential_steering_length::Real  # [m]
    max_depower_length::Real                # [m]
    sampling_time::Real
end

# ----------------------------------------
# PID Factory Function
# ----------------------------------------
function make_pid(; 
    # Proportional gain from standard form (multiplying both P, I and D terms)
    Kp=1.0,

    # Integral time constant: how fast the integral action builds up
    Ti=10.0,

    # Derivative time constant: how strongly it reacts to rate of change
    Td=1.0,

    # Sampling time of the discrete controller [s]
    Ts=0.02,

    # Antiwindup reset time (Tt): determines how fast integral antiwindup reacts
    Tt=1.0,

    # Parameter that limits the gain of the derivative term at high frequencies (2–20 typical)
    N=5.0,

    # Proportion of the reference signal in the proportional term (usually 1.0; <1 if feedforward used)
    b=1.0,

    # Output limits (normalized)
    umin=-1.0, 
    umax=1.0,

    # Initial state of integral term
    I=0.0,

    # Initial state of derivative filter
    D=0.0,

    # Previous feedback value, needed for derivative filter init
    yold=0.0
)
    return DiscretePID(
        K=Kp, Ti=Ti, Td=Td, Ts=Ts, Tt=Tt,
        N=N, b=b, umin=umin, umax=umax,
        I=I, D=D, yold=yold
    )
end

# ----------------------------------------
# Control Logic
# ----------------------------------------
function compute_control(
    ref_linelength_setpoint::Real, 
    steering_order::Real, depower_order::Real,
    length_left_feedback::Real, length_right_feedback::Real, length_power_feedback::Real,
    settings::DifferentialLineLengthControllerSettings
)
    # Compute setpoints for left/right line lengths
    left_sp = ref_linelength_setpoint + steering_order * settings.max_differential_steering_length / 2 + depower_order * settings.max_depower_length
    right_sp = ref_linelength_setpoint - steering_order * settings.max_differential_steering_length / 2 + depower_order * settings.max_depower_length

    # Compute normalized feedback errors
    steering_fdbk = (length_left_feedback - length_right_feedback) / settings.max_differential_steering_length
    depower_fdbk = ((length_left_feedback + length_right_feedback)/2 - length_power_feedback) / settings.max_depower_length

    # Create PIDs
    steering_pid = make_pid(Ts=settings.sampling_time)
    depower_pid = make_pid(Ts=settings.sampling_time)

    # Control torques (Solution 1: decoupled)
    torque_left = steering_pid(steering_order, steering_fdbk) + depower_pid(depower_order, depower_fdbk)
    torque_right = -steering_pid(steering_order, steering_fdbk) + depower_pid(depower_order, depower_fdbk)

    # Control torques (Solution 2: individual line PIDs)
    controller_pid = make_pid(Ts=settings.sampling_time)
    torque_left_2 = controller_pid(left_sp, length_left_feedback)
    torque_right_2 = controller_pid(right_sp, length_right_feedback)

    return torque_left, torque_right, torque_left_2, torque_right_2
end

# ----------------------------------------
# Example Usage
# ----------------------------------------

# Settings
ref_settings = ReferenceLineLengthControllerSettings(
    max_setpoint_speed = 10.0,
    max_setpoint_acceleration = 50.0,
    sampling_time = 0.02
)

differential_settings = DifferentialLineLengthControllerSettings(
    max_differential_steering_length = 1.0,
    max_depower_length = 1.0,
    sampling_time = 0.02
)

# Initial feedback and order values
linelength_feedback = 0.0
linelength_feedback_speed = 0.0
linelength_order = 1.0

# Trajectory limiting
limiter = create_trajectory_limiter(ref_settings)
linelength_setpoint = update_line_length_feedback(limiter, linelength_feedback, linelength_feedback_speed, linelength_order)

# Taking directly the setpoint might help to be more reactive
# However due to the delay from the feedback from control lines,
# the differential depower control might be late when reeling or unreeling
# Using directly the feedback from all three lines is a more robust solution which would work
# whatever is the type of controller used to contol the power line (force, speed, length)
# However, again due to the dealy from feedbacks and in the actuation chain it might be late
# in the phase where the reeling speed is changing.
# By chance this should not impact the differential steering control
use_linelength_direct_feedthrough = false
reference_linelength = use_linelength_direct_feedthrough ? linelength_setpoint : linelength_feedback


  # ud_prime: depower setting in the range of 0 to 1, 0 means fully powered, 1 means fully depowered

  # TODO: define if left/right is in kitesurfer (upwind in the back) or paraglider convention (upwind in the front)

  # Left/right symmetry is assumed
  # Lines are assumed of the same length when steering and depower are zero.
  # This corresponds to the convention already used @TU Delft
  # Note this is not the usual convention for kitesurfers which defines that lines are equal when kitebar is in full power position
  # No trim is considered for now
  
# Dummy control orders and feedbacks
steering_order = 0.2
depower_order = 0.1
length_left_feedback = 0.0
length_right_feedback = 0.0
length_power_feedback = 0.0

# Compute control torques
torque_L1, torque_R1, torque_L2, torque_R2 = compute_control(
    reference_linelength,
    steering_order, depower_order,
    length_left_feedback, length_right_feedback, length_power_feedback,
    differential_settings
)

println("Torque Left (decoupled): ", torque_L1)
println("Torque Right (decoupled): ", torque_R1)
println("Torque Left (individual): ", torque_L2)
println("Torque Right (individual): ", torque_R2)
