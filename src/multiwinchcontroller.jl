using DiscretePIDs
using TrajectoryLimiters
using Parameters
using Test


@with_kw struct LineLengthControllerSettings
    max_setpoint_firstderivative::Real
    max_setpoint_secondderivative::Real
    sampling_time::Real
end

settings = LineLengthControllerSettings(
    max_setpoint_firstderivative=10.0,
    max_setpoint_secondderivative=50.0,
    sampling_time=0.02
)

limiter = TrajectoryLimiter(settings.sampling_time, settings.max_setpoint_firstderivative, settings.max_setpoint_secondderivative)

linelength_feedback = 0.0
linelength_feedback_firstderivative = 0.0
linelength_order = 1.0

state = TrajectoryLimiters.State(linelength_feedback, linelength_feedback_firstderivative, linelength_order, 0.0)
state, _ = limiter(state, linelength_order)

linelength_setpoint = state.x
linelength_feedback = linelength_setpoint


# Taking directly the setpoint might help to be more reactive
# However due to the delay from the feedback from control lines,
# the differential depower control might be late when reeling or unreeling
# Using directly the feedback from all three lines is a more robust solution which would work
# whatever is the type of controller used to contol the power line (force, speed, length)
# However, again due to the dealy from feedbacks and in the actuation chain it might be late
# in the phase where the reeling speed is changing.
# By chance this should not impact the differential steering control
use_linelength_direct_feedthrough = false
if use_linelength_direct_feedthrough
  reference_linelength = linelength_setpoint
else
  reference_linelength = linelength_feedback



@with_kw struct DifferentialLineLengthControllerSettings
    max_differential_steering_length::Real
    max_depower_length::Real
    sampling_time::Real
end

differential_settings = DifferentialLineLengthControllerSettings(
    max_differential_steering_length=1.0,
    max_depower_length=1.0,
    sampling_time=0.02
)

# Dummy orders (replace with actual signals)
steering_order = 0.2
depower_order = 0.1

# Compute setpoints
length_left_setpoint = reference_linelength + steering_order * differential_settings.max_differential_steering_length / 2 + depower_order * differential_settings.max_depower_length
length_right_setpoint = reference_linelength - steering_order * differential_settings.max_differential_steering_length / 2 + depower_order * differential_settings.max_depower_length

# Dummy feedbacks (replace with real sensor inputs)
length_left_feedback = 0
length_right_feedback = 0
length_power_feedback = 0

# Feedback errors
steering_fdbk = (length_left_feedback - length_right_feedback) / differential_settings.max_differential_steering_length
depower_fdbk = ((length_left_feedback + length_right_feedback) / 2 - length_power_feedback) / differential_settings.max_depower_length

function my_pid()
  ## Parameters of PID controller
  # Proportional gain from standard form (multiplying both proportional, integral and derivative terms)
  proportional_gain = 1
  integral_time = 10
  derivative_time = 1
  sampling_time = 0.02

  antiwindup_reset_time = 1

  #parameter that limits the gain of the derivative term at high frequencies, typically ranges from 2 to 20,
  maximum_derivative_gain = 5

  # parameter that gives the proportion of the reference signal that appears in the proportional term. Default to 1
  # Might be reduced depending on feedforward or direct feedthrough used
  fraction_of_set_point = 1

  # Normalized output
  min_output = -1
  max_output = 1
  initial_integral_state = 0

  # Parameters to initialize derivative filter
  initial_derivative_state = 0
  previous_feedback = 0


  # parallel2standard(Kp, Ki, Kd)

  pid = DiscretePID(; K=proportional_gain, Ti=integral_time,
    Td=derivative_time, Tt=antiwindup_reset_time,
    N=maximum_derivative_gain, b=1fraction_of_set_point,
    umin=min_output, umax=max_output, Ts=sampling_time, I=initial_integral_state, D=initial_derivative_state, yold=previous_feedback)
  return pid
end

# Solution 1: decoupled PIDs
steering_pid = my_pid()
depower_pid = my_pid()

set_torque_left = steering_pid(steering_order, steering_fdbk) + depower_pid(depower_order, depower_fdbk)
set_torque_right = -steering_pid(steering_order, steering_fdbk) + depower_pid(depower_order, depower_fdbk)

# Solution 2: individual line controllers (optional alternative)
controller_linelength_pid = my_pid()
set_torque_left_2 = controller_linelength_pid(length_left_setpoint, length_left_feedback)
set_torque_right_2 = controller_linelength_pid(length_right_setpoint, length_right_feedback)
end