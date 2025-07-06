using DiscretePIDs

struct Positive{T<:Real}
    value::T
    function Positive(x::T) where T<:Real
        if x < 0
            throw(ArgumentError("Value must be positive"))
        end
        new{T}(x)
    end
end

# Allow getting the wrapped value easily
Base.getindex(p::Positive) = p.value
Base.show(io::IO, p::Positive) = print(io, p.value)

using TrajectoryLimiters
using Parameters


@with_kw struct LineLengthControllerSettings
    max_setpoint_firstderivative::Real
    max_setpoint_secondderivative::Real
    sampling_time::Real
end

# Define settings of controller
settings = LineLengthControllerSettings(
    max_setpoint_firstderivative=10.0,
    max_setpoint_secondderivative=50.0,
    sampling_time=0.02
)
limiter = TrajectoryLimiter(settings.sampling_time, settings.max_setpoint_firstderivative, settings.max_setpoint_secondderivative)

# Ensure continuity
linelength_feedback = 0.
linelength_feedback_firstderivative = 0.

linelength_order = 1.
state = TrajectoryLimiters.State(linelength_feedback, linelength_feedback_firstderivative, linelength_order, 0.)

state, x = limiter(state, linelength_order)

linelength_setpoint = state.x


# TODO plug the real feedback (for now assume winch controller allows to track setpoint perfectly)
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

# Define settings of controller
settings = DifferentialLineLengthControllerSettings(
    max_differential_steering_length=1.0,
    max_depower_length=1.0,
    sampling_time=0.02
)


# compute_differential_from_reference_linelength
# We assume left/right to be back lines (attached on the trailing edge, at the back compared to power line with is in front of center of effort)
length_left_setpoint = reference_linelength + steering_order * settings.max_differential_steering_length / 2 + depower_order * settings.max_depower_length
length_right_setpoint = reference_linelength + steering_order * settings.max_differential_steering_length / 2 + depower_order * settings.max_depower_length

# Compute feedback from measurements
steering_fdbk = (length_left_feedback - length_right_feedback) / settings.max_differential_steering_length
depower_fdbk = ((length_left_feedback + length_right_feedback) / 2 - length_power_feedback) / settings.max_depower_length

# Solution 1: use decoupled pid for steering and depower
steering_pid = my_pid()
depower_pid = my_pid()
set_torque_left = steering_pid(steering_fdbk - steering_order) + depower_pid(depower_fdbk - depower_order)
set_torque_right = steering_pid(steering_fdbk - steering_order) + depower_pid(depower_fdbk - depower_order)

# Solution 2: use two identifcal line length pid controller
controllerlinelength_pid = my_pid()
set_torque_left = controllinelength_pid(length_left_setpoint, length_left_feedback)
set_torque_right = controllinelength_pid(length_right_setpoint, length_right_feedback)

function my_pid()
  ## Parameters of PID controller
  # Proportional gain from standard form (multiplying both proportional, integral and derivative terms)
  proportional_gain = 1
  integral_time = 10
  derivative_time = 1

  antiwindup_reset_time = 1

  #parameter that limits the gain of the derivative term at high frequencies, typically ranges from 2 to 20,
  maximum_derivative_gain = 5

  # parameter that gives the proportion of the reference signal that appears in the proportional term. Default to 1
  # Might be reduced depending on feedforward or direct feedthrough used
  fraction_of_set_point = 1

  # Normalized output
  umin = -1
  umax = 1
  initial_integral_state = 0

  # Parameters to initialize derivative filter
  initial_derivative_state = 0
  previous_feedback = feedback


  parallel2standard(Kp, Ki, Kd)

  pid = DiscretePID(; K=proportional_gain, Ti=integral_time,
    Td=derivative_time, Tt=antiwindup_reset_time,
    N=maximum_derivative_gain, b=1fraction_of_set_point,
    umin=-min_output, umax=max_output, Ts=sampling_time, I=initial_integral_state, D=initial_derivative_stat, yold=previous_feedback)
  return pid
end
# u = pid(reference, feedback, uff)

# reset_state!(pid)

# ctrl = function (x, t)
#   y = (P.C*x)[]
#   r = 1
#   u = pid(r, y)
# end

# P = c2d(ss(tf(1, [1, 1])), sampling_time)

# ## PI control
# Ti = 1
# C = c2d(ControlSystemsBase.pid(K, Ti), Ts)
# res = step(feedback(P * C), 3)
# res2 = lsim(P, ctrl, 3)

@testitem "First tests" begin
    x = 1
    @test x =1
end