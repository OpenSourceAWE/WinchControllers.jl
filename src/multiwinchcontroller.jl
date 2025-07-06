using DiscretePIDs



using TrajectoryLimiters

ẍM = 50                       # Maximum acceleration
ẋM = 10                       # Maximum velocity
Ts = 0.005                    # Sample time
r(t) = 2.5 + 3 * (t - floor(t)) # Reference to be smoothed
t = 0:Ts:3                   # Time vector
R = r.(t)                    # An array of sampled position references 

limiter = TrajectoryLimiter(Ts, ẋM, ẍM)
state, ẍ = limiter(state, r(t))

set_torque_P = set_torque


# compute_differential_from_reference_linelength
length_left_order = reference_linelength + steering_order * max_differential_steering_length / 2 + depower_order * max_depower_length
length_right_order = reference_linelength + steering_order * max_differential_steering_length / 2 + depower_order * max_depower_length

# compute feedback from measurements
steering_fdbk = (length_left_fdbk - length_right_fdbk) / max_differential_steering_length
depower_fdbk = ((length_left + length_right) / 2 - length_power) / max_depower_length


set_torque_left = pid(steering_fdbk - steering_order) + pid(depower_fdbk - depower_order)
set_torque_right = pid(steering_fdbk - steering_order) + pid(depower_fdbk - depower_order)


## Parameters of PID controller
sampling_time = 0.02

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

u = pid(reference, feedback, uff)

reset_state!(pid)

ctrl = function (x, t)
  y = (P.C*x)[]
  r = 1
  u = pid(r, y)
end

P = c2d(ss(tf(1, [1, 1])), sampling_time)

## PI control
Ti = 1
C = c2d(ControlSystemsBase.pid(K, Ti), Ts)
res = step(feedback(P * C), 3)
res2 = lsim(P, ctrl, 3)