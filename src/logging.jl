"""
    mutable struct WCLogger{Q}
        
Struct with the WinchController log vectors. Q is the number of 
samples that can be logged. 

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct WCLogger{Q} @deftype Float64
    "Index of the next log entry"
    index::Int64 = 1
    "Vector of time stamps [s]"
    time::Vector{Float64} = zeros(Float64, Q)
    "Maximal winch force [N]"
    max_force::Float64 = 0.0
    "Maximal acceleration [m/s^2]"
    max_acc::Float64 = 0.0
    "damage at maximum acceleration"
    damage_factor::Float64 = 0.0
    "set value of the reel-out speed, input of the speed controller [m/s]"    
    v_set_in::Vector{Float64} = zeros(Float64, Q)
    "reel-out speed [m/s]"
    v_ro::Vector{Float64} = zeros(Float64, Q)
    "set reel-out speed, output of the speed controller [m/s]"
    v_set::Vector{Float64} = zeros(Float64, Q)
    "set reel-out speed of the winch [m/s]"
    v_set_out::Vector{Float64} = zeros(Float64, Q)
    "force [N]"
    force::Vector{Float64} = zeros(Float64, Q)
    "force error [N]"
    f_err::Vector{Float64} = zeros(Float64, Q)
    "acceleration [m/s^2]"
    acc::Vector{Float64} = zeros(Float64, Q)
    "set acceleration [m/s^2]"
    acc_set::Vector{Float64} = zeros(Float64, Q)
    "reel-out speed error [m/s]"
    v_err::Vector{Float64} = zeros(Float64, Q)
    "reset flag"
    reset::Vector{Int64} = zeros(Int64, Q)
    "active flag"
    active::Vector{Int64} = zeros(Int64, Q)
    "set force [N]"
    f_set::Vector{Float64} = zeros(Float64, Q)
    "state of the controller"
    state::Vector{Int64} = zeros(Int64, Q)
    "dynamic, mechanical power [W]"
    p_dyn::Vector{Float64} = zeros(Float64, Q)
end
WCLogger(samples::Int64) = WCLogger{samples}()

"""
    WCLogger(duration, dt, max_force=0.0)

Create and initialize a logger for the winch controller system.

# Arguments
- `duration::Number`: The total duration for which logging should occur. [s]
- `dt::Number`:       The time step interval between log entries. [s]

# Returns
A logger object configured to record data at the specified interval for the given duration.
"""
function WCLogger(duration, dt, max_force=0.0, max_acc=0.0, damage_factor=0.0)
    samples = Int(duration / dt + 1)
    lg = WCLogger(samples)
    lg.time = range(0.0, duration, samples)
    lg.max_force = max_force
    lg.max_acc = max_acc
    lg.damage_factor = damage_factor
    lg
end

function length(logger::WCLogger)
    length(logger.time)
end

"""
    log(logger::WCLogger; v_ro=0.0, v_set=0.0, v_set_in=0.0, v_set_out=0.0, force=0.0, f_err=0.0, 
        acc=0.0, acc_set=0.0, p_dyn=0.0)

Logs the current state of the winch controller.

# Arguments
- `logger::WCLogger`: The logger instance used to record the data.
- `v_ro`: (Optional) The measured reel-out velocity. Defaults to `0.0`.
- `v_set`: (Optional) The input of the speed controller. Defaults to `0.0`.
- `v_set_in`: (Optional) The input of the speed controller. Defaults to `0.0`.
- `v_set_out`: (Optional) The setpoint output velocity. Defaults to `0.0`.
- `force`: (Optional) The measured force. Defaults to `0.0`.
- `f_err`: (Optional) The force error. Defaults to `0.0`.
- `acc`: (Optional) The measured acceleration. Defaults to `0.0`.
- `acc_set`: (Optional) The setpoint acceleration. Defaults to `0.0`.
- `p_dyn`: (Optional) The dynamic mechanical power. Defaults to `0.0`.

# Description
This function records the provided parameters to the logger for analysis of the winch controller's performance.
"""
function log(logger::WCLogger; v_ro=0.0, v_set=0.0, v_set_in=0.0, v_set_out=0.0, force=0.0, f_err=0.0, acc=0.0, acc_set=0.0,
             v_err=0.0, reset=0, active=0, f_set=0.0, state=0, p_dyn=0.0)
    idx = logger.index
    logger.v_ro[idx] = v_ro
    logger.v_set[idx] = v_set
    logger.v_set_in[idx] = v_set_in
    logger.v_set_out[idx] = v_set_out
    logger.force[idx] = force
    logger.f_err[idx] = f_err
    logger.acc[idx] = acc
    logger.acc_set[idx] = acc_set
    logger.v_err[idx] = v_err
    logger.reset[idx] = reset
    logger.active[idx] = active
    logger.f_set[idx] = f_set
    logger.state[idx] = state
    logger.p_dyn[idx] = p_dyn
    logger.index += 1
end


"""
    f_err(logger::WCLogger)

Calculate the normalized maximal force error of a log.

# Arguments
- `logger::WCLogger`: The logger instance used to log error messages.

# Returns
The normalized maximal force error with respect to the maximum force of the winch.
"""
function f_err(logger::WCLogger)
    f_max = logger.max_force
    1/f_max * maximum(norm.(filter(!isnan, logger.f_err)))
end

"""
    v_err(logger::WCLogger)

Calculate the normalized root mean square (RMS) of the reel-out speed error.

# Arguments
- `logger::WCLogger`: The logger object used to record the error message.

# Returns
The normalized RMS of the reel-out speed error with respect to the mean of the set values of the reel-out speed.
"""
function v_err(logger::WCLogger)
    v_mean = mean(norm.(filter(!isnan, logger.v_set)))
    1/v_mean * rms(filter(!isnan, logger.v_err))
end

function damage(logger::WCLogger)
    logger.damage_factor * (maximum(norm.(logger.acc)) / logger.max_acc)^2
end

"""
    gamma(logger::WCLogger)

Compute the combined performance indicator ``\\gamma`` for the test case used to create
the provided logs, stored in the `logger`. See: [Combined performance](@ref).

# Arguments
- `logger::WCLogger`: The logger object that shall be used to calculate gamma.

# Returns
- The gamma value associated with the log of the used test case.
"""
function gamma(logger::WCLogger)
    1 - 0.5 * (f_err(logger) + v_err(logger)) - damage(logger)
end
