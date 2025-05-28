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
    time::Vector{Float64} = zeros(Float64, Q)
    "reel-out speed [m/s]"
    v_ro::Vector{Float64} = zeros(Float64, Q)
    "set reel-out speed [m/s]"
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
end
WCLogger(samples::Int64) = WCLogger{samples}()

"""
    WCLogger(duration, dt)

Create and initialize a logger for the winch controller system.

# Arguments
- `duration::Number`: The total duration for which logging should occur. [s]
- `dt::Number`:       The time step interval between log entries. [s]

# Returns
A logger object configured to record data at the specified interval for the given duration.
"""
function WCLogger(duration, dt)
    samples = Int(duration / dt + 1)
    lg = WCLogger(samples)
    lg.time = range(0.0, duration, samples)
    lg
end

function length(logger::WCLogger)
    length(logger.time)
end

"""
    log(logger::WCLogger; v_ro=0.0, v_set_out=0.0, force=0.0, f_err=0.0, acc=0.0, acc_set=0.0)

Logs the current state of the winch controller.

# Arguments
- `logger::WCLogger`: The logger instance used to record the data.
- `v_ro`: (Optional) The measured reel-out velocity. Defaults to `0.0`.
- `v_set_out`: (Optional) The setpoint output velocity. Defaults to `0.0`.
- `force`: (Optional) The measured force. Defaults to `0.0`.
- `f_err`: (Optional) The force error. Defaults to `0.0`.
- `acc`: (Optional) The measured acceleration. Defaults to `0.0`.
- `acc_set`: (Optional) The setpoint acceleration. Defaults to `0.0`.

# Description
This function records the provided parameters to the logger for analysis of the winch controller's performance.
"""
function log(logger::WCLogger; v_ro=0.0, v_set_out=0.0, force=0.0, f_err=0.0, acc=0.0, acc_set=0.0,
             v_err=0.0, reset=0, active=0, f_set=0.0, state=0)
    idx = logger.index
    logger.v_ro[idx] = v_ro
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
    logger.index += 1
end 
