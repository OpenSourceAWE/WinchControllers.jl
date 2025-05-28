"""
    mutable struct WCLogger
        
Struct with the WinchController log vectors.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct WCLogger @deftype Float64
    "reel-out speed [m/s]"
    v_ro::Vector{Float64} = Float64[]
    "set reel-out speed [m/s]"
    v_set_out::Vector{Float64} = Float64[]
    "force [N]"
    force::Vector{Float64} = Float64[]
    "force error [N]"
    f_err::Vector{Float64} = Float64[]
    "acceleration [m/s^2]"
    acc::Vector{Float64} = Float64[]
    "set acceleration [m/s^2]"
    acc_set::Vector{Float64} = Float64[]
    "reel-out speed error [m/s]"
    v_err::Vector{Float64} = Float64[]
    "reset flag"
    reset::Vector{Int64} = Int64[]
    "active flag"
    active::Vector{Int64} = Int64[]
    "set force [N]"
    f_set::Vector{Float64} = Float64[]
    "state of the controller"
    state::Vector{Int64} = Int64[]
end
function reset_logging!(log::WCLogger)
    log.v_ro = Float64[]
    log.v_set_out = Float64[]
    log.force = Float64[]
    log.f_err = Float64[]
    log.acc = Float64[]
    log.acc_set = Float64[]
    log.v_err = Float64[]
    log.reset = Int64[]
    log.active = Int64[]
    log.f_set = Float64[]
    log.state = Int64[]
end
function push_logging!(log::WCLogger, v_ro, v_set_out, force, f_err, acc, acc_set, v_err, reset, active, f_set, state)
    push!(log.v_ro, v_ro)
    push!(log.v_set_out, v_set_out)
    push!(log.force, force)
    push!(log.f_err, f_err)
    push!(log.acc, acc)
    push!(log.acc_set, acc_set)
    push!(log.v_err, v_err)
    push!(log.reset, reset)
    push!(log.active, active)
    push!(log.f_set, f_set)
    push!(log.state, state)
end