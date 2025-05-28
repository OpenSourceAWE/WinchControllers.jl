"""
    mutable struct WCLogger
        
Struct with the WinchController log vectors.

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
function length(logger::WCLogger)
    logger.index - 1
end
