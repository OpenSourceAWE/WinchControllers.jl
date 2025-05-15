module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays, NLsolve

export WCSettings, WinchController
export get_speed, set_force, calc_v_set, set_v_set, on_timer, get_acc, get_state, get_status

abstract type AbstractForceController end
const AFC = AbstractForceController

# Write your package code here.
include("utils.jl")
include("components.jl")
include("wc_settings.jl")
include("wc_components.jl")
include("winchcontroller.jl")

end
