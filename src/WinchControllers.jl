module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays, NLsolve, DocStringExtensions

export WCSettings, WinchController, Mixer_2CH, Mixer_3CH, Integrator
export get_speed, set_force, calc_v_set, set_v_set, on_timer, get_acc, get_state, get_status
export select_b, select_c

abstract type AbstractForceController end
const AFC = AbstractForceController

# Write your package code here.
include("utils.jl")
include("components.jl")
include("wc_settings.jl")
include("wc_components.jl")
include("winchcontroller.jl")

end
