module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays, NLsolve, DocStringExtensions, YAML

import Base.reset

export WCSettings, WinchController, Winch, update
export Mixer_2CH, Mixer_3CH, Integrator, UnitDelay, RateLimiter, CalcVSetIn
export set_f_set, set_v_sw, set_reset, set_inactive, set_vset_pc
export set_tracking, set_f_set, get_v_error, calc_vro, get_f_err
export get_speed, set_force, calc_v_set, set_v_set, on_timer, get_acc, get_state, get_status
export select_b, select_c, reset, calc_output
export merge_angles, @limit
export SpeedController, set_inactive, set_v_act, set_v_set, set_v_set_in, set_tracking, get_v_set_out
export LowerForceController, UpperForceController

abstract type AbstractForceController end
const AFC = AbstractForceController

# Write your package code here.
include("utils.jl")
include("components.jl")
include("wc_settings.jl")
include("wc_components.jl")
include("winchcontroller.jl")

end
