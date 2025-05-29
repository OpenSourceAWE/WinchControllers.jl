module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays, NLsolve, DocStringExtensions, YAML
using LinearAlgebra, Statistics

import Base.reset
import Base.log
import Base.length

export WCSettings, WinchController, Winch, update
export Mixer_2CH, Mixer_3CH, Integrator, UnitDelay, RateLimiter, CalcVSetIn
export set_f_set, set_v_sw, set_reset, set_inactive, set_vset_pc
export set_tracking, set_f_set, get_v_set, get_v_err, calc_vro, get_f_err
export get_speed, set_force, calc_v_set, set_v_set, on_timer, get_acc, get_state, get_status
export select_b, select_c, reset, calc_output
export merge_angles, @limit, saturate
export SpeedController, set_inactive, set_v_act, set_v_set, set_v_set_in, set_tracking, get_v_set_out
export LowerForceController, UpperForceController
export get_startup, get_triangle_wind
export WCLogger, log, f_err, v_err, gamma, damage

abstract type AbstractForceController end
const AFC = AbstractForceController

# Write your package code here.
include("wc_settings.jl")
include("utils.jl")
include("components.jl")
include("wc_components.jl")
include("winchcontroller.jl")
include("logging.jl")

end
