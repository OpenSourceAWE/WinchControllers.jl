module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays, NLsolve, DocStringExtensions, YAML
using LinearAlgebra, Statistics, Pkg

import Base.reset
import Base.log
import Base.length

export WCSettings, WinchController, Winch, update
export WinchControllerState, wcsLowerForceLimit, wcsUpperForceLimit, wcsSpeedControl
export Mixer_2CH, Mixer_3CH, Integrator, UnitDelay, RateLimiter, CalcVSetIn
export set_f_set, set_v_sw, set_reset, set_inactive, set_vset_pc
export set_tracking, set_f_set, get_v_set, get_v_set_in, get_v_err, calc_vro, get_f_err
export get_speed, set_force, calc_v_set, set_v_set, on_timer, get_acc, get_state, get_status
export select_b, select_c, reset, calc_output 
export merge_angles, @limit, saturate
export SpeedController, set_inactive, set_v_act, set_v_set, set_v_set_in, set_tracking, get_v_set_out
export LowerForceController, UpperForceController
export get_startup, get_triangle_wind
export WCLogger, log, f_err, v_err, gamma, damage, rms

abstract type AbstractForceController end
const AFC = AbstractForceController

function help(url="https://opensourceawe.github.io/WinchControllers.jl/dev/") 
    if Sys.islinux()
        io = IOBuffer()
        run(pipeline(`xdg-open $url`, stderr = io))
        # ignore any error messages
        out_data = String(take!(io)) 
    else
        DefaultApplication.open(url)
    end
    nothing
end

"""
    copy_examples()

Copy all example scripts to the folder "examples"
(it will be created if it doesn't exist).
"""
function copy_examples()
    PATH = "examples"
    if ! isdir(PATH) 
        mkdir(PATH)
    end
    src_path = joinpath(dirname(pathof(@__MODULE__)), "..", PATH)
    copy_files("examples", readdir(src_path))
end

function copy_model_settings()
    files = ["settings.yaml", "system.yaml", "system_tuned.yaml", "wc_settings.yaml"]
    dst_path = abspath(joinpath(pwd(), "data"))
    copy_files("data", files)
    set_data_path(joinpath(pwd(), "data"))
    println("Copied $(length(files)) files to $(dst_path) !")
end

function install_examples(add_packages=true)
    copy_examples()
    copy_settings()
    copy_model_settings()
    if add_packages
        Pkg.add("KiteUtils")
        Pkg.add("WinchModels")
        Pkg.add("ControlPlots")
        Pkg.add("LaTeXStrings")
        Pkg.add("StatsBase")
        Pkg.add("Timers")
        Pkg.add("NOMAD")
    end
end

function copy_files(relpath, files)
    if ! isdir(relpath) 
        mkdir(relpath)
    end
    src_path = joinpath(dirname(pathof(@__MODULE__)), "..", relpath)
    for file in files
        cp(joinpath(src_path, file), joinpath(relpath, file), force=true)
        chmod(joinpath(relpath, file), 0o774)
    end
    files
end

# Write your package code here.
include("wc_settings.jl")
include("utils.jl")
include("components.jl")
include("wc_components.jl")
include("winchcontroller.jl")
include("logging.jl")

end
