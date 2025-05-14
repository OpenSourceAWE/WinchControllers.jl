module WinchControllers

using Parameters, StructTypes, KiteUtils, WinchModels, StaticArrays

export WCSettings

abstract type AbstractForceController end
const AFC = AbstractForceController

# Write your package code here.
include("utils.jl")
include("components.jl")
include("wc_settings.jl")
include("wc_components.jl")

end
