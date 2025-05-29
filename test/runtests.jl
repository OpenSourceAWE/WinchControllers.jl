using WinchControllers, KiteUtils
using Test

const __TEST__ = true

cd("..")
KiteUtils.set_data_path("data") 

@testset verbose=true "WinchControllers.jl " begin

@testset verbose=true "components.jl " begin
    include("test_components.jl")
end

@testset verbose=true "utils.jl            " begin
    include("test_utils.jl")
end

@testset verbose=true "winchcontrollers.jl " begin
    include("test_winchcontroller.jl")
end

@testset verbose=true "wc_settings.jl      " begin
    include("test_wc_settings.jl")
end

@testset verbose=true "aqua.jl             " begin
    include("aqua.jl")
end
end