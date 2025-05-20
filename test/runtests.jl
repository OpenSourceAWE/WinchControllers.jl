using WinchControllers, KiteUtils
using Test

const __TEST__ = true

cd("..")
KiteUtils.set_data_path("data") 

@testset verbose=true "WinchControllers.jl " begin
    include("test_winchcontroller.jl")
end

@testset verbose=true "utils.jl            " begin
    include("test_utils.jl")
end

@testset verbose=true "wc_settings.jl      " begin
    include("test_wc_settings.jl")
end

@testset verbose=true "aqua.jl             " begin
    include("aqua.jl")
end
