using KiteUtils, WinchControllers
using Pkg, Test

const __TEST__ = true

if basename(pwd()) == "test"; cd(".."); end
KiteUtils.set_data_path(joinpath(dirname(@__DIR__), "data"))

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

@testset verbose=true "reelout.jl          " begin
    include("test_reelout.jl")
end

@testset verbose=true "aqua.jl             " begin
    include("aqua.jl")
end
end