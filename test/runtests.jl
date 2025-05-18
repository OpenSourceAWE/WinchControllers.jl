using WinchControllers
using Test

const __TEST__ = true

@testset "WinchControllers.jl" begin
    include("test_winchcontroller.jl")
end

@testset "utils.jl" begin
    include("test_utils.jl")
end
