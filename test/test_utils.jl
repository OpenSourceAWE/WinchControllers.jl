if false include("../src/utils.jl") end
if false include("../src/wc_settings.jl") end

@testset "merge_angles" begin
    alpha=0
    beta=pi/2
    factor_beta=0
    angle = merge_angles(alpha, beta, factor_beta)
    @test angle ≈ 0 atol=1e-8
    factor_beta=1
    angle = merge_angles(alpha, beta, factor_beta)
    @test angle ≈ pi/2 rtol=1e-8
    factor_beta=0.5
    angle = merge_angles(alpha, beta, factor_beta)
    @test angle ≈ pi/4 rtol=1e-8
    angle = merge_angles(alpha, beta+2pi, factor_beta)
    @test angle ≈ pi/4 rtol=1e-8
    angle = merge_angles(alpha, beta-2pi, factor_beta)
    @test angle ≈ pi/4 rtol=1e-8
end

@testset "@limit" begin
    x = 22
    @limit x 0 10
    @test x == 10
    x = -10
    @limit x 0 10
    @test x == 0
    x = -10
    @limit x -1.5 10
    @test x == -1.5
    x = 22
    @limit x 10
    @test x == 10
    x = -22
    @limit x 10
    @test x == -22
end

@testset "get_startup" begin
    wcs = WCSettings(dt=0.02)
    startup_signal = get_startup(wcs, 1000)
    @test length(startup_signal) == 1000
    @test all(startup_signal[14:end] .== 1.0)
end