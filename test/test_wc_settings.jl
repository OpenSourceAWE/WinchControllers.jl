@testset "settings from yaml" begin
    update = false
    dt = 0.001
    wcs = WCSettings(update; dt)
    @test wcs.i_speed ≈ 4.0
    @test wcs.pf_low ≈ 0.000144
    update = true
    dt = 0.001
    wcs = WCSettings(update; dt)
    @test wcs.pf_low ≈ 0.00014
end

