@testset "saturate" begin
    @test saturate( 10.0, -1,   1) == 1
    @test saturate(-10.0, -1,   1) == -1
    @test saturate( 10.0, -1.5, 1.5) == 1.5
    @test saturate( 0.5, -1.5, 1.5) == 0.5
end

@testset "wrap2pi" begin
    @test wrap2pi(0) == 0
    @test wrap2pi(pi-0.001) == pi-0.001
    @test wrap2pi(-pi) == -pi
    @test wrap2pi(2pi) == 0
    @test wrap2pi(-2pi) == 0
    @test wrap2pi(1.5pi) ≈ -0.5pi
end

@testset "Integrator" begin
    dt = 0.05
    int1 = Integrator(dt)
    @test int1.output == 0.0
    @test int1.last_output == 0.0
    @test int1.i == 1.0
    int2 = Integrator(dt, 2)
    @test int2.output == 0.0
    @test int2.last_output == 0.0
    @test int2.i == 2.0
    int3 = Integrator(dt, 2, 2)
    @test int3.output == 2.0
    @test int3.last_output == 2.0
    @test int3.i == 2.0
    @test calc_output(int3, 0.5) == 2.05
    reset(int3, 1.1)
    @test int3.output == 1.1
    @test int3.last_output == 1.1
    @test int3.i == 2.0
    int3 = Integrator(dt, 2, 2)
    calc_output(int3, 0.5) == 2.05
    on_timer(int3)
    @test int3.last_output == 2.05
end

@testset "WinchController" begin
    wcs = WCSettings(dt=0.02)
    cvi = CalcVSetIn(wcs)
    force = wcs.f_low
    v_ro = calc_vro(wcs, force)
    @test v_ro ≈ 0
    v_ro = calc_vro(wcs, wcs.f_high)
    @test v_ro ≈ wcs.vf_max
    set_vset_pc(cvi, nothing, wcs.f_low)
    @test calc_output(cvi) ≈ 0
    set_vset_pc(cvi, nothing, wcs.f_high)
    @test calc_output(cvi) ≈ wcs.vf_max
    v_out_set = 1.0
    set_vset_pc(cvi, v_out_set)
    @test calc_output(cvi) ≈ wcs.vf_max
    for i in 1:15
        on_timer(cvi)
    end
    @test calc_output(cvi) ≈ v_out_set
end

@testset "UnitDelay" begin
    ud = UnitDelay()
    for i in 1:3
        out=calc_output(ud, i)
        on_timer(ud)
        @test out == i-1
    end
    reset(ud)
    @test ud.last_input == 0
    @test ud.last_output == 0
end

@testset "RateLimiter" begin
    rl = RateLimiter(1.0, 0.5)
    input = [0,0,1,2,3,3,3,3,3,2,1,0,0,0,0,0]
    out = zeros(16)
    for i in 1:16
        out[i] = calc_output(rl, input[i])
        on_timer(rl)
        # println(input[i], " ", out[i])
    end
    @test out == [0,0,0.5,1,1.5,2,2.5,3,3,2.5,2,1.5,1,0.5,0,0]
end

@testset "Mixer_2CH" begin
    m2 = Mixer_2CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m2, x[i], y[i])
        select_b(m2, i > 2)
        on_timer(m2)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])
end

@testset "Mixer_3CH" begin
    m3 = Mixer_3CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m3, x[i], y[i], 0)
        select_b(m3, i > 2)
        on_timer(m3)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])
    m3 = Mixer_3CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m3, x[i], 0, y[i])
        select_c(m3, i > 2)
        on_timer(m3)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])    
end

@testset "SpeedController" begin
    wcs = WCSettings(dt=0.02)
    sc = SpeedController(wcs)
    @test sc.wcs.dt == 0.02
    set_tracking(sc, 1.0)
    set_inactive(sc, true)
    set_inactive(sc, false)
    @test sc.integrator.output == 1.0
    @test sc.integrator.last_output == 1.0
    @test sc.limiter.output == 1.0
    @test sc.limiter.last_output == 1.0
    set_v_set_in(sc, 2.2)
    @test sc.v_set_in ≈ 2.2
    set_v_act(sc, 3.3)
    @test sc.v_act ≈ 3.3
    pid1 = SpeedController(wcs)
    set_inactive(pid1, false)
    set_v_set(pid1, -0.5)
    set_tracking(pid1, -0.5)
    set_inactive(pid1, false)
    set_v_set_in(pid1, 4.0)
    v_ro = 1.0
    set_v_act(pid1, v_ro)
    v_set_out = get_v_set_out(pid1)
    @test v_set_out ≈ -0.16
    set_v_act(pid1, 1.1)
    on_timer(pid1)
    v_set_out = get_v_set_out(pid1)
    @test get_v_err(pid1) ≈ 2.9
end

@testset "Winch" begin
    wcs = WCSettings(dt=0.02)
    set = se()
    @test set.winch_model == "AsyncMachine"
    w = Winch(wcs, set)
    v_set = 4.0
    set_v_set(w, v_set)
    @test w.v_set == v_set
    force = 1000.0
    set_force(w, force)
    @test w.force == force
    on_timer(w)
    @test get_speed(w) ≈  0.8841505712049121
    @test get_acc(w) ≈ 44.207528560245606
end

# using KiteControllers, Test
function test_winch()
    wcs = WCSettings(dt=0.02)
    set = se()
    w = Winch(wcs, set)
    v_set = 4.0
    set_v_set(w, v_set)
    @test w.v_set == v_set
    force = 1000.0
    set_force(w, force)
    @test w.force == force
    w
end

@testset "LowerForceController" begin
    wcs = WCSettings(dt=0.02)
    lfc = LowerForceController(wcs)
    @test lfc.wcs.dt == 0.02
    WinchControllers._set(lfc)
    @test lfc.active
    lfc.v_act = 1.0
    lfc.force = 1.0
    WinchControllers._update_reset(lfc)
    @test ! lfc.active
    set_v_act(lfc, 0.0)
    @test lfc.v_act == 0.0
    set_force(lfc, 300)
    @test lfc.force == 300
    set_reset(lfc, true)
    @test lfc.reset
    set_v_sw(lfc, 2.2)
    @test lfc.v_sw == 2.2
    set_tracking(lfc, 3.3)
    @test lfc.tracking == 3.3
    lfc = LowerForceController(wcs)
    x = [0.1, 0.2, 0.0]
    sat2_in, sat2_out, rate_out, int_in = WinchControllers.calc_sat2in_sat2out_rateout_intin(lfc, x)
    v_set_out = get_v_set_out(lfc)
    lfc.f_err = -2.0
    @test get_f_err(lfc) == -2.0
    lfc.f_set = 400.0
    @test WinchControllers.get_f_set_low(lfc) == 0.0
    WinchControllers._set(lfc)
    @test WinchControllers.get_f_set_low(lfc) == 400.0
    WinchControllers.on_timer(lfc)
    set_f_set(lfc, 330.0)
    @test lfc.f_set == 330.0
end

@testset "UpperForceController" begin
    wcs = WCSettings(dt=0.02)
    ufc = UpperForceController(wcs)
    @test ufc.wcs.dt == 0.02
    WinchControllers._set(ufc)
    @test ufc.active
    ufc.v_act = -1.0
    ufc.force = 1.0
    WinchControllers._update_reset(ufc)
    @test ! ufc.active
    set_v_act(ufc, 0.0)
    @test ufc.v_act == 0.0
    set_force(ufc, 3500)
    @test ufc.force == 3500
    set_reset(ufc, true)
    @test ufc.reset
    set_v_sw(ufc, 2.2)
    @test ufc.v_sw == 2.2
    set_tracking(ufc, 3.3)
    @test ufc.tracking == 3.3
    ufc = UpperForceController(wcs)
    x = [0.1, 0.2, 0.3]
    sat2_in, sat2_out, rate_out, int_in, int2_in = WinchControllers.calc_sat2in_sat2out_rateout_intin(ufc, x)
    v_set_out = get_v_set_out(ufc)
    ufc.f_err = -2.0
    @test get_f_err(ufc) == -2.0
    ufc.f_set = 3300.0
    @test WinchControllers.get_f_set_upper(ufc) == 0.0
    WinchControllers._set(ufc)
    @test WinchControllers.get_f_set_upper(ufc) == 3300.0
    WinchControllers.on_timer(ufc)
    set_f_set(ufc, 330.0)
    @test ufc.f_set == 330.0
end

@testset "WinchController" begin
    wcs = WCSettings(dt=0.02)
    wc = WinchController(wcs)
    v_act = 1.0
    force = 500.0
    f_low = wcs.f_low
    calc_v_set(wc, v_act, force, f_low)
    on_timer(wc)
    @test get_state(wc) == 1 # wcsSpeedControl
    @test isnothing(WinchControllers.get_set_force(wc))
end
