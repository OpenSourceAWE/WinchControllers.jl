using Test, WinchControllers

@testset "force_to_torque" begin
    @test force_to_torque(1000.0, 0.2, 5.0, 10.0) ≈ -30.0
    @test force_to_torque(1000.0, 0.2, 5.0, 10.0; ff_scale=0.5) ≈ -10.0
    # friction is applied in full regardless of ff_scale
    @test force_to_torque(0.0, 0.2, 5.0, -25.0) ≈ -25.0
    @test force_to_torque(0.0, 0.2, 5.0, -25.0; ff_scale=0.0) ≈ -25.0
end

@testset "winch_acc_limit" begin
    @test winch_acc_limit(5.0) === 5.0
    @test winch_acc_limit(0.0) === Inf
    @test winch_acc_limit(-3.0) === Inf
end

@testset "winch_position_torque! saturation and rate limiting" begin
    dt = 0.1
    wcs = WCSettings(dt=dt)

    # zero error: torque is exactly the force feed-forward, no correction
    wpc = WinchPosController(wcs; dt)
    torque = winch_position_torque!(wpc, 0.0, 0.0, 0.0, 1000.0, 0.2, 5.0, 10.0, dt, 2.0, 100.0)
    @test torque ≈ force_to_torque(1000.0, 0.2, 5.0, 10.0; ff_scale=wpc.ff_scale)
    @test wpc.v_sp_prev == 0.0

    # huge length error, rate limit not binding: setpoint saturates at speed_limit
    wpc_sat = WinchPosController(wcs; dt)
    winch_position_torque!(wpc_sat, 1000.0, 0.0, 0.0, 0.0, 0.2, 5.0, 0.0, dt, 2.0, 100.0)
    @test wpc_sat.v_sp_prev == 2.0

    # huge length error, speed limit not binding: setpoint ramps at acceleration_limit*dt per step
    wpc_ramp = WinchPosController(wcs; dt)
    dv_max = 1.0 * dt
    winch_position_torque!(wpc_ramp, 1000.0, 0.0, 0.0, 0.0, 0.2, 5.0, 0.0, dt, 100.0, 1.0)
    @test wpc_ramp.v_sp_prev ≈ dv_max
    winch_position_torque!(wpc_ramp, 1000.0, 0.0, 0.0, 0.0, 0.2, 5.0, 0.0, dt, 100.0, 1.0)
    @test wpc_ramp.v_sp_prev ≈ 2dv_max
end

@testset "winch_force_torque! f_lpf init and force floor" begin
    dt = 0.1
    wcs = WCSettings(dt=dt)

    # first call initializes f_lpf to the measured force, so it steps no torque
    wfc = WinchForceController(wcs)
    @test isnan(wfc.f_lpf)
    t1 = winch_force_torque!(wfc, 0.0, 0.0, 0.0, 800.0, 0.2, 5.0, 10.0, dt)
    @test wfc.f_lpf == 800.0
    @test t1 ≈ force_to_torque(800.0, 0.2, 5.0, 10.0)

    # subsequent call low-passes towards the new force, not directly onto it
    t2 = winch_force_torque!(wfc, 0.0, 0.0, 0.0, 0.0, 0.2, 5.0, 10.0, dt)
    alpha = dt / (wfc.force_tau + dt)
    f_lpf_expected = 800.0 + alpha * (0.0 - 800.0)
    @test wfc.f_lpf ≈ f_lpf_expected
    @test t2 ≈ force_to_torque(f_lpf_expected, 0.2, 5.0, 10.0)

    # reference force below force_min is floored before conversion to torque
    wfc_floor = WinchForceController(wcs)
    t_floor = winch_force_torque!(wfc_floor, 0.0, 0.0, 0.0, 50.0, 0.2, 5.0, 10.0, dt)
    @test t_floor ≈ force_to_torque(wfc_floor.force_min, 0.2, 5.0, 10.0)
end
