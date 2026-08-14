using Test, WinchControllers, KiteUtils

@testset "REEL_OUT law" begin
    dt = 0.02
    wcs = WCSettings(dt=dt)
    update(wcs)
    wcs.mode = "reelout"
    wcs.f_high = 8000.0   # above the dummy force's 6000 N peak: no limiter engages

    K_KITE = 240.0        # 5 m/s wind -> 6000 N
    duration = 60.0
    v_wind_max = 5.0
    freq_wind = 0.1        # 1/period; 10 s period

    set = load_settings("system.yaml")
    lg = WCLogger(duration, dt, set.max_force, wcs.max_acc, wcs.damage_factor, wcs.jerk_factor)
    startup = get_startup(wcs, length(lg))
    v_wind_vec = startup .* get_triangle_wind(wcs, 0.0, v_wind_max, freq_wind, length(lg))

    wc = WinchController(wcs)
    winch = WinchControllers.Winch(wcs, set)
    f_low = wcs.f_low

    v_ro = zeros(length(lg))
    force = zeros(length(lg))
    speed_control = falses(length(lg))
    for i in eachindex(v_wind_vec)
        local f, v_set_out
        v_wind = v_wind_vec[i]
        v_act = get_speed(winch)
        f = K_KITE * v_wind^2
        set_force(winch, f)
        v_set_out = calc_v_set(wc, v_act, f, f_low)
        set_v_set(winch, v_set_out)
        on_timer(winch)
        on_timer(wc)
        v_ro[i] = v_act
        force[i] = f
        speed_control[i] = get_state(wc) == Int(wcsSpeedControl)
    end

    # Skip the soft-start ramp. Compare only where the speed controller has been
    # continuously active for >= 1 s: each trough drives the LowerForceController to
    # reel all the way in (v_ro down to ~-5 m/s here) before force climbs back above
    # f_low, and speed control needs a settling window to recover from that excursion
    # once it hands back — a real transient, not a law error, and not the hand-over
    # itself (which is instantaneous).
    settle_n = round(Int, 1.0 / dt)
    streak = zeros(Int, length(lg))
    for i in eachindex(speed_control)
        streak[i] = speed_control[i] ? (i == 1 ? 1 : streak[i - 1] + 1) : 0
    end
    settled = (lg.time .>= wcs.t_startup + 1.0) .& (streak .>= settle_n)
    @test count(settled) > length(lg) ÷ 4

    v_ro_expected = wcs.kv .* sqrt.(force[settled])
    @test maximum(abs.(v_ro[settled] .- v_ro_expected)) < 0.25
end
