using Test, WinchControllers, KiteUtils

# The forward tension curve of the trajectory optimizer, which `calc_vro_soft`
# inverts: nominal `T = (v/kv)^2`, soft-saturated at `f_high` FIRST and at
# `f_low` second. Written out here rather than imported, so the test pins the
# inverse against an independent statement of the law.
sp_fwd(x) = max(x, 0) + log1p(exp(-abs(x)))
function tension_fwd(wcs, v)
    t = (v / wcs.kv)^2
    t = wcs.f_high - sp_fwd(wcs.softplus_beta * (wcs.f_high - t)) / wcs.softplus_beta
    wcs.f_low + sp_fwd(wcs.softminus_beta * (t - wcs.f_low)) / wcs.softminus_beta
end

function soft_settings(; dt = 0.02)
    wcs = WCSettings(dt = dt)
    wcs.mode = "reelout"
    wcs.force_limit = "soft"
    wcs.kv = 0.0408
    wcs.f_low = 350.0
    wcs.f_high = 8000.0
    wcs.v_sat = 8.0
    wcs
end

@testset "LowPass" begin
    lp = LowPass(0.1, 0.0)
    @test calc_output(lp, 3.0) == 3.0        # tau = 0 is a pass-through
    on_timer(lp)
    @test calc_output(lp, -1.0) == -1.0

    lp = LowPass(0.1, 1.0)
    @test calc_output(lp, 2.0) == 2.0        # first sample passes through
    on_timer(lp)
    out = calc_output(lp, 0.0)
    @test out ≈ 2.0 * (1 - 0.1 / 1.1)
    on_timer(lp)
    for _ in 1:400
        calc_output(lp, 0.0)
        on_timer(lp)
    end
    @test abs(lp.output) < 1e-6              # settles on the input
    reset(lp)
    @test calc_output(lp, 5.0) == 5.0

    # Asymmetric: fast attack, slow release.
    lp = LowPass(0.1, 2.0, 0.0)          # instant on the way up, tau 2 on the way down
    @test calc_output(lp, 0.0) == 0.0
    on_timer(lp)
    @test calc_output(lp, 10.0) == 10.0  # rising -> tau_rise = 0, passes straight through
    on_timer(lp)
    out = calc_output(lp, 0.0)           # falling -> tau = 2.0, lags
    @test out ≈ 10.0 * (1 - 0.1 / 2.1)
    @test out > 9.0

    # Biased toward the PEAKS of an oscillation, which is the point of using it
    # to drive a limiter: the mean of the output exceeds the mean of the input.
    lp = LowPass(0.02, 2.0, 0.1)
    sig = [100.0 + 50.0 * sin(2pi * i * 0.02 / 5.0) for i in 1:2000]
    out = Float64[]
    for x in sig
        push!(out, calc_output(lp, x)); on_timer(lp)
    end
    settled = out[1001:end]
    @test sum(settled) / length(settled) > sum(sig[1001:end]) / length(sig[1001:end])
    @test maximum(settled) <= maximum(sig) + 1e-9   # never overshoots the input

    # tau_rise defaults to tau, i.e. the symmetric filter is unchanged.
    a = LowPass(0.02, 1.5); b = LowPass(0.02, 1.5, 1.5)
    for x in (3.0, -2.0, 7.0, 7.0, 0.5)
        @test calc_output(a, x) == calc_output(b, x)
        on_timer(a); on_timer(b)
    end
end

@testset "force_limit_tau_rise defaults to symmetric" begin
    # The sentinel is NaN, resolved where the filter is built. A plain default of
    # `force_limit_tau` would freeze the STRUCT default, so a settings file (or a
    # caller) that sets only `force_limit_tau` would get a silently asymmetric
    # filter -- rising on 1.0 s while falling on whatever was configured.
    wcs = soft_settings()
    @test isnan(wcs.force_limit_tau_rise)
    wcs.force_limit_tau = 2.48              # as a YAML load would leave it
    wc = WinchController(wcs)
    @test wc.calc.filter.tau == 2.48
    @test wc.calc.filter.tau_rise == 2.48   # follows tau, not the struct default

    # And an explicit value is honoured.
    wcs2 = soft_settings()
    wcs2.force_limit_tau = 2.48
    wcs2.force_limit_tau_rise = 0.25
    wc2 = WinchController(wcs2)
    @test wc2.calc.filter.tau == 2.48
    @test wc2.calc.filter.tau_rise == 0.25

    # `force_limit_tau = 0` must stay a pass-through in BOTH directions.
    wcs3 = soft_settings()
    wcs3.force_limit_tau = 0.0
    wc3 = WinchController(wcs3)
    @test wc3.calc.filter.tau == 0.0
    @test wc3.calc.filter.tau_rise == 0.0
end

@testset "calc_vro_soft" begin
    wcs = soft_settings()

    # Exact round trip against the forward law, over the range the optimizer uses.
    for v in [0.5, 1.0, 2.0, 3.0, 3.75]
        @test calc_vro_soft(wcs, tension_fwd(wcs, v)) ≈ v atol = 1e-9
    end

    # The values the plan tabulates, so a change of convention is visible here.
    @test tension_fwd(wcs, 1.0) ≈ 1176.0 atol = 0.1
    @test tension_fwd(wcs, 3.75) ≈ 7506.7 atol = 0.1
    # 1/softminus_beta is larger than f_low, so the smoothing dominates the limit
    # it smooths: the effective floor is 883 N, not 350 N.
    @test tension_fwd(wcs, 0.0) ≈ 883.2 atol = 0.1
    @test calc_vro_soft(wcs, 883.0) == 0.0

    @test calc_vro_soft(wcs, 0.0) == 0.0
    @test calc_vro_soft(wcs, wcs.f_low) == 0.0
    @test calc_vro_soft(wcs, wcs.f_high) == wcs.v_sat
    @test calc_vro_soft(wcs, 20_000.0) == wcs.v_sat     # never exceeds v_sat
    forces = range(0.0, 20_000.0; length = 2001)
    speeds = [calc_vro_soft(wcs, f) for f in forces]
    @test all(isfinite, speeds)
    @test all(>=(-1e-12), diff(speeds))                 # monotonic, never reels in

    # `f_low` is per-call: `calc_v_set` passes a wind-dependent one.
    @test calc_vro_soft(wcs, 1000.0, 700.0) == 0.0
    @test calc_vro_soft(wcs, 1000.0, 350.0) > 0.0

    # Approaching f_high the soft law reels out FASTER than the bare square root
    # — shedding force is how a speed law limits it. Near the floor it reels out
    # slower, down to a stop; the two cross around 4 kN on this winch.
    for f in [6000.0, 7000.0, 7900.0]
        @test calc_vro_soft(wcs, f) > wcs.kv * sqrt(f)
    end
    for f in [1000.0, 1500.0, 2500.0]
        @test calc_vro_soft(wcs, f) < wcs.kv * sqrt(f)
    end
end

@testset "soft force limit, WinchController" begin
    dt = 0.02
    wcs = soft_settings(; dt)
    wcs.force_limit_tau = 0.0        # unfiltered, so the law is testable pointwise
    wc = WinchController(wcs)

    # The UpperForceController is held in reset for the whole run.
    @test wc.ufc.reset
    @test ! wc.ufc.active
    for f in [1000.0, 8000.0, 12_000.0]
        for _ in 1:100
            calc_v_set(wc, 0.0, f, wcs.f_low)
            on_timer(wc)
        end
        @test ! wc.ufc.active
        @test get_state(wc) != Int(wcsUpperForceLimit)
    end

    # `v_set_in` follows the soft law, not the bare square root.
    wc = WinchController(wcs)
    for _ in 1:200
        calc_v_set(wc, 0.0, 7000.0, wcs.f_low)
        on_timer(wc)
    end
    @test wc.calc.input_a ≈ calc_vro_soft(wcs, 7000.0, wcs.f_low)
    @test wc.calc.input_a > calc_vro(wcs, 7000.0)

    # The hand-over speeds stay on the NOMINAL law: the soft law is 0 at f_low
    # and v_sat at f_high, which would jam both force controllers.
    @test wc.lfc.v_sw ≈ calc_vro(wcs, wc.lfc.f_set) * 1.05
    @test wc.ufc.v_sw ≈ calc_vro(wcs, wc.ufc.f_set) * 0.95

    # The LowerForceController stays: the soft law never commands reel-in.
    wc = WinchController(wcs)
    for _ in 1:400
        calc_v_set(wc, 0.0, 100.0, wcs.f_low)
        on_timer(wc)
    end
    @test wc.lfc.active
    @test get_state(wc) == Int(wcsLowerForceLimit)
end

@testset "soft force limit, filtering" begin
    dt = 0.02
    wcs = soft_settings(; dt)
    wcs.force_limit_tau = 1.0
    wc = WinchController(wcs)

    # A square-wave force ripple around 7000 N: the filter must shrink the swing
    # of the speed command, which the near-vertical inverse would amplify.
    ripple = [7000.0 + (isodd(i ÷ 5) ? 900.0 : -900.0) for i in 1:1000]
    v_in = Float64[]
    for f in ripple
        calc_v_set(wc, 0.0, f, wcs.f_low)
        push!(v_in, wc.calc.input_a)
        on_timer(wc)
    end
    settled = v_in[501:end]
    raw = [calc_vro_soft(wcs, f, wcs.f_low) for f in ripple[501:end]]
    @test (maximum(settled) - minimum(settled)) < 0.4 * (maximum(raw) - minimum(raw))
    # and it is unbiased: the mean of the filtered command tracks the mean force
    @test abs(sum(settled) / length(settled) -
              calc_vro_soft(wcs, 7000.0, wcs.f_low)) < 0.2
end

@testset "force_limit validation" begin
    wcs = WCSettings(dt = 0.02)
    wcs.force_limit = "soft"
    wcs.mode = "piecewise"
    @test_throws ErrorException WinchController(wcs)   # soft needs the reel-out law

    wcs.force_limit = "smooth"
    @test_throws ErrorException WinchController(wcs)

    # "hard" is the default and leaves the UpperForceController running.
    wcs = WCSettings(dt = 0.02)
    @test wcs.force_limit == "hard"
    wc = WinchController(wcs)
    @test ! wc.ufc.reset
end
