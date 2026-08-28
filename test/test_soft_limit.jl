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

# Independent statement of the smooth minimum `calc_vro_soft` uses for the
# reel-in/reel-out handover, same reasoning as `tension_fwd` above.
soft_min_ref(a, b, beta) = -log(exp(-beta * a) + exp(-beta * b)) / beta

# Independent statement of the tension-curve inverse `soft_lfc = true` uses
# above f_low: HARD-clamped at f_low (no softminus_beta there any more -- see
# calc_vro_soft's docstring), only softplus_beta still smooths the f_high
# side. Reuses the package's own `sp_inv` for that shared, already
# round-trip-tested f_high step; only the f_low side is independent here.
function v_sqrt_hard(wcs, f)
    f >= wcs.f_high && return wcs.v_sat
    t = wcs.f_high - WinchControllers.sp_inv(wcs.softplus_beta * (wcs.f_high - f)) / wcs.softplus_beta
    min(wcs.kv * sqrt(max(t, 0.0)), wcs.v_sat)
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

    # The `soft_lfc` keyword defaults to the `wcs.soft_lfc` field, which is
    # false here: every call above already exercises the regression path.
    # Spelling it out once more so a future change of the default cannot
    # silently break it.
    @test calc_vro_soft(wcs, 1000.0) == calc_vro_soft(wcs, 1000.0; soft_lfc=false)
    @test ! wcs.soft_lfc
end

@testset "calc_vro_soft, soft_lfc" begin
    wcs = soft_settings()
    @test wcs.v_reel_in == -2.0    # the struct default
    @test wcs.reel_in_beta == 20.0 # the struct default; satisfies reel_in_beta*kv*sqrt(f_low)>=8
    v_ri = wcs.v_reel_in          # -2.0
    m = -v_ri / wcs.f_low         # slope of the reel-in line [m/s per N]

    # The line spans the WHOLE physically valid range below f_low (force is
    # never negative), so there is no separate ramp-width setting any more --
    # but it is shifted DOWN by log(2)/reel_in_beta from the line through
    # (0, v_reel_in) and (f_low, 0), so it meets soft_min continuously at
    # f_low (where the unshifted line and the tension-curve inverse both sit
    # at exactly 0) without eroding its own constant slope -- see
    # calc_vro_soft's docstring. Exact at f_low/2 (the shift, not an
    # approximation); at force = 0 the shifted value is clamped back up to
    # v_ri by the outer max, since it would otherwise undershoot it.
    @test calc_vro_soft(wcs, 0.0; soft_lfc=true) == v_ri
    @test calc_vro_soft(wcs, wcs.f_low; soft_lfc=true) ≈ -log(2) / wcs.reel_in_beta
    @test calc_vro_soft(wcs, wcs.f_low / 2; soft_lfc=true) ≈ v_ri / 2 - log(2) / wcs.reel_in_beta

    # Clamp below zero force (never happens physically, but the formula must
    # not run away for a caller that somehow passes a negative value).
    @test calc_vro_soft(wcs, -100.0; soft_lfc=true) == v_ri

    # Monotone over the whole range, no dead band anywhere above f_low — the
    # defect the sharper corner exists to close.
    forces = range(-100.0, 1.1 * wcs.f_high; length = 2001)
    speeds = [calc_vro_soft(wcs, f; soft_lfc=true) for f in forces]
    @test all(isfinite, speeds)
    @test all(>=(-1e-9), diff(speeds))
    @test count(==(0.0), [calc_vro_soft(wcs, f; soft_lfc=true)
                           for f in (wcs.f_low + 0.5):0.5:(wcs.f_high - 0.5)]) == 0

    # The point of the whole construction: a finite slope through the crossing,
    # in contrast to the vertical tangent of the plain √ law at f_low. The
    # shifted line's slope is exactly m everywhere below f_low (no decay-based
    # erosion approaching the crossing, unlike blending through soft_min
    # itself would give) except where the v_reel_in floor clamps it flat near
    # force = 0, hence the one-sided bound; checked up to f_low - 1, clear of
    # the kink into soft_min exactly at f_low.
    d(x) = (calc_vro_soft(wcs, x + 0.01; soft_lfc=true) -
            calc_vro_soft(wcs, x - 0.01; soft_lfc=true)) / 0.02
    for f in -50.0:5.0:(wcs.f_low - 1.0)
        @test abs(d(f)) <= 1.05 * m
    end

    # Soft handover above f_low: `soft_min(line, v_sqrt_hard, reel_in_beta)` —
    # never above the hard min (never exceeds what the tension curve allows,
    # nor the line), and never more than `log(2)/reel_in_beta` below it.
    slack = log(2) / wcs.reel_in_beta
    for f in [400.0, 500.0, 700.0, 1000.0, 3000.0]
        v_sqrt = v_sqrt_hard(wcs, f)
        hard_min = min(m * (f - wcs.f_low), v_sqrt)
        v = calc_vro_soft(wcs, f; soft_lfc=true)
        @test v <= hard_min + 1e-9
        @test v >= hard_min - slack - 1e-9
        @test v <= v_sqrt + 1e-9
    end

    # Near f_low the (hard-clamped) tension curve inverse is already close to
    # its own kv*sqrt(f_low) value, clearing the line by such a wide margin
    # that the softening is invisible — matches the hard min closely.
    for f in [351.0, 355.0, 360.0, 400.0]
        v_sqrt = v_sqrt_hard(wcs, f)
        hard_min = min(m * (f - wcs.f_low), v_sqrt)
        @test calc_vro_soft(wcs, f; soft_lfc=true) ≈ hard_min atol = 1e-4
    end

    # Pinned exactly against the independent soft_min_ref, close to the actual
    # crossing (F_x ≈ 505 N here, where line ≈ v_sqrt_hard) where the
    # softening is largest.
    for f in [480.0, 495.0, 505.0, 515.0, 530.0]
        v_sqrt = v_sqrt_hard(wcs, f)
        @test calc_vro_soft(wcs, f; soft_lfc=true) ≈
              soft_min_ref(m * (f - wcs.f_low), v_sqrt, wcs.reel_in_beta)
    end
end

@testset "soft_min (calc_vro_soft's handover, pinned independently)" begin
    # Exact at the crossing: soft_min(a, a, beta) = a - log(2)/beta.
    for beta in [1.0, 5.0, 20.0, 100.0]
        @test soft_min_ref(1.5, 1.5, beta) ≈ 1.5 - log(2) / beta
    end

    # Always <= the hard min, converging to it as beta grows. beta is kept
    # moderate (not e.g. 1e6): the NAIVE log-sum-exp reference above
    # over/underflows once beta*max(|a|, |b|) exceeds ~700, unlike the actual
    # (numerically stable) `soft_min` — 100 is already sharp enough here to
    # confirm convergence without hitting that wall.
    for (a, b) in [(1.0, 2.0), (-0.5, 3.0), (0.0, 0.0), (5.0, 5.0001)]
        for beta in [1.0, 10.0, 100.0]
            @test soft_min_ref(a, b, beta) <= min(a, b) + 1e-12
        end
        @test soft_min_ref(a, b, 100.0) ≈ min(a, b) atol = 1e-2
    end
    @test soft_min_ref(0.0, 10.0, 5.0) ≈ 0.0 atol = 1e-2   # far apart: matches hard min closely

    # Symmetric in its two arguments.
    @test soft_min_ref(2.0, 7.0, 3.0) == soft_min_ref(7.0, 2.0, 3.0)

    # Monotone non-decreasing in F when both a(F) and b(F) are.
    beta = 4.0
    a(F) = 0.01 * F
    b(F) = 0.5 + 0.001 * F
    vals = [soft_min_ref(a(F), b(F), beta) for F in 0.0:0.1:200.0]
    @test all(>=(-1e-12), diff(vals))
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

    # `soft_lfc = false` (the default): the LowerForceController stays,
    # since the soft law never commands reel-in on its own.
    @test ! wcs.soft_lfc
    wc = WinchController(wcs)
    for _ in 1:400
        calc_v_set(wc, 0.0, 100.0, wcs.f_low)
        on_timer(wc)
    end
    @test wc.lfc.active
    @test get_state(wc) == Int(wcsLowerForceLimit)
end

@testset "soft force limit, soft_lfc" begin
    dt = 0.02
    wcs = soft_settings(; dt)
    wcs.force_limit_tau = 0.0
    wcs.soft_lfc = true   # struct default reel_in_beta=20 already satisfies
                           # reel_in_beta * kv * sqrt(f_low) >= 8
    wc = WinchController(wcs)

    # The LowerForceController is held in permanent reset, same as the UFC.
    @test wc.lfc.reset
    @test ! wc.lfc.active

    # Below f_low, `calc_v_set` now feeds the reel-in ramp instead of 0 into the
    # speed controller. Checked on `input_a`, the direct output of the law
    # (same quantity the pre-existing "v_set_in follows the soft law" test
    # above checks), not the closed-loop `v_set` — this run pins v_act at 0.0
    # forever, so the speed loop never actually settles.
    for _ in 1:400
        calc_v_set(wc, 0.0, 100.0, wcs.f_low)
        on_timer(wc)
    end
    @test ! wc.lfc.active
    @test wc.calc.input_a ≈ calc_vro_soft(wcs, 100.0, wcs.f_low; soft_lfc=true)
    @test wc.calc.input_a < 0.0

    # With both force controllers permanently in reset, the SpeedController is
    # always in control — `get_state`/`get_f_err` can never report anything
    # else, at any force.
    for f in [0.0, 100.0, wcs.f_low, 1000.0, wcs.f_high, 12_000.0]
        for _ in 1:50
            calc_v_set(wc, 0.0, f, wcs.f_low)
            on_timer(wc)
        end
        @test get_state(wc) == Int(wcsSpeedControl)
        @test isnan(get_f_err(wc))
    end

    # `f_err(logger)`/`gamma(logger)` must not throw when every logged f_err is
    # NaN — the direct consequence of `get_f_err` above.
    logger = WCLogger(1.0, dt, 1.0, 1.0, 0.2, 0.9)
    for _ in 1:length(logger)
        log(logger; f_err = NaN, v_set = 1.0, v_err = 0.0, jerk = 0.0, acc = 0.0)
    end
    @test f_err(logger) == 0.0
    @test isfinite(gamma(logger))
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

@testset "soft_lfc validation" begin
    # Requires force_limit = "soft".
    wcs = WCSettings(dt = 0.02)
    wcs.soft_lfc = true
    @test wcs.force_limit == "hard"
    @test_throws ErrorException WinchController(wcs)

    base() = begin
        w = WCSettings(dt = 0.02)
        w.mode = "reelout"
        w.force_limit = "soft"
        w.soft_lfc = true
        w
    end

    # The struct default (reel_in_beta = 20, kv = 0.06) passes the sharpness
    # check this whole design exists to enforce (see calc_vro_soft) on its own.
    wcs = base()
    @test wcs.reel_in_beta * wcs.kv * sqrt(wcs.f_low) >= 8
    @test WinchController(wcs) isa WinchController

    # An insufficiently sharp reel_in_beta fails it: with kv = 0.0408 (as
    # data/wc_settings_soft_lfc.yaml uses), reel_in_beta = 2 gives
    # 2 * 0.0408 * sqrt(350) ≈ 1.53 < 8 -- the exact combination that yaml
    # shipped with before being retuned, leaving a visible hump instead of a
    # smooth corner at the f_low handover.
    wcs = base()
    wcs.kv = 0.0408
    wcs.reel_in_beta = 2.0
    @test wcs.reel_in_beta * wcs.kv * sqrt(wcs.f_low) < 8
    @test_throws ErrorException WinchController(wcs)

    # Sharpening reel_in_beta is enough on its own.
    wcs.reel_in_beta = 20.0
    @test WinchController(wcs) isa WinchController

    # v_reel_in must be strictly negative and within v_ri_max.
    wcs = base()
    wcs.v_reel_in = 0.5
    @test_throws ErrorException WinchController(wcs)
    wcs.v_reel_in = -100.0
    @test_throws ErrorException WinchController(wcs)

    # reel_in_beta must be positive (the sharpness check above subsumes this).
    wcs = base()
    wcs.reel_in_beta = 0.0
    @test_throws ErrorException WinchController(wcs)
    wcs.reel_in_beta = -5.0
    @test_throws ErrorException WinchController(wcs)
end
