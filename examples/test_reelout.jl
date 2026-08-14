# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers, Statistics; tic()

# Test the REEL_OUT law (v_set = kv * sqrt(force)) against a triangle wind speed and a
# dummy tether-force model, using the same winch-controller loop as
# test_winchcontroller.jl. Run twice: once with f_high above the peak force, to see the
# clean law, and once at the default f_high, to see the hand-over to the upper-force
# controller at the wind peaks.
using WinchControllers, MakieControlPlots, KiteUtils

# Open-loop dummy force: independent of v_ro, so K_KITE alone sets the peak — chosen so
# that 5 m/s wind gives 6000 N. The winch cannot influence its own input, so this checks
# the law and the limiter hand-over, not closed-loop stability.
const K_KITE = 240.0   # [N*s^2/m^2]
calc_force_openloop(v_wind, v_ro) = K_KITE * v_wind^2

# Closed-loop variant (the model test_winchcontroller.jl uses): force depends on the
# speed relative to the wind, so the winch affects its own input and the loop can
# actually oscillate, not just lag.
calc_force_closedloop(v_wind, v_ro) = K_KITE * (v_wind - v_ro)^2

CLOSED_LOOP = false
calc_force = CLOSED_LOOP ? calc_force_closedloop : calc_force_openloop

set = load_settings("system.yaml")

DURATION   = 60.0
V_WIND_MAX = 5.0   # max wind speed of the test wind                    [m/s]
V_WIND_MIN = 0.0   # min wind speed of the test wind                    [m/s]
FREQ_WIND  = 0.1   # 1/period; 10 s period, six periods over DURATION   [1/s]

# f_low = 350 N is reached at v_wind = sqrt(350/K_KITE) = 1.21 m/s, so the
# lower-force controller engages every trough — expected, not a failure.

function run_leg(f_high, label)
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.mode = "reelout"
    wcs.f_high = f_high

    lg = WCLogger(DURATION, wcs.dt, set.max_force, wcs.max_acc, wcs.damage_factor, wcs.jerk_factor)
    STARTUP = get_startup(wcs, length(lg))
    V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, length(lg))

    wc = WinchController(wcs)
    winch = Winch(wcs, set)
    f_low = wcs.f_low

    for i in 1:length(lg)
        local force, v_set_out
        v_wind = V_WIND[i]

        v_act = get_speed(winch)
        force = calc_force(v_wind, v_act)
        set_force(winch, force)

        v_set_out = calc_v_set(wc, v_act, force, f_low)

        set_v_set(winch, v_set_out)
        on_timer(winch)
        on_timer(wc)

        status = get_status(wc)
        log(lg; v_wind, v_ro=v_act, acc=get_acc(winch), state=get_state(wc), reset=status[1],
            active=status[2], force=status[3], jerk=winch.jerk, f_set=status[4],
            f_err=get_f_err(wc), v_err=get_v_err(wc), v_set=get_v_set(wc), v_set_out,
            v_set_in=get_v_set_in(wc))
    end

    f_high_line = fill(f_high * 0.001, length(lg))
    p = plotx(lg.time, lg.v_wind, [lg.force * 0.001, f_high_line], [lg.v_ro, lg.v_set_in], lg.state,
        title = "REEL_OUT law test — $label",
        ylabels = ["v_wind [m/s]", "F_t [kN]", "v_ro [m/s]", "state"],
        labels = ["v_wind", ["F_t", "f_high"], ["v_ro", "v_set_in"],
                  "0=lower force, 1=speed, 2=upper force"],
        ysize = 10,
        fig = "test_reelout_$label")
    display(p)
    lg
end

lg_clean   = run_leg(8000.0, "clean")
lg_limited = run_leg(WCSettings(dt=0.02).f_high, "limited")

toc()

kv = WCSettings(dt=0.02).kv
println("kv * sqrt(6000 N) = $(round(kv * sqrt(6000.0), digits=2)) m/s (clean-leg peak target)")
println("Note: every trough drives force below f_low and the LowerForceController reels \
        all the way in, so v_ro dips well below the kv*sqrt(F) line right after each \
        trough — see the state panel. test/test_reelout.jl checks tracking only once \
        speed control has been continuously active for >= 1 s.")
