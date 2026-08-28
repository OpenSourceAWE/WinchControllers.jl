# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using WinchControllers, MakieControlPlots
import MakieControlPlots: plot

# Winch curve under SOFT force limiting, with the optional reel-in extension
# (`WCSettings.soft_reel_in`): a straight line through `(f_low, 0)` and
# `(f_low - f_reel_in_band, v_reel_in)`, capped where the reel-out tension curve
# overtakes it -- see `calc_vro_soft`. It needs a lower corner sharp enough that
# the tension curve itself reaches 0 at f_low (softminus_beta * f_reel_in_band >= 2);
# the shipped default misses this by ~27x, so it is bumped here for the demo.
wcs = WCSettings(dt=0.02)
update(wcs)
wcs.force_limit = "soft"
wcs.softminus_beta = max(wcs.softminus_beta, 2.0 / wcs.f_reel_in_band)

# Winch curve WITH reel-in, in the style of a motor/winch datasheet torque-speed
# curve: speed on the x-axis, force on the y-axis. Sweep force past f_high to show
# all regions: the reel-in ramp/clamp below f_low, the soft-saturated section, and
# the v_sat plateau above f_high.
force = range(0.0, 1.1 * wcs.f_high, 500)
speed = calc_vro_soft.(Ref(wcs), force; reel_in=true)

p = plot(speed, force; xlabel="speed [m/s]", ylabel="force [N]", fig="winch_curve")
display(p)

# Same law with and without the reel-in extension. Force on the x-axis here: the
# two curves diverge in SPEED, not force, so they cannot share a y-axis grid.
force_cmp = range(wcs.f_low - wcs.f_reel_in_band - 50.0, 1.1 * wcs.f_high, 500)
speed_false = calc_vro_soft.(Ref(wcs), force_cmp; reel_in=false)
speed_true  = calc_vro_soft.(Ref(wcs), force_cmp; reel_in=true)

pc = plot(force_cmp, [speed_false, speed_true];
          xlabel="force [N]", ylabel="speed [m/s]",
          labels=["reel_in = false", "reel_in = true"], fig="winch_curve_compare")
display(pc)
