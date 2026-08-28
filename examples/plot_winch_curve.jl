# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using WinchControllers, MakieControlPlots, KiteUtils
import MakieControlPlots: plotxy

# Winch curve under SOFT force limiting, with and without the reel-in extension
# (`WCSettings.soft_lfc`): a straight line through `(0, v_reel_in)` and
# `(f_low, 0)` -- the whole physically valid range below f_low, since force is
# never negative -- see `calc_vro_soft`. It needs a lower corner sharp enough
# that the tension curve itself reaches 0 at f_low (softminus_beta * f_low >= 8);
# the shipped default of `data/wc_settings.yaml` misses this by roughly an
# order of magnitude, so this example loads `data/wc_settings_soft_lfc.yaml`
# instead (via `system_soft_lfc.yaml`), which already sets force_limit,
# softminus_beta, soft_lfc, v_reel_in and reel_in_beta -- edit that file to
# change the curve, not this script.
load_settings("system_soft_lfc.yaml")
wcs = WCSettings(dt=0.02)
update(wcs)

# Both curves, in the style of a motor/winch datasheet torque-speed curve: speed
# on the x-axis, force on the y-axis. Sweep force past f_high to show all
# regions: the reel-in line below f_low, the soft-saturated section, and the
# v_sat plateau above f_high. The two curves diverge in SPEED at a given force,
# not the other way round, so they need independent X series -- `plotxy` rather
# than `plot`.
force = range(0.0, 1.1 * wcs.f_high, 500)
speed_false = calc_vro_soft.(Ref(wcs), force; soft_lfc=false)
speed_true  = calc_vro_soft.(Ref(wcs), force; soft_lfc=true)

p = plotxy([speed_false, speed_true], [force, force];
           xlabel="speed [m/s]", ylabel="force [N]",
           legend=["soft_lfc = false", "soft_lfc = true"], fig="winch_curve")
display(p)
