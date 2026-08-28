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
# never negative -- see `calc_vro_soft`. It needs reel_in_beta sharp enough
# that the smooth line/tension-curve handover at f_low is actually invisible
# (reel_in_beta * kv * sqrt(f_low) >= 8); the shipped default of
# `data/wc_settings.yaml` is fine (reel_in_beta = 20), but this example loads
# `data/wc_settings_soft_lfc.yaml` instead (via `system_soft_lfc.yaml`), which
# sets force_limit, soft_lfc, v_reel_in and reel_in_beta explicitly -- edit
# that file to change the curve, not this script.
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

# Third curve: AWETrim's own forward tension curve (`Winch.tension_curve` in
# AWETrim/src/awetrim/system/winch.py), independently re-derived here (AWETrim
# is Python, out of reach at plot time) -- T(v) = (v/k_v)^2, soft-clamped from
# ABOVE at f_max first, then from BELOW at f_min (the order calc_vro_soft's own
# docstring says its inversion UNDOES in reverse). AWETrim never inverts this to
# v(F) itself, so it is plotted directly as force(speed), unlike the two curves
# above. Reel-out only (AWETrim's server rejects mode="reelin"), hence v >= 0.
#
# Parameters are the literal WinchParams examples/awetrim_client.jl's
# `winch_from_wc` sends for the v03 (3 m/s wind) reel-out scenario in
# SimpleKiteControllers.jl, cross-checked against the archived
# output/scenarios/v03/reelout_150m_opt.yaml -- see PlanWinchCurve.md, "Can you
# plot the function that AWETrim uses".
sp_fwd(x) = max(x, 0.0) + log1p(exp(-abs(x)))
function awetrim_tension(v; k_v = 0.0408, f_min = 350.0, f_max = 8000.0,
                          softplus_beta = 0.001, softminus_beta = 0.001)
    t = (v / k_v)^2
    t = t - sp_fwd(softplus_beta * (t - f_max)) / softplus_beta
    t + sp_fwd(softminus_beta * (f_min - t)) / softminus_beta
end
v_awetrim = range(0.0, wcs.v_sat, 500)
force_awetrim = awetrim_tension.(v_awetrim)

# Fourth curve: calc_vro_soft with use_awe_trim=0.5, halfway between the
# `soft_lfc = true` curve above (0.0) and AWETrim's own curve above (1.0) --
# see calc_vro_soft's docstring. `soft_lfc=true` matches `speed_true`, the
# curve it is meant to sit between AWETrim and.
speed_blend = calc_vro_soft.(Ref(wcs), force; soft_lfc=true, use_awe_trim=0.5)

p = plotxy([speed_false, speed_true, speed_blend, v_awetrim], [force, force, force, force_awetrim];
           xlabel="speed [m/s]", ylabel="force [N]",
           legend=["soft_lfc = false", "soft_lfc = true", "use_awe_trim = 0.5", "AWETrim (v03, 3 m/s)"],
           fig="winch_curve")
display(p)
