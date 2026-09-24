# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using WinchControllers, MakieControlPlots, KiteUtils
import MakieControlPlots: plotxy, Makie

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

# Set true to plot only the `soft_lfc = true` curve, e.g. to inspect it in
# isolation without the other curves cluttering the plot.
SOFT_LFC_ONLY = true

# Both curves, in the style of a motor/winch datasheet torque-speed curve: speed
# on the x-axis, force on the y-axis. Sweep force past f_high to show all
# regions: the reel-in line below f_low, the soft-saturated section, and the
# v_sat plateau above f_high. The two curves diverge in SPEED at a given force,
# not the other way round, so they need independent X series -- `plotxy` rather
# than `plot`.
# Up to 8400 N, the V3 winch's rated force (SimpleKiteControllers.jl's max-force criterion).
force = range(0.0, 8400.0, 500)
speed_false = calc_vro_soft.(Ref(wcs), force; soft_lfc=false)
speed_true  = calc_vro_soft.(Ref(wcs), force; soft_lfc=true)

# Third curve: AWETrim's own forward tension curve (`Winch.tension_curve` in
# AWETrim/src/awetrim/system/winch.py), independently re-derived here (AWETrim
# is Python, out of reach at plot time) -- T(v) = (v_raw/k_v)^2, soft-clamped
# from ABOVE at f_max first, then from BELOW at f_min (the order calc_vro_soft's
# own docstring says its inversion UNDOES in reverse). `v_raw` undoes the soft
# v_sat clamp (`Winch._undo_v_sat_clamp`, applied since AWETrim receives
# `v_sat_beta`): `v = soft_min(v_raw, v_sat, v_sat_beta)` inverted in closed
# form, its gap to v_sat floored at 1e-6 m/s as there, so the curve rises to
# f_max at v_sat instead of ending short of it. AWETrim evaluates this curve
# only as a function of speed, so it is plotted as force(speed), unlike the
# two curves above. Reel-out only (AWETrim's server rejects mode="reelin"),
# hence v >= 0.
#
# Parameters: WinchControllers' AWE_TRIM_* constants, the WinchParams
# SimpleKiteControllers.jl's `winch_from_wc` sends at 3 m/s wind, and this
# file's v_sat/v_sat_beta, which that client sends as v_max/v_sat_beta.
sp_fwd(x) = max(x, 0.0) + log1p(exp(-abs(x)))
function awetrim_tension(v; k_v = WinchControllers.AWE_TRIM_KV,
                          f_min = WinchControllers.AWE_TRIM_F_MIN,
                          f_max = WinchControllers.AWE_TRIM_F_MAX,
                          softplus_beta = WinchControllers.AWE_TRIM_BETA,
                          softminus_beta = WinchControllers.AWE_TRIM_BETA,
                          v_sat = wcs.v_sat, v_sat_beta = wcs.v_sat_beta)
    gap = max(v_sat - v, 1e-6)
    v_raw = isinf(v_sat_beta) ? v : v - log(-expm1(-v_sat_beta * gap)) / v_sat_beta
    t = (v_raw / k_v)^2
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

# Font of the LearningControl paper (TeX Gyre Termes = the Times of the Copernicus
# class), same theme as SimpleKiteControllers.jl's plot_powercurve.jl.
const PAPER_THEME = Makie.Theme(
    fonts = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
               italic = "TeX Gyre Termes Italic"),
    Axis = (; xticklabelsize = 20, yticklabelsize = 20),
)
const FIGURES_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))

if SOFT_LFC_ONLY
    power = force .* speed_true ./ 1000
    # Same relative margin below zero on both axes, so their zeros line up.
    margin = 0.05
    f_top = 1.05 * maximum(force)
    p_top = 1.05 * maximum(power)
    ylims = ((-margin * f_top, f_top), (-margin * p_top, p_top))
    # savefig re-runs the builder, so the theme must be active for the save too.
    Makie.with_theme(PAPER_THEME) do
        plot(speed_true, force, power;
             xlabel="reel-out speed [m/s]", ylabels=["force [N]", "power [kW]"],
             labels=["force", "power"],
             xticks=floor(Int, minimum(speed_true)):ceil(Int, maximum(speed_true)),
             yticks=(nothing, 5), ylims,
             labelsize=22, legendsize=16,
             disp=true, fig="winch_curve")
        mkpath(FIGURES_DIR)
        savefig(joinpath(FIGURES_DIR, "winch_curve.pdf"))
    end
else
    p = plotxy([speed_false, speed_true, speed_blend, v_awetrim], [force, force, force, force_awetrim];
               xlabel="speed [m/s]", ylabel="force [N]",
               legend=["soft_lfc = false", "soft_lfc = true", "use_awe_trim = 0.5", "AWETrim (3 m/s)"],
               fig="winch_curve")
    display(p)
end
