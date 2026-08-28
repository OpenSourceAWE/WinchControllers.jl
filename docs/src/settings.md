```@meta
CurrentModule = WinchControllers
```
# Winchcontroller Settings

## Introductions
Three `.yaml` files are needed to configure the winch controller:
1. system.yaml
2. settings.yaml
3. wc_settings.yaml

The first file lists the `yaml` files that shall be used for the current project. The second file defines the model parameters. The third file defines
the parameters of the winch controller. All of these files must be stored in
the `data` directory.

Example for a `system.yaml` file:
```yaml
system:
    sim_settings:  "settings.yaml"       # model and simulator settings
    wc_settings:   "wc_settings.yaml"    # winch controller settings
```

The following sequence of commands will load the model and the winch controller settings for a given project:
```julia
using WinchControllers, KiteUtils

set_data_path("data") # set the path where the yaml files are stored
set = load_settings("system.yaml")
wcs = WCSettings()
update(wcs)
``` 
Both, `set` and `wcs` are simple structs. You can see the content of the struct by typing the name of one of these variables. If you type `wcs. <TAB><TAB>` you can see all elements of the `wcs` struct.

## The file wc_settings.yaml

```yaml
wc_settings:
    dt: 0.02         # time step of the winch controller
    test: false      # if true, a simplified formula for the calculation of v_set will be used
    fac: 0.25        # factor for I and P of lower force controller
    max_iter: 100    # max iterations limit for the PID solvers
    iter: 0          # actual max iterations of the PID solvers
    t_startup: 0.25  # startup time for soft start  
    t_blend: 0.1     # blending time of the mixers in seconds
    v_sat_error: 1.0 # limit of reel-out speed error, used by the input sat block of the speed controller
    v_sat: 8.0       # limit of reel-out speed, used by the output sat block of the speed controller
    v_ri_max: 8.0    # maximal reel-in speed [m/s]
    p_speed: 0.125   # P value of the speed controller
    i_speed: 4.0     # I value of the speed controller
    kb_speed: 4.0    # back calculation constant for the anti-windup loop of the speed controller
    kt_speed: 5.0    # tracking constant of the speed controller
    vf_max: 2.75     # reel-out velocity where the set force should reach it's maximum
    pf_low: 0.00014  # P constant of the lower force controller 0.013,  0.00014 also works
    if_low: 0.01125  # I constant of the lower force controller 0.0225, 0.01125 also works
    df_low: 0.0      # D constant of lower force controller 0.000017
    nf_low:  0       # filter constant n of upper force controller
    kbf_low: 8.0     # back calculation constant for the anti-windup loop of the lower force controller
    ktf_low: 8.0     # tracking constant of the lower force controller
    f_low: 350       # lower force limit [N]
    f_reelin: 700    # set force for reel-in phase [N]
    f_high: 3800     # upper force limit [N]
    pf_high: 0.0002304 # P constant of upper force controller
    if_high: 0.012     # I constant of upper force controller
    df_high: 2.04e-5  # D constant of upper force controller
    nf_high: 15.0    # filter constant n of upper force controller
    kbf_high: 1.0    # back calculation constant for the anti-windup loop of the upper force controller
    ktf_high: 10.0   # tracking constant of the upper force controller
    winch_iter: 10   # iterations of the winch model
    max_acc: 8.0     # maximal acceleration of the winch (derivative of the set value of the reel-out speed)
    damage_factor: 0.05 # damage at max acceleration for jerk_factor=0
    jerk_factor: 0.90   # factor for the jerk (derivative of the acceleration), 0..1 
    kv: 0.06         # proportional factor of the square root law, see function calc_vro

    # --- Soft force limiting (REEL_OUT mode), see calc_vro_soft --------------
    force_limit: "hard"    # "hard" (default) or "soft"; see WCSettings.force_limit
    softplus_beta: 1e-3     # "soft" only: corner sharpness of the UPPER limit [1/N]
    softminus_beta: 1e-3    # "soft" only: corner sharpness of the LOWER limit [1/N]
    force_limit_tau: 1.0    # "soft" only: low-pass time constant on the force [s]
    force_limit_tau_rise: NaN  # "soft" only: same, while the force is RISING [s]; NaN = symmetric
    soft_reel_in: false     # "soft" only: also replace the LowerForceController with a reel-in line
    v_reel_in: -2.0         # soft_reel_in only: reel-in speed at zero force [m/s]
    reel_in_beta: 20.0      # soft_reel_in only: sharpness of the smooth line/curve handover [s/m]
```

`soft_reel_in` draws a straight line from `(0, v_reel_in)` to `(f_low, 0)` — the whole
physically valid range below `f_low`, since force is never negative — so there is no
separate ramp-width setting; the slope is `-v_reel_in / f_low`. It needs
`softminus_beta * f_low >= 8` (the shipped defaults above fail this by roughly an order
of magnitude): the lower corner must be sharp enough that the tension curve itself
reaches 0 at `f_low`, or a dead band of zero speed opens between the line and the curve
once the `LowerForceController` is switched off. `WinchController` validates this, and
every other `soft_reel_in` invariant, at construction time.

Above `f_low` the line hands over to the reel-out tension curve via a smooth minimum
(`soft_min`, sharpness `reel_in_beta`) rather than a hard `min`, so the handover has no
kink. Larger `reel_in_beta` is sharper; the curve sits `log(2) / reel_in_beta` below the
hard minimum right at the crossing, and matches it closely away from the crossing —
including near the `(f_low, 0)` anchor itself, which the smoothing does not disturb.
