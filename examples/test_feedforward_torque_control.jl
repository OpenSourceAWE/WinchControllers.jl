# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using Timers, Statistics; tic()

# Test the speed controller in combination with the controller for the lower and upper force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test2.png
using WinchControllers, ControlPlots, KiteUtils, WinchModels

# Calculate the pulling force of the kite as function of the reel-out speed and the wind speed in the
# direction of the tether at the height of the kite. Most simplified model, massless, constant L/D,
# constant elevation angle.
function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

set = deepcopy(load_settings("system.yaml"))
wcs = WCSettings(dt=0.02)
update(wcs)
wcs.test = true

# define the simulation parameters
DURATION   = 10.0
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

# create the logger
lg::WCLogger = WCLogger(DURATION, wcs.dt, set.max_force, wcs.max_acc, wcs.damage_factor)

STARTUP = get_startup(wcs, length(lg))    
V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, length(lg))

# create and initialize winch controller 
wc = FFWinchController(wcs, set)
winch = Winch(wcs, set, wm=TorqueControlledMachine(set))
f_low = wcs.f_low

for i in 1:length(lg)
    local force, v_set
    # model
    v_wind = V_WIND[i]

    v_act = get_speed(winch)
    force = calc_force(v_wind, v_act)

    # controller
    τ_set_out = calc_τ_set(wc, v_act, force, f_low)
    
    # update model
    set_v_set(winch, v_set_out)
    
    on_timer(winch)
    on_timer(wc)

    # get values for logging
    status = get_status(wc)
    
    # log the values
    log(lg; v_ro=v_act, acc=get_acc(winch), state=get_state(wc), reset=status[1], active=status[2], force=status[3], 
        f_set=status[4], f_err=get_f_err(wc), v_err=get_v_err(wc), v_set=get_v_set(wc), v_set_out, v_set_in=get_v_set_in(wc))
end

# plot the results  
p1=plotx(lg.time, V_WIND, [lg.v_ro, lg.v_set_in], lg.f_err*0.001, lg.v_err, lg.acc, lg.force*0.001, lg.state,
    title="Winch controller test, all controllers active",
    ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
    ysize=10,
    labels=["v_wind", ["v_reel_out", "v_set_in"]],
    fig="test_winchcontroller",)

display(p1)
toc()

println("Max iterations needed: $(wcs.iter)")
println("Performance of force controllers: $(round(100*(1-f_err(lg)), digits=2)) %")
println("Performance of speed controller:  $(round(100*(1-v_err(lg)), digits=2)) %")
println("Damage:                           $(round(100*(damage(lg)), digits=2)) %")
println("Combined performance γ: $(round(100*gamma(lg), digits=2)) %")    