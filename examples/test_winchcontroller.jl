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
using WinchControllers, ControlPlots, KiteUtils

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
lg::WCLogger = WCLogger(DURATION, wcs.dt)

STARTUP = get_startup(wcs, length(lg))    
V_WIND = STARTUP .* get_triangle_wind(wcs, V_WIND_MIN, V_WIND_MAX, FREQ_WIND, length(lg))

# create and initialize winch controller 
wc = WinchController(wcs)
winch = WinchControllers.Winch(wcs, set)
f_low = wcs.f_low

for i in 1:length(lg)
    local force
    # model
    v_wind = V_WIND[i]

    v_act = get_speed(winch)
    force = calc_force(v_wind, v_act)
    set_force(winch, force)

    # controller
    v_set = calc_v_set(wc, v_act, force, f_low)
    
    # update model
    set_v_set(winch, v_set)
    
    on_timer(winch)
    on_timer(wc)

    # calculate some values for logging
    state  = get_state(wc)
    status = get_status(wc)
    force  = status[3]
    f_set  = status[4]
    if state in [0,2]
        f_err = force - f_set
        v_err = NaN
    else
        v_err = v_act - v_set
        f_err = NaN
    end
    # log the values
    log(lg; v_ro=v_act, acc=get_acc(winch), state, reset=status[1], active=status[2], 
            force, f_set, f_err, v_err, v_set_out=v_set)
end

# plot the results  
p1=plotx(lg.time, V_WIND, [lg.v_ro, lg.v_set_out], lg.f_err*0.001, lg.v_err, lg.acc, lg.force*0.001, lg.state,
    title="Winch controller test, all controllers active",
    ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
    labels=["v_wind", ["v_reel_out", "v_set_out"]],
    fig="test_winchcontroller",)

display(p1)
toc()

println("Max iterations needed: $(wcs.iter)")
