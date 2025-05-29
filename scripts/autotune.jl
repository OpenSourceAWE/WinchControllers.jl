# this script shall tune the controller parameters
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, KiteUtils, PRIMA, ControlPlots

function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

function plot(lg::WCLogger)
    p1=plotx(lg.time, lg.v_wind, [lg.v_ro, lg.v_set_in], lg.f_err*0.001, lg.v_err, lg.acc, lg.force*0.001, lg.state,
        title="Winch controller test, all controllers active",
        ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "f_err [kN]", "v_error [m/s]", "acc [m/s²]", "force [kN]", "state"], 
        ysize=10,
        labels=["v_wind", ["v_reel_out", "v_set_in"]],
        fig="test_winchcontroller",)
    display(p1)
end

function simulate(x::Vector{Cdouble}; return_lg::Bool = false)
    set = deepcopy(load_settings("system.yaml"))
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.test = true
    wcs.i_speed = x[1] # set the speed controller gain
    wcs.p_speed = x[2] # set the speed controller proportional gain

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
    wc = WinchController(wcs)
    winch = WinchControllers.Winch(wcs, set)
    f_low = wcs.f_low

    for i in 1:length(lg)
        local force, v_set
        # model
        v_wind = V_WIND[i]

        v_act = get_speed(winch)
        force = calc_force(v_wind, v_act)
        set_force(winch, force)

        # controller
        v_set_out = calc_v_set(wc, v_act, force, f_low)
        
        # update model
        set_v_set(winch, v_set_out)
        
        on_timer(winch)
        on_timer(wc)

        # calculate some values for logging
        state  = get_state(wc)
        status = get_status(wc)
        force  = status[3]
        f_set  = status[4]
        v_set = NaN
        if state in [0,2]
            f_err = force - f_set
            v_set_in = NaN
        else
            v_set = wc.v_set
            v_set_in = wc.sc.v_set_in
            f_err = NaN
        end
        
        # log the values
        log(lg; v_wind, v_ro=v_act, acc=get_acc(winch), state, reset=status[1], active=status[2], 
                force, f_set, f_err, v_err=get_v_err(wc), v_set, v_set_out, v_set_in)
    end
    if return_lg
        return lg
    end
    # calculate the performance metrics
    -gamma(lg)
end

function autotune()
    global info, lg
    # Define the parameters for the autotuning
    x0 = [4.0, 0.25] # initial guess for the speed controller gain
    x, info = prima(simulate, x0;
        xl = [2.0, 0.0],
        xu = [10.0, 1.0],
        rhobeg = 0.2,
        maxfun = 200
    )
    println("Autotuning results: $x")
    println("Iterations: $(info.nf)")
    println(PRIMA.reason(info.status))

    if issuccess(info)
        println("Running simulation with tuned parameters...")
        lg = simulate(x; return_lg=true)

        println("\nPerformance of force controllers: $(round(100*(1-f_err(lg)), digits=2)) %")
        println("Performance of speed controller:  $(round(100*(1-v_err(lg)), digits=2)) %")
        println("Damage:                           $(round(100*(damage(lg)), digits=2)) %")
        println("Combined performance γ: $(round(-100*info.fx, digits=2)) %")  
    else
        println("Autotuning failed: $(PRIMA.reason(info.status))")
    end 
end

autotune()