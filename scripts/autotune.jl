# this script shall tune the controller parameters
# TODO: save the results in wc_settings_tuned.yaml
# TODO: optimize the lower force controller
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, KiteUtils, PRIMA, ControlPlots

TUNED::Bool = false

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

function simulate(wcs::WCSettings; return_lg::Bool = false)
    if TUNED
        set = load_settings("system.yaml")
    else
        set = load_settings("system_tuned.yaml")
    end
 
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
        # model
        v_wind = V_WIND[i]
        v_act = get_speed(winch)
        force = calc_force(v_wind, v_act)
        set_force(winch, force)

        # controller
        v_set_out = calc_v_set(wc, v_act, force, f_low)
        
        # update model input
        set_v_set(winch, v_set_out)
        
        # run integration step
        on_timer(winch)
        on_timer(wc)

        # get values for logging and log them
        status = get_status(wc)
        log(lg; v_wind, v_ro=v_act, acc=get_acc(winch), state=get_state(wc), reset=status[1], active=status[2], force=status[3], 
            f_set=status[4], f_err=get_f_err(wc), v_err=get_v_err(wc), v_set=get_v_set(wc), v_set_out, v_set_in=get_v_set_in(wc))
    end
    if return_lg
        return wcs, lg
    end
    # calculate the performance metrics
    -gamma(lg)
end


function simulate_sc(x::Vector{Cdouble}; return_lg::Bool = false)
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.test = true
    wcs.i_speed = x[1] # set the speed controller gain
    wcs.p_speed = x[2] # set the speed controller proportional gain
    wcs.t_blend = x[3] # set the blending time for switching between controllers
    simulate(wcs; return_lg)
end

function simulate_lfc(x::Vector{Cdouble}; return_lg::Bool = false)
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.test = true
    wcs.pf_low = x[1] # set the lower force controller gain
    wcs.if_low = x[2] # set the lower force controller integral gain
    simulate(wcs; return_lg)
end

function autotune(controller::WinchControllerState)
    global x, info, lg, TUNED
    if TUNED
        load_settings("system_tuned.yaml")
    else
        load_settings("system.yaml")
    end
 
    if controller == wcsSpeedControl
        println("Autotuning speed controller...")
        # Define the parameters for the autotuning
        x0 = [4.0, 0.25, 0.2] # initial guess for the speed controller gain
        x, info = bobyqa(simulate_sc, x0;
            xl = [2.0, 0.0, 0.02],
            xu = [10.0, 0.5, 0.4],
            rhobeg = 0.1,
            npt=10,
            maxfun = 500
        )
    elseif controller == wcsLowerForceLimit
        println("Autotuning lower force limit controller...")
        # Define the parameters for the autotuning
        x0 = [0.00014, 0.01125] # initial guess for the speed controller gain
        x, info = bobyqa(simulate_lfc, x0;
            xl = 0.25 .* x0,
            xu = 2.0 .* x0,
            rhobeg = minimum(x0)/4,
            maxfun = 500
        )
    elseif controller == wcsUpperForceControl
        println("Autotuning upper force control...")
    else
        error("Unknown controller state: $controller")
        return
    end

    println("Autotuning results: $x")
    println("Iterations: $(info.nf)")
    println(PRIMA.reason(info.status))

    if issuccess(info)
        println("Running simulation with tuned parameters...")
        TUNED = true
        if controller == wcsSpeedControl
            wcs, lg = simulate_sc(x; return_lg=true)
        elseif controller == wcsLowerForceLimit
            wcs, lg = simulate_lfc(x; return_lg=true)
        elseif controller == wcsUpperForceControl
            wcs, lg = simulate_ufc(x; return_lg=true)
        end

        println("\nPerformance of force controllers: $(round(100*(1-f_err(lg)), digits=2)) %")
        println("Performance of speed controller:  $(round(100*(1-v_err(lg)), digits=2)) %")
        println("Damage:                           $(round(100*(damage(lg)), digits=2)) %")
        println("Combined performance γ: $(round(-100*info.fx, digits=2)) %")  
        return wcs
    else
        println("Autotuning failed: $(PRIMA.reason(info.status))")
        return nothing
    end 
end

function copy_settings()
    cp("data/wc_settings.yaml", "data/wc_settings_tuned.yaml"; force=true)
end
function change_value(lines, varname, value::Union{Integer, Float64})
    KiteUtils.change_value(lines, varname, repr(round(value, digits=4)))
end
function update_settings(wcs::WCSettings)
    lines = KiteUtils.readfile("data/wc_settings_tuned.yaml") 
    lines = change_value(lines, "i_speed:", wcs.i_speed)
    lines = change_value(lines, "p_speed:", wcs.p_speed)
    lines = change_value(lines, "t_blend:", wcs.t_blend)
    lines = change_value(lines, "pf_low:", wcs.pf_low)
    lines = change_value(lines, "if_low:", wcs.if_low)
    KiteUtils.writefile(lines, "data/wc_settings_tuned.yaml")
end

wcs = autotune(wcsSpeedControl)
if ! isnothing(wcs)
    copy_settings()
    update_settings(wcs)
    println()
    wcs = autotune(wcsLowerForceLimit)
    if ! isnothing(wcs)
        update_settings(wcs)
    end
    @info "Tuned settings saved to data/wc_settings_tuned.yaml"
end


# autotune(wcsUpperForceControl)