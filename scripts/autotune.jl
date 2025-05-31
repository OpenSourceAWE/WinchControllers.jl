# this script tunes the controller parameters (well, eight of them)
using Pkg
if ! ("NOMAD" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    using Test
end
using WinchControllers, KiteUtils, NOMAD, ControlPlots

LF = 2.5 # limit factor
TUNED::Bool = false
load_settings("system.yaml")

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
        set = load_settings("system_tuned.yaml")
    else
        set = load_settings("system.yaml")
    end
 
    # define the simulation parameters
    DURATION   = 10.0
    V_WIND_MAX = 9.0 # max wind speed of test wind
    V_WIND_MIN = 0.0 # min wind speed of test wind
    FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

    # create the logger
    lg::WCLogger = WCLogger(DURATION, wcs.dt, set.max_force, wcs.max_acc, wcs.damage_factor, wcs.jerk_factor)

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
        log(lg; v_wind, v_ro=v_act, acc=get_acc(winch), state=get_state(wc), reset=status[1], active=status[2], 
            force=status[3], f_set=status[4], f_err=get_f_err(wc), v_err=get_v_err(wc), v_set=get_v_set(wc), 
            v_set_out, v_set_in=get_v_set_in(wc), jerk=winch.jerk, p_dyn=winch.p_dyn)
    end
    if return_lg
        return wcs, lg
    end
    # calculate the performance metrics
    -gamma(lg)
end

function simulate_all(x::Vector; return_lg::Bool = false)
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.test = true
    wcs.i_speed = x[1] # set the speed controller gain
    wcs.p_speed = x[2] # set the speed controller proportional gain
    wcs.t_blend = x[3] # set the blending time for switching between controllers
    wcs.pf_low  = x[4] # set the speed controller gain
    wcs.if_low  = x[5] # set the speed controller proportional gain
    wcs.pf_high = x[6] # set the lower force controller gain
    wcs.if_high = x[7] # set the lower force controller integral gain
    wcs.df_high = x[8]
    simulate(wcs; return_lg)
end

function eval_fct(x)
  bb_outputs = [simulate_all(x)]
  success = bb_outputs[1] < -0.7
  count_eval = true
  return (success, count_eval, bb_outputs)
end

function autotune(max_iter=1000)
    global x, lg, TUNED, result
    if TUNED
        load_settings("system_tuned.yaml")
    else
        load_settings("system.yaml")
    end
    wcs = WCSettings(dt=0.02)
    update(wcs)
    wcs.test = true
    println("Autotuning all controllers...")
    x0 = [wcs.p_speed, wcs.i_speed, wcs.t_blend, wcs.pf_low, wcs.if_low, wcs.pf_high, wcs.if_high, maximum([1e-5, wcs.df_high])] # initial guess for the speed controller gain

    pb = NomadProblem(8,         # number of inputs of the blackbox
                      1,         # number of outputs of the blackbox
                      ["OBJ"],   # type of outputs of the blackbox
                      eval_fct;
                      lower_bound = 1/LF .* x0,
                      upper_bound = LF   .* x0)
    pb.options.max_bb_eval = max_iter 
    result = solve(pb, x0)
    x = result.x_best_feas
    
    println("Autotuning results: $x")

    println("Running simulation with tuned parameters...")
    TUNED = true
    wcs, lg = simulate_all(x; return_lg=true)

    println("\nPerformance of force controllers: $(round(100*(1-f_err(lg)), digits=2)) %")
    println("Performance of speed controller:  $(round(100*(1-v_err(lg)), digits=2)) %")
    println("Damage with jerk:                 $(round(100*(damage(lg)), digits=2)) %")
    println("Combined performance γ: $(round(-100*result.bbo_best_feas[1], digits=2)) %")  
    wcs
end

function copy_settings()
    cp("data/wc_settings.yaml", "data/wc_settings_tuned.yaml"; force=true)
    load_settings("system_tuned.yaml")
end
function change_value(lines, varname, value::Union{Integer, Float64})
    KiteUtils.change_value(lines, varname, repr(round(value, digits=6)))
end
function update_settings(wcs::WCSettings)
    lines = KiteUtils.readfile("data/wc_settings_tuned.yaml") 
    lines = change_value(lines, "i_speed:", wcs.i_speed)
    lines = change_value(lines, "p_speed:", wcs.p_speed)
    lines = change_value(lines, "t_blend:", wcs.t_blend)
    lines = change_value(lines, "pf_low:", wcs.pf_low)
    lines = change_value(lines, "if_low:", wcs.if_low)
    lines = change_value(lines, "pf_high:", wcs.pf_high)
    lines = change_value(lines, "if_high:", wcs.if_high) 
    lines = change_value(lines, "df_high:", wcs.df_high)    
    KiteUtils.writefile(lines, "data/wc_settings_tuned.yaml")
end

function tune_all(max_iter=1000)
    wcs = autotune(max_iter)
    if ! isnothing(wcs)
        update_settings(wcs)
        println()
        @info "Tuned settings saved to data/wc_settings_tuned.yaml"
    end
end

copy_settings()
tune_all(20)
tune_all()